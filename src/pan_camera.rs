// 2d camera that you can control with mouse

use bevy::input::gamepad::GamepadEvent;
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
// use bevy_inspector_egui::bevy_egui::EguiContexts;

pub struct PanCamera2dPlugin;

impl Plugin for PanCamera2dPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<PanCamera>();
        app.add_systems(Update, apply_camera_controls);
        app.add_systems(Update, update_camera.after(apply_camera_controls));
    }
}

#[derive(Bundle, Default)]
pub struct PanCamera2dBundle {
    pub pan_camera: PanCamera,
    pub camera2d: Camera2d,
}

#[derive(Debug, Component, Reflect)]
pub struct PanCamera {
    pub zoom: f32,
    pub zoom_sensitivity: f32,
    pub pan_sensitivity: f32,
    pub min_zoom: f32,
    pub max_zoom: f32,
    pub target: Vec2,
    pub active: bool,
}

impl Default for PanCamera {
    fn default() -> Self {
        Self {
            zoom: 1.,
            zoom_sensitivity: 0.1,
            pan_sensitivity: 1.,
            min_zoom: 0.,
            max_zoom: f32::INFINITY,
            target: Vec2::ZERO,
            active: true,
        }
    }
}

#[derive(Default)]
// We want to allow the camera to be controlled from all gamepads,
// so we can't use Res<Axis<GamepadAxis>> specific to a gamepad.
// This state is derived from latest event of each gamepad, and is
// zeroed out when any gamepad disconnects.
struct GamepadState {
    left_stick_x: f32,
    right_stick_x: f32,
    left_stick_y: f32,
    right_stick_y: f32,
    left_trigger: f32,
    right_trigger: f32,
}

#[allow(clippy::too_many_arguments)]
fn apply_camera_controls(
    mut scroll_events: EventReader<MouseWheel>,
    mut move_events: EventReader<MouseMotion>,
    mut gamepad_events: EventReader<GamepadEvent>,
    mut gamepad_state: Local<GamepadState>,
    time: Res<Time>,
    buttons: Res<ButtonInput<MouseButton>>,
    // mut egui_contexts: EguiContexts,
    mut camera_query: Query<(&mut PanCamera, &Camera, &GlobalTransform)>,
    window_query: Query<&Window, With<PrimaryWindow>>,
) {
    // if let Some(egui_ctx) = egui_contexts.try_ctx_mut() {
    //     if egui_ctx.wants_pointer_input() { return; }
    // }

    let mouse_position = std::cell::LazyCell::<Option<Vec2>, _>::new(|| {
        let window = window_query.get_single().ok()?;
        let (_, camera, gxform) = camera_query.get_single().ok()?;
        let cursor_position = window.cursor_position()?;
        let cursor_ray = camera.viewport_to_world(gxform, cursor_position).ok()?;
        let position = cursor_ray.intersect_plane(Vec3::ZERO, InfinitePlane3d::new(Vec3::Z))
            .map(|pos| cursor_ray.origin + cursor_ray.direction * pos);
        position.map(|pos| Vec2::new(pos.x, pos.y))
    });

    #[derive(Debug)]
    enum MyEvent {
        Zoom((f32, Option<Vec2>)),
        Pan((f32, f32)),
    }

    let mut events = vec![];

    for ev in scroll_events.read() {
        events.push(MyEvent::Zoom((ev.y, *mouse_position)));
    }

    if buttons.pressed(MouseButton::Right) {
        for ev in move_events.read() {
            events.push(MyEvent::Pan((ev.delta.x, ev.delta.y)));
        }
    }

    for ev in gamepad_events.read() {
        match ev {
            GamepadEvent::Axis(ev) => {
                match ev.axis {
                    GamepadAxis::LeftStickX => gamepad_state.left_stick_x = ev.value,
                    GamepadAxis::LeftStickY => gamepad_state.left_stick_y = ev.value,
                    GamepadAxis::RightStickX => gamepad_state.right_stick_x = ev.value,
                    GamepadAxis::RightStickY => gamepad_state.right_stick_y = ev.value,
                    _ => {}
                }
            }

            GamepadEvent::Button(ev) => {
                match ev.button {
                    GamepadButton::LeftTrigger | GamepadButton::LeftTrigger2 =>
                        gamepad_state.left_trigger = ev.value,
                    GamepadButton::RightTrigger | GamepadButton::RightTrigger2 =>
                        gamepad_state.right_trigger = ev.value,
                    _ => {}
                }
            }

            GamepadEvent::Connection(ev) => {
                if ev.disconnected() {
                    // avoid rotating forever if gamepad disconnects
                    *gamepad_state = default();
                }
            }
        }
    }

    let gamepad_axis_multiplier = time.delta_secs() * 1000.;
    let gamepad_zoom_multiplier = time.delta_secs() * 40.;

    if gamepad_state.left_stick_x != 0. || gamepad_state.left_stick_y != 0. {
        events.push(MyEvent::Pan((
            -gamepad_state.left_stick_x.powi(3) * gamepad_axis_multiplier,
            gamepad_state.left_stick_y.powi(3) * gamepad_axis_multiplier,
        )));
    }

    if gamepad_state.right_trigger - gamepad_state.left_trigger != 0. {
        events.push(MyEvent::Zoom((
            (gamepad_state.right_trigger.powi(3) - gamepad_state.left_trigger.powi(3))
                * gamepad_zoom_multiplier,
            None,
        )));
    }

    if events.is_empty() { return; }

    let mut camcount = 0;
    for (mut camera, _, _) in camera_query.iter_mut() {
        if !camera.active { return; }
        camcount += 1;

        for event in events.iter() {
            match event {
                MyEvent::Zoom((dy, at)) => {
                    let at = at.unwrap_or(camera.target);
                    let old_zoom = camera.zoom;

                    camera.zoom = (camera.zoom * ((1. + camera.zoom_sensitivity).powf(-dy)))
                        .clamp(camera.min_zoom, camera.max_zoom);

                    camera.target = camera.target - (at - camera.target) * (camera.zoom / old_zoom - 1.);
                }
                MyEvent::Pan((dx, dy)) => {
                    camera.target = camera.target + Vec2::new(-*dx, *dy) * camera.pan_sensitivity * camera.zoom;
                }
            }
        }
    }

    if camcount > 1 {
        bevy::log::warn!("found {} active PanCameras, only 1 expected", camcount);
    }
}

fn update_camera(
    mut camera_query: Query<(&mut PanCamera, &mut OrthographicProjection, &mut Transform)>,
) {
    for (camera, mut projection, mut transform) in camera_query.iter_mut() {
        if !camera.active { return; }

        transform.translation = Vec3::new(camera.target.x, camera.target.y, 0.);
        projection.scale = camera.zoom;
    }
}
