#![allow(clippy::too_many_arguments)]
use std::time::Duration;

use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::gltf::GltfExtras;
use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_inspector_egui::bevy_egui::EguiContexts;
use bevy_mod_raycast::prelude::*;

mod asset_reader;
mod orbit_camera;

#[derive(Resource)]
struct MapFile(String);

fn main() {
    let file = std::env::args().nth(1).unwrap_or_default();

    if std::env::args().len() < 2 || file.starts_with('-') {
        println!("Usage: {} scene.gltf", std::env::args().next().unwrap());
        return;
    }

    App::new()
        .add_plugins((
            asset_reader::AssetReaderPlugin,
            DefaultPlugins,
            orbit_camera::OrbitCameraPlugin,
            FrameTimeDiagnosticsPlugin,
            LogDiagnosticsPlugin {
                wait_duration: Duration::from_millis(1000),
                filter: Some(vec![FrameTimeDiagnosticsPlugin::FPS]),
                ..default()
            },
            bevy_inspector_egui::quick::WorldInspectorPlugin::default()
                .run_if(input_toggle_active(false, KeyCode::F12)),
        ))
        .add_systems(Startup, (spawn_environment, spawn_scene))
        .add_systems(Update, mouse_move)
        .insert_resource(ClearColor(Color::rgb(0., 0., 0.)))
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 300.,
        })
        .insert_resource(Msaa::default())
        .insert_resource(MapFile(file))
        .init_resource::<Highlighted>()
        .run();
}

fn spawn_environment(mut commands: Commands) {
    let camera_transform = Transform::from_xyz(0., 540., 720.).looking_at(Vec3::new(0., 0., 267.), Vec3::Y);
    let camera_rotation = camera_transform.rotation.to_euler(EulerRot::YXZ);

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(0.7, 0.7, 1.0).looking_at(Vec3::new(0.0, 0.3, 0.0), Vec3::Y),
            ..default()
        },
        orbit_camera::OrbitCamera {
            gimbal_x: -camera_rotation.0,
            gimbal_y: -camera_rotation.1,
            distance: camera_transform.translation.length(),
            ..default()
        },
    ));

    commands.spawn(DirectionalLightBundle {
        transform: Transform::from_rotation(Quat::from_rotation_x(-0.7)),
        directional_light: DirectionalLight {
            illuminance: 5500.,
            shadows_enabled: false,
            ..default()
        },
        ..default()
    });
}

fn spawn_scene(mut commands: Commands, asset_server: Res<AssetServer>, map_file: Res<MapFile>) {
    commands.spawn(SceneBundle {
        scene: asset_server.load(format!("{}#Scene0", map_file.0)),
        ..default()
    });
}

#[derive(Resource, Default)]
struct Highlighted {
    last_time: Duration,
    current: Option<Entity>,
    entities: Vec<(Entity, Handle<StandardMaterial>)>,
}

fn mouse_move(
    window_query: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut raycast: Raycast,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut material_query: Query<&mut Handle<StandardMaterial>>,
    mut highlighted: ResMut<Highlighted>,
    time: Res<Time<Real>>,
    extras: Query<(Entity, &GltfExtras)>,
    parents: Query<&Parent>,
    children: Query<&Children>,
    egui_ctx_query: Query<(), With<PrimaryWindow>>,
    mut egui_contexts: EguiContexts,
) {
    if !egui_ctx_query.is_empty() && egui_contexts.ctx_mut().wants_pointer_input() {
        return;
    }

    if highlighted.last_time + Duration::from_millis(100) > time.elapsed() {
        return;
    }

    let entity = (|| {
        let window = window_query.get_single().ok()?;
        let (camera, gxform) = camera_query.get_single().ok()?;
        let cursor_position = window.cursor_position()?;
        let cursor_ray = camera.viewport_to_world(gxform, cursor_position)?;

        let hit = raycast.cast_ray(cursor_ray, &RaycastSettings::default());
        let entity = hit.first()?.0;

        if extras.get(entity).is_ok() {
            return Some(entity);
        }

        #[allow(clippy::manual_find)]
        for parent in parents.iter_ancestors(entity) {
            if extras.get(parent).is_ok() {
                return Some(parent);
            }
        }

        None
    })();

    if highlighted.current == entity {
        return;
    }

    highlighted.current = entity;
    highlighted.last_time = time.elapsed();

    for (entity, old_material) in std::mem::take(&mut highlighted.entities) {
        if let Ok(mut handle) = material_query.get_mut(entity) {
            *handle = old_material;
        }
    }

    if let Some(entity) = entity {
        for child in children.iter_descendants(entity) {
            if let Ok(mut handle) = material_query.get_mut(child) {
                highlighted.entities.push((child, handle.clone()));

                let mut material = materials.get_mut(&*handle).expect("material not found").clone();
                material.base_color = Color::DARK_GREEN;
                *handle = materials.add(material);
            }
        }
    }
}
