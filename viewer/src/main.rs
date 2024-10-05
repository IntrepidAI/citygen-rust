#![allow(clippy::too_many_arguments)]
use std::time::Duration;

use bevy::color::palettes::css::DARK_GREEN;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::gltf::GltfExtras;
use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_inspector_egui::bevy_egui::EguiContexts;
use bevy_mod_raycast::prelude::*;

use crate::city_gen::WayType;
use crate::pan_camera::{PanCamera, PanCamera2dBundle};

mod city_gen;
mod pan_camera;

#[derive(Resource)]
struct RoadNetworkResource(crate::city_gen::RoadNetwork);

impl RoadNetworkResource {
    fn new(segment_count_limit: usize) -> Self {
        Self(city_gen::generate_segments(segment_count_limit))
    }

    fn to_lua(&self) -> String {
        let mut result = vec![];

        for edge in self.0.graph.edge_indices() {
            let (a, b) = self.0.graph.edge_endpoints(edge).unwrap();
            let a = &self.0.graph[a];
            let b = &self.0.graph[b];

            let edge_weight = self.0.graph.edge_weight(edge).unwrap();
            let is_highway = match edge_weight {
                WayType::Highway => true,
                WayType::Normal => false,
            };

            result.push(format!(
                "map.spawn_road {{ src = {{ {}, {} }}, dst = {{ {}, {} }}, highway = {} }}",
                a.position.x / 4., a.position.y / 4., b.position.x / 4., b.position.y / 4., is_highway
            ));
        }

        result.join("\n")
    }
}

// #[derive(Resource)]
// struct Rapier()

fn main() {
    // let file = std::env::args().nth(1).unwrap_or_default();

    // if std::env::args().len() < 2 || file.starts_with('-') {
    //     println!("Usage: {} scene.gltf", std::env::args().next().unwrap());
    //     return;
    // }

    App::new()
        .add_plugins((
            DefaultPlugins,
            // orbit_camera::OrbitCameraPlugin,
            pan_camera::PanCamera2dPlugin,
            FrameTimeDiagnosticsPlugin,
            LogDiagnosticsPlugin {
                wait_duration: Duration::from_millis(1000),
                filter: Some(vec![FrameTimeDiagnosticsPlugin::FPS]),
                ..default()
            },
            bevy_inspector_egui::quick::WorldInspectorPlugin::default()
                .run_if(input_toggle_active(false, KeyCode::F12)),
        ))
        .add_systems(Startup, spawn_environment)
        .add_systems(Update, mouse_move)
        .add_systems(Update, show_gizmos)
        .add_systems(Update, ui)
        .insert_resource(ClearColor(Color::srgb(0., 0., 0.)))
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 300.,
        })
        .insert_resource(Msaa::default())
        // .insert_resource(MapFile(file))
        .insert_resource(RoadNetworkResource::new(400))
        .init_resource::<Highlighted>()
        .run();
}

fn ui(
    mut commands: Commands,
    mut egui_contexts: EguiContexts,
    mut limit: Local<usize>,
    mut limit_txt: Local<String>,
) {
    let Some(ctx) = egui_contexts.try_ctx_mut() else { return };

    bevy_inspector_egui::egui::Window::new("")
        .title_bar(false)
        .auto_sized()
        .show(ctx, |ui| {
            if ui.text_edit_singleline(&mut *limit_txt).changed() {
                if let Ok(value) = limit_txt.parse() {
                    *limit = value;
                }
            }

            if ui.button("regenerate").clicked() {
                *limit += 1;
                *limit_txt = limit.to_string();
                commands.insert_resource(RoadNetworkResource::new(*limit));
            }

            if ui.button("write to file").clicked() {
                commands.add(|world: &mut World| {
                    let network = world.get_resource::<RoadNetworkResource>().unwrap();
                    let mut file = std::fs::File::create("map.lua").unwrap();
                    let text = network.to_lua();
                    use std::io::Write;
                    file.write_all(text.as_bytes()).unwrap();
                });
            }
        });
}

fn show_gizmos(
    window_query: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut gizmos: Gizmos,
    network: Res<RoadNetworkResource>,
    mut egui_contexts: EguiContexts,
) {
    let Some(ctx) = egui_contexts.try_ctx_mut() else { return };

    crate::city_gen::draw_segments(&mut gizmos, &network.0);

    let intersect = (|| {
        let window = window_query.get_single().ok()?;
        let (camera, gxform) = camera_query.get_single().ok()?;
        let cursor_position = window.cursor_position()?;
        let cursor_ray = camera.viewport_to_world(gxform, cursor_position)?;
        cursor_ray.intersect_plane(Vec3::ZERO, InfinitePlane3d::new(Vec3::Z))
            .map(|pos| cursor_ray.origin + cursor_ray.direction * pos)
    })();

    // if let Some(intersect) = intersect {
    //     let p1 = Vec2::new(10000., 10000.);
    //     let p2 = intersect.xy();
    //     if let Some((p2, _)) = city_gen::local_constraints_apply(p1, p2, &network.0.graph, &network.0.tree) {
    //         gizmos.line_2d(Vec2::new(10000., 10000.), p2, Color::RED);
    //     }
    // }

    bevy_inspector_egui::egui::Window::new("x")
        .title_bar(false)
        .auto_sized()
        .show(ctx, |ui| {
            ui.label(format!("{intersect:?}"));
            if let Some(intersect) = intersect {
                let intersect = intersect.truncate();
                let population = noise::permutationtable::PermutationTable::new(1);//rand.generate());
                ui.label(format!("{:?}", city_gen::sample_population(&population, intersect)));
            }
        });
}

fn spawn_environment(mut commands: Commands) {
    // let camera_transform = Transform::from_xyz(0., 540.*30., 720.*30.).looking_at(Vec3::new(0., 0., 267.), Vec3::Y);
    // let camera_rotation = camera_transform.rotation.to_euler(EulerRot::YXZ);

    // commands.spawn((
    //     Camera3dBundle::default(),
    //     orbit_camera::OrbitCamera {
    //         gimbal_x: -camera_rotation.0,
    //         gimbal_y: -camera_rotation.1,
    //         distance: camera_transform.translation.length(),
    //         ..default()
    //     },
    // ));

    commands.spawn(PanCamera2dBundle {
        pan_camera: PanCamera {
            zoom: 30.,
            ..default()
        },
        ..default()
    });

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
    mut egui_contexts: EguiContexts,
) {
    let Some(ctx) = egui_contexts.try_ctx_mut() else { return };
    if ctx.wants_pointer_input() { return };

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
                material.base_color = DARK_GREEN.into();
                *handle = materials.add(material);
            }
        }
    }
}
