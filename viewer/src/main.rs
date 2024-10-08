#![allow(clippy::too_many_arguments)]
use std::time::Duration;

use bevy::color::palettes::css::DARK_GREEN;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::gltf::GltfExtras;
use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_inspector_egui::bevy_egui::EguiContexts;
use bevy_inspector_egui::egui::{Color32, DragValue, Slider, Spinner};
use bevy_mod_raycast::prelude::*;
use city_gen::GeneratorConfig;

use crate::city_gen::WayType;
use crate::pan_camera::{PanCamera, PanCamera2dBundle};

const GENERATOR_ITERATION_TIMEOUT: Duration = Duration::from_millis(50);

mod city_gen;
mod pan_camera;

#[derive(Resource, Default)]
struct CurrentGeneratorConfig(GeneratorConfig);

#[derive(Resource)]
struct RoadNetworkResource(crate::city_gen::RoadNetwork);

impl RoadNetworkResource {
    fn new(config: GeneratorConfig) -> Self {
        // Self(city_gen::generate_segments(config))
        let mut network = city_gen::RoadNetwork::new(config);
        network.generate(GENERATOR_ITERATION_TIMEOUT / 4);
        Self(network)
    }

    fn to_lua(&self) -> String {
        let mut result = vec![
            r#"local map = require("map")"#.to_string(),
        ];

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
                a.position.x, a.position.y, b.position.x, b.position.y, false//is_highway
            ));
        }

        for building in &self.0.buildings {
            result.push(format!(
                "map.spawn {{ mesh = \"{}\", position = {{ x = {}, y = {} }}, rotation = {{ yaw = {} }} }}",
                building.mesh, building.position.x, building.position.y, building.orientation.as_degrees()
            ));
        }

        for object in &self.0.objects {
            result.push(format!(
                "map.spawn {{ mesh = \"{}\", position = {{ x = {}, y = {} }}, rotation = {{ yaw = {} }} }}",
                object.mesh, object.position.x, object.position.y, object.orientation.as_degrees()
            ));
        }

        for goal in &self.0.checkpoints {
            result.push(format!(
                "map.spawn_goal {{ position = {{ x = {}, y = {} }}, active = true }}",
                goal.x, goal.y
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
        .add_systems(Update, |mut network: ResMut<RoadNetworkResource>| {
            network.0.generate(GENERATOR_ITERATION_TIMEOUT);
        })
        .insert_resource(ClearColor(Color::srgb(0., 0., 0.)))
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 300.,
        })
        .insert_resource(Msaa::default())
        // .insert_resource(MapFile(file))
        .insert_resource(CurrentGeneratorConfig::default())
        .insert_resource(RoadNetworkResource::new(GeneratorConfig::default()))
        .init_resource::<Highlighted>()
        .run();
}

struct UiSettings {
    auto_generate: bool,
    filename: String,
}

impl Default for UiSettings {
    fn default() -> Self {
        Self {
            auto_generate: true,
            filename: "map.lua".to_string(),
        }
    }
}

fn ui(
    mut commands: Commands,
    mut egui_contexts: EguiContexts,
    mut settings: ResMut<CurrentGeneratorConfig>,
    mut ui_settings: Local<UiSettings>,
    window_query: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut network: ResMut<RoadNetworkResource>,
) {
    let Some(ctx) = egui_contexts.try_ctx_mut() else { return };

    bevy_inspector_egui::egui::Window::new("")
        .title_bar(false)
        .auto_sized()
        .max_width(250.)
        .show(ctx, |ui| {
            let mut changed = false;

            ui.horizontal(|ui| {
                ui.label("seed:");
                changed |= ui.add(DragValue::new(&mut settings.0.seed).speed(1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("segments:");
                changed |= ui.add(DragValue::new(&mut settings.0.segment_count_limit).speed(1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("checkpoints:");
                changed |= ui.add(DragValue::new(&mut settings.0.checkpoint_count).speed(1.0)).changed();
            });

            ui.horizontal(|ui| {
                changed |= ui.checkbox(&mut settings.0.generate_buildings, "generate buildings").changed();
                ui.label("(slow)");
            });

            ui.horizontal(|ui| {
                changed |= ui.checkbox(&mut settings.0.generate_trees, "generate trees").changed();
                ui.label("(slow)");
            });

            ui.horizontal(|ui| {
                ui.label("branch angle deviation:");
                changed |= ui.drag_angle(&mut settings.0.branch_angle_deviation).changed();
            });

            ui.horizontal(|ui| {
                ui.label("straight angle deviation:");
                changed |= ui.drag_angle(&mut settings.0.straight_angle_deviation).changed();
            });

            ui.horizontal(|ui| {
                ui.label("minimum intersection deviation:");
                changed |= ui.drag_angle(&mut settings.0.minimum_intersection_deviation).changed();
            });

            ui.horizontal(|ui| {
                ui.label("normal segment length:");
                changed |= ui.add(DragValue::new(&mut settings.0.normal_segment_length).speed(1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("highway segment length:");
                changed |= ui.add(DragValue::new(&mut settings.0.highway_segment_length).speed(1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("max snap distance:");
                changed |= ui.add(DragValue::new(&mut settings.0.max_snap_distance).speed(1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("normal branch probability:");
                changed |= ui.add(Slider::new(&mut settings.0.normal_branch_probability, 0.0..=1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("highway branch probability:");
                changed |= ui.add(Slider::new(&mut settings.0.highway_branch_probability, 0.0..=1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("normal branch pop threshold:");
                changed |= ui.add(Slider::new(&mut settings.0.normal_branch_population_threshold, 0.0..=1.0)).changed();
            });

            ui.horizontal(|ui| {
                ui.label("highway branch pop threshold:");
                changed |= ui.add(Slider::new(&mut settings.0.highway_branch_population_threshold, 0.0..=1.0)).changed();
            });

            ui.horizontal(|ui| {
                if network.0.is_generating() {
                    if ui.button("stop").clicked() {
                        network.0.stop_generating();
                    }

                    ui.add(Spinner::new().color(Color32::DARK_GRAY));
                } else {
                    if ui.button("generate").clicked() || (ui_settings.auto_generate && changed) {
                        // *limit += 1;
                        // *limit_txt = limit.to_string();
                        commands.insert_resource(RoadNetworkResource::new(settings.0.clone()));
                    }

                    changed |= ui.checkbox(&mut ui_settings.auto_generate, "auto").changed();
                }
            });

            ui.separator();

            ui.text_edit_singleline(&mut ui_settings.filename);

            if ui.button("write to file").clicked() {
                let filename = ui_settings.filename.clone();
                commands.add(|world: &mut World| {
                    let network = world.get_resource::<RoadNetworkResource>().unwrap();
                    let mut file = std::fs::File::create(filename).unwrap();
                    let text = network.to_lua();
                    use std::io::Write;
                    file.write_all(text.as_bytes()).unwrap();
                });
            }

            ui.separator();

            ui.horizontal(|ui| {
                let mouse_position: Option<Vec2> = (|| {
                    let window = window_query.get_single().ok()?;
                    let (camera, gxform) = camera_query.get_single().ok()?;
                    let cursor_position = window.cursor_position()?;
                    let cursor_ray = camera.viewport_to_world(gxform, cursor_position)?;
                    let position = cursor_ray.intersect_plane(Vec3::ZERO, InfinitePlane3d::new(Vec3::Z))
                        .map(|pos| cursor_ray.origin + cursor_ray.direction * pos);
                    position.map(|pos| Vec2::new(pos.x, pos.y))
                })();

                ui.label("mouse at");
                if let Some(mouse_position) = mouse_position {
                    ui.label(format!(
                        "x: {:.2}, y: {:.2}",
                        mouse_position.x, mouse_position.y
                    ));
                } else {
                    ui.label("n/a");
                }
            });
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

    // let intersect = (|| {
    //     let window = window_query.get_single().ok()?;
    //     let (camera, gxform) = camera_query.get_single().ok()?;
    //     let cursor_position = window.cursor_position()?;
    //     let cursor_ray = camera.viewport_to_world(gxform, cursor_position)?;
    //     cursor_ray.intersect_plane(Vec3::ZERO, InfinitePlane3d::new(Vec3::Z))
    //         .map(|pos| cursor_ray.origin + cursor_ray.direction * pos)
    // })();

    // if let Some(intersect) = intersect {
    //     let p1 = Vec2::new(10000., 10000.);
    //     let p2 = intersect.xy();
    //     if let Some((p2, _)) = city_gen::local_constraints_apply(p1, p2, &network.0.graph, &network.0.tree) {
    //         gizmos.line_2d(Vec2::new(10000., 10000.), p2, Color::RED);
    //     }
    // }

    // bevy_inspector_egui::egui::Window::new("x")
    //     .title_bar(false)
    //     .auto_sized()
    //     .show(ctx, |ui| {
    //         ui.label(format!("{intersect:?}"));
    //         if let Some(intersect) = intersect {
    //             let intersect = intersect.truncate();
    //             let population = noise::permutationtable::PermutationTable::new(1);//rand.generate());
    //             ui.label(format!("{:?}", city_gen::sample_population(&population, intersect)));
    //         }
    //     });
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
    // mut commands: Commands,
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
    // mut network: ResMut<RoadNetworkResource>,
) {
    let Some(ctx) = egui_contexts.try_ctx_mut() else { return };
    if ctx.wants_pointer_input() { return };

//     {
//         // let mut network = RoadNetworkResource::new(1);

//         let mouse_position: Option<Vec2> = (|| {
//             let window = window_query.get_single().ok()?;
//             let (camera, gxform) = camera_query.get_single().ok()?;
//             let cursor_position = window.cursor_position()?;
//             let cursor_ray = camera.viewport_to_world(gxform, cursor_position)?;
//             let position = cursor_ray.intersect_plane(Vec3::ZERO, InfinitePlane3d::new(Vec3::Z))
//                 .map(|pos| cursor_ray.origin + cursor_ray.direction * pos);
//             position.map(|pos| Vec2::new(pos.x, pos.y))
//         })();
//         let Some(mouse_position) = mouse_position else { return };

//         let shape_pos = Isometry2::from_parts(Translation2::new(mouse_position.x, mouse_position.y), Default::default());
//         let size = Vec2::new(30., 30.);
//         let shape = rapier2d::parry::shape::Cuboid::new(Vector2::new(size.x / 2., size.y / 2.));
//         let filter = QueryFilter::default()
//             .groups(InteractionGroups::all().with_filter(city_gen::RAPIER_GROUP_WAY_CENTERLINE | city_gen::RAPIER_GROUP_BUILDING));
//         let placement = network.0.calculate_placement(shape_pos, &shape, filter, None);
// // dbg!(&placement);

//         if let Some(placement) = placement {
//             network.0.buildings.push(BuildingInfo {
//                 position: Vec2::new(placement.x, placement.y),
//                 orientation: 0.0.into(),
//                 size,
//                 color: bevy::color::palettes::css::RED.into(),
//             });
//             // dbg!(&network.0.buildings.len());
//         }
//         // commands.insert_resource(RoadNetworkResource::new(1));
//     }

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
