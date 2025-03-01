#![allow(clippy::too_many_arguments)]
use std::time::Duration;

use bevy::color::palettes::css;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy::window::PrimaryWindow;
use bevy_inspector_egui::bevy_egui::EguiContexts;
use bevy_inspector_egui::egui::{Color32, DragValue, Slider, Spinner};
use city_gen::{GeneratorConfig, RAPIER_GROUP_BUILDING, RAPIER_GROUP_WAY_CENTERLINE};
use petgraph::graph::NodeIndex;
use rapier2d::na::{Isometry2, Vector2};
use rapier2d::prelude::{Group, InteractionGroups, QueryFilter};

use crate::pan_camera::{PanCamera, PanCamera2dBundle};

const GENERATOR_ITERATION_TIMEOUT: Duration = Duration::from_millis(50);

mod city_gen;
mod pan_camera;

#[derive(Resource, Default)]
pub enum DebugState {
    #[default]
    None,
    PlaceNode,
    ConnectNode(NodeIndex),
    PlaceBuilding,
}

#[derive(Resource, Default)]
struct CurrentGeneratorConfig(GeneratorConfig);

#[derive(Resource)]
struct RoadNetworkResource {
    network: crate::city_gen::RoadNetwork,
    saved: Option<String>,
}

impl RoadNetworkResource {
    fn new(config: GeneratorConfig) -> Self {
        // Self(city_gen::generate_segments(config))
        let mut network = city_gen::RoadNetwork::new(config);
        network.generate(GENERATOR_ITERATION_TIMEOUT / 4);
        Self {
            network,
            saved: None,
        }
    }

    fn to_lua(&self) -> String {
        let mut result = vec![
            r#"local map = require("map")"#.to_string(),
        ];

        for edge in self.network.graph.edge_indices() {
            let (a, b) = self.network.graph.edge_endpoints(edge).unwrap();
            let a = &self.network.graph[a];
            let b = &self.network.graph[b];

            // let edge_weight = self.0.graph.edge_weight(edge).unwrap();
            // let is_highway = match edge_weight {
            //     WayType::Highway => true,
            //     WayType::Normal => false,
            // };

            result.push(format!(
                "map.spawn_road {{ src = {{ {}, {} }}, dst = {{ {}, {} }}, highway = {} }}",
                a.position.x, a.position.y, b.position.x, b.position.y, false//is_highway
            ));
        }

        for building in &self.network.buildings {
            result.push(format!(
                "map.spawn {{ mesh = \"{}\", position = {{ x = {}, y = {} }}, rotation = {{ yaw = {} }} }}",
                building.mesh, building.position.x, building.position.y, building.orientation.as_degrees()
            ));
        }

        for object in &self.network.objects {
            result.push(format!(
                "map.spawn {{ mesh = \"{}\", position = {{ x = {}, y = {} }}, rotation = {{ yaw = {} }} }}",
                object.mesh, object.position.x, object.position.y, object.orientation.as_degrees()
            ));
        }

        for goal in &self.network.checkpoints {
            result.push(format!(
                "map.spawn_goal {{ position = {{ x = {}, y = {} }}, active = true }}",
                goal.x, goal.y
            ));
        }

        result.join("\n")
    }
}

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
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
        .add_systems(Update, show_gizmos)
        .add_systems(Update, ui)
        .add_systems(Update, |mut network: ResMut<RoadNetworkResource>| {
            if network.network.is_generating() {
                network.network.generate(GENERATOR_ITERATION_TIMEOUT);
                network.saved = None;
            }
        })
        .add_systems(Update, visual_debugging)
        .init_resource::<DebugState>()
        .insert_resource(ClearColor(Color::srgb(0., 0., 0.)))
        // .insert_resource(AmbientLight {
        //     color: Color::WHITE,
        //     brightness: 300.,
        // })
        .insert_resource(CurrentGeneratorConfig::default())
        .insert_resource(RoadNetworkResource::new(GeneratorConfig::default()))
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
    mut state: ResMut<DebugState>,
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
                if network.network.is_generating() {
                    if ui.button("stop").clicked() {
                        network.network.stop_generating();
                    }

                    ui.add(Spinner::new().color(Color32::DARK_GRAY));
                } else {
                    if ui.button("generate").clicked() || (ui_settings.auto_generate && changed) {
                        commands.insert_resource(RoadNetworkResource::new(settings.0.clone()));
                        *state = DebugState::None;
                    }

                    changed |= ui.checkbox(&mut ui_settings.auto_generate, "auto").changed();
                }
            });

            ui.separator();

            ui.text_edit_singleline(&mut ui_settings.filename);

            if ui.button("write to file").clicked() {
                let filename = ui_settings.filename.clone();
                commands.queue(|world: &mut World| {
                    let mut network = world.get_resource_mut::<RoadNetworkResource>().unwrap();
                    let mut file = std::fs::File::create(&filename).unwrap();
                    let text = network.to_lua();
                    use std::io::Write;
                    file.write_all(text.as_bytes()).unwrap();
                    network.saved = Some(filename);
                });
            }

            if let Some(file) = network.saved.as_ref() {
                ui.label(format!("saved as {file}"));
            }

            ui.separator();

            ui.horizontal(|ui| {
                let mouse_position: Option<Vec2> = (|| {
                    let window = window_query.get_single().ok()?;
                    let (camera, gxform) = camera_query.get_single().ok()?;
                    let cursor_position = window.cursor_position()?;
                    let cursor_ray = camera.viewport_to_world(gxform, cursor_position).ok()?;
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

            ui.horizontal(|ui| {
                if ui.button("place roads").clicked() {
                    *state = DebugState::PlaceNode;
                }

                if ui.button("place buildings").clicked() {
                    *state = DebugState::PlaceBuilding;
                }
            });
        });
}

fn spawn_environment(mut commands: Commands) {
    commands.spawn(PanCamera2dBundle {
        pan_camera: PanCamera {
            zoom: 8.,
            ..default()
        },
        ..default()
    });
}

fn show_gizmos(mut gizmos: Gizmos, network: Res<RoadNetworkResource>) {
    crate::city_gen::draw_segments(&mut gizmos, &network.network);
}

fn visual_debugging(
    mut gizmos: Gizmos,
    mut network: ResMut<RoadNetworkResource>,
    mut state: ResMut<DebugState>,
    window_query: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
) {
    let mouse_position: Option<Vec2> = (|| {
        let window = window_query.get_single().ok()?;
        let (camera, gxform) = camera_query.get_single().ok()?;
        let cursor_position = window.cursor_position()?;
        let cursor_ray = camera.viewport_to_world(gxform, cursor_position).ok()?;
        let position = cursor_ray.intersect_plane(Vec3::ZERO, InfinitePlane3d::new(Vec3::Z))
            .map(|pos| cursor_ray.origin + cursor_ray.direction * pos);
        position.map(|pos| Vec2::new(pos.x, pos.y))
    })();

    let Some(mouse_position) = mouse_position else { return };

    match *state {
        DebugState::None => {}
        DebugState::PlaceNode => {
            let new_node = network.network.add_node(mouse_position);
            *state = DebugState::ConnectNode(new_node);
        }
        DebugState::ConnectNode(source) => {
            if let Some(source_pos) = network.network.graph.node_weight(source).map(|node| node.position) {
                let snapping = network.network.find_snapping(source, mouse_position);
                let new_target = if let Some((target, _)) = snapping {
                    target
                } else {
                    mouse_position
                };
                gizmos.line_2d(source_pos, new_target, css::AQUAMARINE);
            }
        }
        DebugState::PlaceBuilding => {
            let filter = QueryFilter::default()
                .groups(InteractionGroups::new(Group::all(), RAPIER_GROUP_WAY_CENTERLINE | RAPIER_GROUP_BUILDING));

            if let Some(placement) = network.network.calculate_placement(
                Isometry2::translation(mouse_position.x, mouse_position.y),
                &rapier2d::parry::shape::Cuboid::new(Vector2::new(20. / 2., 20. / 2.)),
                filter,
                None,
            ) {
                gizmos.rect_2d(
                    Isometry2d::from_translation(Vec2::new(placement.x, placement.y)),
                    Vec2::new(20., 20.),
                    css::AQUAMARINE,
                );
            }
        }
    }
}
