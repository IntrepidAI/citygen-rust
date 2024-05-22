#![allow(clippy::too_many_arguments)]
use std::net;
use std::time::{Duration, SystemTime};

use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, LogDiagnosticsPlugin};
use bevy::gltf::GltfExtras;
use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy::render::render_asset::RenderAssetUsages;
use bevy::render::render_resource::{Extent3d, TextureDimension, TextureFormat};
use bevy::window::PrimaryWindow;
use bevy_inspector_egui::bevy_egui::EguiContexts;
use bevy_mod_raycast::prelude::*;
use noise::core::open_simplex::open_simplex_2d;
use noise::permutationtable::PermutationTable;

mod asset_reader;
mod city_gen;
mod orbit_camera;

#[derive(Resource)]
struct MapFile(String);

#[derive(Resource)]
struct RoadNetwork(crate::city_gen::RoadNetwork);

impl RoadNetwork {
    fn new(segment_count_limit: usize) -> Self {
        Self(city_gen::generate_segments(segment_count_limit))
    }
}

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
        .add_systems(Update, show_gizmos)
        .add_systems(Update, ui)
        .insert_resource(ClearColor(Color::rgb(0., 0., 0.)))
        .insert_resource(AmbientLight {
            color: Color::WHITE,
            brightness: 300.,
        })
        .insert_resource(Msaa::default())
        .insert_resource(MapFile(file))
        .insert_resource(RoadNetwork::new(1))
        .init_resource::<Highlighted>()
        .run();
}

fn ui(
    mut commands: Commands,
    mut contexts: EguiContexts,
    mut limit: Local<usize>,
    mut limit_txt: Local<String>,
) {
    let ctx = contexts.ctx_mut();

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
                commands.insert_resource(RoadNetwork::new(*limit));
            }
        });
}

fn show_gizmos(
    window_query: Query<&Window, With<PrimaryWindow>>,
    camera_query: Query<(&Camera, &GlobalTransform)>,
    mut gizmos: Gizmos,
    network: Res<RoadNetwork>,
    mut contexts: EguiContexts,
) {
    crate::city_gen::draw_segments(&mut gizmos, &network.0);

    let intersect = (|| {
        let window = window_query.get_single().ok()?;
        let (camera, gxform) = camera_query.get_single().ok()?;
        let cursor_position = window.cursor_position()?;
        let cursor_ray = camera.viewport_to_world(gxform, cursor_position)?;
        cursor_ray.intersect_plane(Vec3::ZERO, Plane3d::new(Vec3::Z))
            .map(|pos| cursor_ray.origin + cursor_ray.direction * pos)
    })();

    // if let Some(intersect) = intersect {
    //     let p1 = Vec2::new(10000., 10000.);
    //     let p2 = intersect.xy();
    //     if let Some((p2, _)) = city_gen::local_constraints_apply(p1, p2, &network.0.graph, &network.0.tree) {
    //         gizmos.line_2d(Vec2::new(10000., 10000.), p2, Color::RED);
    //     }
    // }

    let ctx = contexts.ctx_mut();

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
    let camera_transform = Transform::from_xyz(0., 540.*30., 720.*30.).looking_at(Vec3::new(0., 0., 267.), Vec3::Y);
    let camera_rotation = camera_transform.rotation.to_euler(EulerRot::YXZ);

    commands.spawn((
        Camera3dBundle::default(),
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

fn spawn_scene(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    map_file: Res<MapFile>,
    mut images: ResMut<Assets<Image>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // let random_from_time = SystemTime::now()
    //     .duration_since(SystemTime::UNIX_EPOCH)
    //     .unwrap()
    //     .as_millis() as u32;
    // let hasher = PermutationTable::new(1);

    // let mut buffer = vec![];
    // let size = 1024;

    // let mut min = f64::MAX;
    // let mut max = f64::MIN;

    // for i in 0..size {
    //     for j in 0..size {
    //         let x = i as f64 / size as f64 * 2. - 1.;
    //         let y = j as f64 / size as f64 * 2. - 1.;

    //         let color = open_simplex_2d(noise::Vector2::new(x * 10., y * 10.), &hasher);

    //         min = min.min(color);
    //         max = max.max(color);

    //         let color = ((color / 1.2 + 0.5) * 256.) as u8;
    //         // let color = 128;

    //         buffer.push(color);
    //         buffer.push(color);
    //         buffer.push(color);
    //         buffer.push(color);
    //     }
    // }
    // dbg!(min, max);

    // let image = images.add(Image::new(
    //     Extent3d {
    //         width: size,
    //         height: size,
    //         depth_or_array_layers: 1,
    //     },
    //     TextureDimension::D2,
    //     buffer,
    //     TextureFormat::Rgba8Unorm,
    //     RenderAssetUsages::all(),
    // ));

    // commands.spawn(PbrBundle {
    //     mesh: meshes.add(Plane3d::new(Vec3::Z).mesh().size(102400.0, 102400.0)),
    //     material: materials.add(image),
    //     ..default()
    // });

    // commands.spawn(SceneBundle {
    //     scene: asset_server.load(format!("{}#Scene0", map_file.0)),
    //     ..default()
    // });
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
