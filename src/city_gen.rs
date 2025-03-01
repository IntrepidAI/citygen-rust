// use bevy::{gizmos::gizmos::Gizmos, math::Vec2, render::color::Color};
// use slotmap::{new_key_type, SlotMap};

// new_key_type! { pub struct NodeId; }
// new_key_type! { pub struct WayId; }

// struct Node {
//     position: Vec2,
//     ways: Vec<WayId>,
// }

// struct Way {
//     nodes: [NodeId; 2],
//     way_type: WayType,
// }

// pub enum WayType {
//     Highway,
//     Normal,
// }

// struct RoadNetwork {
//     nodes: SlotMap<NodeId, Node>,
//     ways: SlotMap<WayId, Way>,
// }

// impl RoadNetwork {
//     pub fn generate() -> Self {
//         let mut nodes = SloatMap::new();
//         let mut ways = SlotMap::new();

//         let node1 = nodes.insert(Node { position: Vec2::new(0.0, 0.0), ways: Vec::new() });

//         let root_way =
//     }

//     pub fn add_node(&mut self, position: Vec2) -> NodeId {
//         self.nodes.insert(Node { position, ways: Vec::new() })
//     }

//     pub fn add_way(&mut self, start: NodeId, end: NodeId, way_type: WayType) -> WayId {
//         let way_id = self.ways.insert(Way { nodes: [start, end], way_type });

//         let start_node = self.nodes.get_mut(start).unwrap();
//         let end_node = self.nodes.get_mut(end).unwrap();

//         start_node.ways.push(way_id);
//         end_node.ways.push(way_id);
//     }

//     pub fn draw(&self, gizmos: &mut Gizmos) {
//         for way in self.ways.values() {
//             let [start, end] = way.nodes;

//             let start = self.nodes[start].position;
//             let end = self.nodes[end].position;

//             if way.is_highway {
//                 gizmos.line_2d(start, end, Color::YELLOW);
//             } else {
//                 gizmos.line_2d(start, end, Color::WHITE);
//             }
//         }
//     }
// }

use std::collections::BinaryHeap;
use std::time::{Duration, Instant};

use bevy::color::palettes::css;
use bevy::color::Color;
use bevy::gizmos::gizmos::Gizmos;
use bevy::math::{Isometry2d, Rot2, Vec2, Vec2Swizzles};
use bevy::utils::HashSet;
use nalgebra::{Isometry2, Point2, Translation2, Vector2};
use nanorand::Rng;
use noise::core::open_simplex::open_simplex_2d;
use petgraph::graph::{EdgeIndex, NodeIndex};
use petgraph::visit::EdgeRef;
use petgraph::Undirected;
use rapier2d::parry::query::ShapeCastOptions;
use rapier2d::prelude::*;
// use rstar::primitives::GeomWithData;

// const SEGMENT_COUNT_LIMIT: usize = 20;
// const BRANCH_ANGLE_DEVIATION: f32 = 3.0f32 * std::f32::consts::PI / 180.0f32;
// const STRAIGHT_ANGLE_DEVIATION: f32 = 15.0f32 * std::f32::consts::PI / 180.0f32;
// const MINIMUM_INTERSECTION_DEVIATION: f32 = 40.0f32 * std::f32::consts::PI / 180.0f32;
// const NORMAL_SEGMENT_LENGTH: f32 = 100.; // 300
// const HIGHWAY_SEGMENT_LENGTH: f32 = 150.; // 400
// const NORMAL_BRANCH_PROBABILITY: f32 = 0.4;
// const HIGHWAY_BRANCH_PROBABILITY: f32 = 0.05;
// const NORMAL_BRANCH_POPULATION_THRESHOLD: f32 = 0.;
// const HIGHWAY_BRANCH_POPULATION_THRESHOLD: f32 = 0.;
// const NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY: u32 = 5;
// const MAX_SNAP_DISTANCE: f32 = 30.; // 50

#[derive(Debug, Clone)]
pub struct GeneratorConfig {
    pub seed: u32,
    pub segment_count_limit: usize,
    pub branch_angle_deviation: f32,
    pub straight_angle_deviation: f32,
    pub minimum_intersection_deviation: f32,
    pub normal_segment_length: f32,
    pub highway_segment_length: f32,
    pub normal_branch_probability: f32,
    pub highway_branch_probability: f32,
    pub normal_branch_population_threshold: f32,
    pub highway_branch_population_threshold: f32,
    pub normal_branch_time_delay_from_highway: u32,
    pub max_snap_distance: f32,
    pub checkpoint_count: usize,
    pub generate_buildings: bool,
    pub generate_trees: bool,
}

impl Default for GeneratorConfig {
    fn default() -> Self {
        GeneratorConfig {
            seed: 2,
            segment_count_limit: 300,
            branch_angle_deviation: 3.0f32.to_radians(),
            straight_angle_deviation: 15.0f32.to_radians(),
            minimum_intersection_deviation: 40.0f32.to_radians(),
            normal_segment_length: 100., // 300
            highway_segment_length: 150., // 400
            normal_branch_probability: 0.4,
            highway_branch_probability: 0.05,
            normal_branch_population_threshold: 0.,
            highway_branch_population_threshold: 0.,
            normal_branch_time_delay_from_highway: 5,
            max_snap_distance: 30., // 50
            checkpoint_count: 10,
            generate_buildings: false,
            generate_trees: false,
        }
    }
}

pub const RAPIER_GROUP_INTERSECTION_SNAP: Group = Group::GROUP_1;
pub const RAPIER_GROUP_WAY_CENTERLINE: Group = Group::GROUP_2;
pub const RAPIER_GROUP_BUILDING: Group = Group::GROUP_3;

pub struct NodeInfo {
    pub position: Vec2,
    // pub snapping_collider: ColliderHandle,
    pub can_snap: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WayType {
    Highway,
    Normal,
}

pub struct BuildingInfo {
    pub position: Vec2,
    pub orientation: Rot2,
    pub size: Vec2,
    pub color: Color,
    pub mesh: String,
    pub offset_from_way: f32,
}

pub struct ObjectInfo {
    pub position: Vec2,
    pub orientation: Rot2,
    pub size: f32,
    pub mesh: String,
}

enum ColliderUserData {
    NodeIndex(NodeIndex),
    EdgeIndex(EdgeIndex),
}

impl From<u128> for ColliderUserData {
    fn from(data: u128) -> Self {
        let (high, low) = ((data >> 64) as u64, data as u64);
        match high {
            1 => ColliderUserData::NodeIndex(NodeIndex::new(low as usize)),
            2 => ColliderUserData::EdgeIndex(EdgeIndex::new(low as usize)),
            _ => panic!("Invalid collider user data"),
        }
    }
}

impl From<ColliderUserData> for u128 {
    fn from(data: ColliderUserData) -> Self {
        match data {
            ColliderUserData::NodeIndex(node) => (1 << 64) | node.index() as u128,
            ColliderUserData::EdgeIndex(edge) => (2 << 64) | edge.index() as u128,
        }
    }
}

#[derive(Default)]
enum GeneratorState {
    NetworkInit,
    Network {
        rand: nanorand::WyRand,
        population: Box<noise::permutationtable::PermutationTable>,
        queue: BinaryHeap<CheckTask>,
    },
    BuildingsInit,
    Buildings {
        rand: nanorand::WyRand,
        edge_indices: Vec<EdgeIndex>,
    },
    ObjectsInit,
    Objects {
        rand: nanorand::WyRand,
        edge_indices: Vec<EdgeIndex>,
    },
    #[default]
    Done,
}

pub struct RoadNetwork {
    config: GeneratorConfig,
    pub graph: petgraph::Graph<NodeInfo, WayType, Undirected>,
    pub buildings: Vec<BuildingInfo>,
    pub objects: Vec<ObjectInfo>,
    pub checkpoints: Vec<Vec2>,
    // pub tree: rstar::RTree<GeomWithData<[f32; 2], NodeIndex>>,
    // rapier2d objects
    query_pipeline: QueryPipeline,
    rigid_bodies: RigidBodySet,
    colliders: ColliderSet,
    modified_colliders: Vec<ColliderHandle>,
    generator_state: GeneratorState,
}

impl Default for RoadNetwork {
    fn default() -> Self {
        RoadNetwork {
            config: GeneratorConfig::default(),
            graph: petgraph::Graph::new_undirected(),
            buildings: Vec::new(),
            objects: Vec::new(),
            checkpoints: Vec::new(),
            // tree: rstar::RTree::new(),
            query_pipeline: QueryPipeline::new(),
            rigid_bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            modified_colliders: Vec::new(),
            generator_state: GeneratorState::NetworkInit,
        }
    }
}

impl RoadNetwork {
    pub fn new(config: GeneratorConfig) -> Self {
        RoadNetwork {
            config,
            ..Default::default()
        }
    }

    pub fn is_generating(&self) -> bool {
        #[allow(clippy::match_like_matches_macro)]
        match self.generator_state {
            GeneratorState::Done => false,
            _ => true,
        }
    }

    pub fn stop_generating(&mut self) {
        self.generator_state = GeneratorState::Done;
    }

    pub fn generate(&mut self, timeout: Duration) {
        let now = Instant::now();
        let max_time = now + timeout;

        while Instant::now() < max_time {
            match &self.generator_state {
                GeneratorState::NetworkInit => {
                    self.generate_network_init();
                }
                GeneratorState::Network { .. } => {
                    self.generate_network(max_time);
                }
                GeneratorState::BuildingsInit => {
                    self.generate_buildings_init();
                }
                GeneratorState::Buildings { .. } => {
                    self.generate_buildings(max_time);
                }
                GeneratorState::ObjectsInit => {
                    self.generate_objects_init();
                }
                GeneratorState::Objects { .. } => {
                    self.generate_objects(max_time);
                }
                GeneratorState::Done => {
                    break;
                }
            }
        }
    }

    fn generate_network_init(&mut self) {
        let root_node = self.add_node(Vec2::new(0.0, 0.0));
        let mut queue = BinaryHeap::new();

        queue.push(CheckTask {
            source: root_node,
            target: Vec2::new(self.config.highway_segment_length, 0.),
            way_type: WayType::Highway,
            priority: 0,
        });
        queue.push(CheckTask {
            source: root_node,
            target: Vec2::new(-self.config.highway_segment_length, 0.),
            way_type: WayType::Highway,
            priority: 0,
        });

        self.generator_state = GeneratorState::Network {
            rand: nanorand::WyRand::new_seed(self.config.seed as u64),
            population: Box::new(noise::permutationtable::PermutationTable::new(self.config.seed)),
            queue,
        };
    }

    fn generate_network(&mut self, until: Instant) {
        let GeneratorState::Network { mut rand, population, mut queue } = std::mem::take(&mut self.generator_state) else {
            unreachable!();
        };

        loop {
            if self.graph.edge_count() & 0x7 == 0 && Instant::now() >= until {
                self.generator_state = GeneratorState::Network { rand, population, queue };
                return;
            }

            let Some(mut task) = queue.pop() else {
                break;
            };

            if self.graph.edge_count() > self.config.segment_count_limit {
                break;
            }

            let target_index;
            let old_target = task.target;

            if let Some((new_target, new_index)) = self.find_snapping(task.source, task.target) {
                task.target = new_target;
                target_index = new_index;
            } else {
                continue;
            }

            let target_index = if let Some(target_index) = target_index {
                if self.graph.find_edge(task.source, target_index).is_some() {
                    continue;
                }

                target_index
            } else {
                self.add_node(task.target)
            };

            self.add_way(task.source, target_index, task.way_type);

            // {
            //     graph[task.source].can_snap = true;

            //     let mut iter = graph.edges(task.source).fuse();
            //     let e1 = iter.next();
            //     let e2 = iter.next();
            //     let e3 = iter.next();

            //     if let (Some(e1), Some(e2), None) = (e1, e2, e3) {
            //         if e1.weight() == e2.weight() {
            //             let p0 = graph[e1.source()].position;
            //             let p1 = graph[e1.target()].position;
            //             let p2 = graph[e2.source()].position;
            //             let p3 = graph[e2.target()].position;

            //             if ((p1-p0).normalize_or_zero()).dot((p3-p2).normalize_or_zero()) < 0.1 {
            //                 graph[task.source].can_snap = false;
            //             }
            //         }
            //     }
            // }

            if old_target != task.target {
                continue;
            }

            for mut new_task in global_goals_generate(&self.config, &mut rand, &self.graph, &population, task.source, target_index) {
                new_task.priority += 1 + task.priority;
                queue.push(new_task);
            }
        }

        let mut node_indices = self.graph.node_indices().collect::<Vec<_>>();
        for _ in 0..self.config.checkpoint_count {
            if node_indices.is_empty() { break; }
            let node_index = node_indices.remove(rand.generate_range(0..node_indices.len()));
            self.checkpoints.push(self.graph[node_index].position);
        }

        self.generator_state = GeneratorState::BuildingsInit;
    }

    fn generate_buildings_init(&mut self) {
        if !self.config.generate_buildings {
            self.generator_state = GeneratorState::ObjectsInit;
            return;
        }

        let mut edge_indices = self.graph.edge_indices().collect::<Vec<_>>();
        edge_indices.reverse();

        self.generator_state = GeneratorState::Buildings {
            rand: nanorand::WyRand::new_seed(self.config.seed as u64),
            edge_indices,
        };
    }

    fn generate_objects_init(&mut self) {
        if !self.config.generate_trees {
            self.generator_state = GeneratorState::Done;
            return;
        }

        let mut edge_indices = self.graph.edge_indices().collect::<Vec<_>>();
        edge_indices.reverse();

        self.generator_state = GeneratorState::Objects {
            rand: nanorand::WyRand::new_seed(self.config.seed as u64),
            edge_indices,
        };
    }

    fn generate_buildings(&mut self, until: Instant) {
        let GeneratorState::Buildings { mut rand, mut edge_indices } = std::mem::take(&mut self.generator_state) else {
            unreachable!();
        };

        loop {
            if Instant::now() >= until {
                self.generator_state = GeneratorState::Buildings { rand, edge_indices };
                return;
            }

            let Some(edge_index) = edge_indices.pop() else {
                break;
            };

            for _ in 0..100 {
                let (node1, node2) = self.graph.edge_endpoints(edge_index).unwrap();
                let mut node1 = &self.graph[node1];
                let mut node2 = &self.graph[node2];

                if rand.generate::<bool>() {
                    (node1, node2) = (node2, node1);
                }

                let (size, mesh) = match rand.generate::<f32>() {
                    0.0..0.2 => (Vec2::new(35.438, 12.4346), "buildings/building1.glb".into()),
                    0.2..0.4 => (Vec2::new(37.788, 9.65812), "buildings/building2.glb".into()),
                    0.4..0.7 => (Vec2::new(23.2295, 16.0061), "buildings/building31.glb".into()),
                    0.7..1.0 => (Vec2::new(23.2295, 16.0061), "buildings/building32.glb".into()),
                    _ => unreachable!(),
                };
                let size = size.yx(); // TODO: wrong orientation

                let t = rand.generate::<f32>();
                let position = node1.position + (node2.position - node1.position) * t;
                let offset_from_way = size.y / 2. + 5.;
                let offset = (node2.position - node1.position).normalize_or_zero().perp() * offset_from_way;
                let position = position - offset;
                let angle = (node1.position - node2.position).to_angle();
                let node1_position = node1.position;
                let node2_position = node2.position;

                // debug
                // network.buildings.push(BuildingInfo {
                //     position,
                //     orientation: angle.into(),
                //     size,
                //     color: css::BLUE.into(),
                // });

                let shape_pos = Isometry2::new(Vector2::new(position.x, position.y), angle);
                let shape = rapier2d::parry::shape::Cuboid::new(Vector2::new(size.x / 2., size.y / 2.));
                let place_along_line = Vector2::new(node1.position.x - node2.position.x, node1.position.y - node2.position.y);
                self.update_pipeline();

                let filter = QueryFilter::default()
                    .groups(InteractionGroups::new(Group::all(), RAPIER_GROUP_WAY_CENTERLINE | RAPIER_GROUP_BUILDING));
                let placement = self.calculate_placement(shape_pos, &shape, filter, Some(place_along_line));
                let mut placed = false;

                if let Some(placement) = placement {
                    let position = Vec2::new(placement.x, placement.y);
                    // inverse-lerp to calculate new t, make sure it's still inside segment
                    let new_t = (position - node1_position).dot(node2_position - node1_position) / (node2_position - node1_position).length_squared();

                    if (0.0..=1.0).contains(&new_t) {
                        self.add_building(BuildingInfo {
                            position,
                            orientation: angle.into(),
                            size,
                            color: css::DARK_MAGENTA.into(),
                            mesh,
                            offset_from_way,
                        });
                        placed = true;
                    }
                }

                if !placed {
                    break;
                }
            }
        }

        self.generator_state = GeneratorState::ObjectsInit;
    }

    fn generate_objects(&mut self, until: Instant) {
        let GeneratorState::Objects { mut rand, mut edge_indices } = std::mem::take(&mut self.generator_state) else {
            unreachable!();
        };

        loop {
            if Instant::now() >= until {
                self.generator_state = GeneratorState::Objects { rand, edge_indices };
                return;
            }

            let Some(edge_index) = edge_indices.pop() else {
                break;
            };

            for _ in 0..100 {
                let (node1, node2) = self.graph.edge_endpoints(edge_index).unwrap();
                let mut node1 = &self.graph[node1];
                let mut node2 = &self.graph[node2];

                if rand.generate::<bool>() {
                    (node1, node2) = (node2, node1);
                }

                let (size, mesh) = match rand.generate::<f32>() {
                    0.0..0.5 => (2., "trees/tree_a.glb".into()),
                    0.5..1.0 => (2., "trees/tree_b.glb".into()),
                    _ => unreachable!(),
                };

                let t = rand.generate::<f32>();
                let position = node1.position + (node2.position - node1.position) * t;
                let offset = (node2.position - node1.position).normalize_or_zero().perp() * (size / 2. + 3. + 6. * rand.generate::<f32>());
                let position = position - offset;
                let angle = (node2.position - node1.position).to_angle();
                let node1_position = node1.position;
                let node2_position = node2.position;

                // debug
                // network.buildings.push(BuildingInfo {
                //     position,
                //     orientation: angle.into(),
                //     size,
                //     color: css::BLUE.into(),
                // });

                let shape_pos = Isometry2::new(Vector2::new(position.x, position.y), angle);
                let shape = rapier2d::parry::shape::Ball::new(size);
                let place_along_line = Vector2::new(node1.position.x - node2.position.x, node1.position.y - node2.position.y);
                self.update_pipeline();

                let filter = QueryFilter::default()
                    .groups(InteractionGroups::new(Group::all(), RAPIER_GROUP_WAY_CENTERLINE | RAPIER_GROUP_BUILDING));
                let placement = self.calculate_placement(shape_pos, &shape, filter, Some(place_along_line));
                let mut placed = false;

                if let Some(placement) = placement {
                    let position = Vec2::new(placement.x, placement.y);
                    // inverse-lerp to calculate new t, make sure it's still inside segment
                    let new_t = (position - node1_position).dot(node2_position - node1_position) / (node2_position - node1_position).length_squared();

                    if (0.0..=1.0).contains(&new_t) {
                        self.add_object(ObjectInfo {
                            position,
                            orientation: angle.into(),
                            size,
                            mesh,
                        });
                        placed = true;
                    }
                }

                if !placed {
                    break;
                }
            }
        }

        self.generator_state = GeneratorState::Done;
    }

    pub fn add_node(&mut self, position: Vec2) -> NodeIndex {
        let node = self.graph.add_node(NodeInfo { position, can_snap: true });
        // self.tree.insert(GeomWithData::new([position.x, position.y], node));
        let collider = self.colliders.insert(
            ColliderBuilder::ball(self.config.max_snap_distance)
                .translation(Vector2::new(position.x, position.y))
                .collision_groups(InteractionGroups::new(RAPIER_GROUP_INTERSECTION_SNAP, Group::all()))
                .user_data(ColliderUserData::NodeIndex(node).into())
                .build()
        );
        self.modified_colliders.push(collider);
        node
    }

    pub fn add_way(&mut self, start: NodeIndex, end: NodeIndex, way_type: WayType) {
        let edge_index = self.graph.add_edge(start, end, way_type);
        let start_pos = self.graph[start].position;
        let end_pos = self.graph[end].position;
        let collider = self.colliders.insert(
            ColliderBuilder::segment(Point2::new(start_pos.x, start_pos.y), Point2::new(end_pos.x, end_pos.y))
                .collision_groups(InteractionGroups::new(RAPIER_GROUP_WAY_CENTERLINE, Group::all()))
                .user_data(ColliderUserData::EdgeIndex(edge_index).into())
                .build()
        );
        self.modified_colliders.push(collider);
    }

    pub fn add_building(&mut self, building: BuildingInfo) {
        let collider = self.colliders.insert(
            ColliderBuilder::cuboid(building.size.x / 2., building.size.y / 2.)
                .translation(Vector2::new(building.position.x, building.position.y))
                .rotation(building.orientation.as_radians())
                .collision_groups(InteractionGroups::new(RAPIER_GROUP_BUILDING, Group::all()))
                .build()
        );
        self.buildings.push(building);
        self.modified_colliders.push(collider);
    }

    pub fn add_object(&mut self, object: ObjectInfo) {
        let collider = self.colliders.insert(
            ColliderBuilder::ball(object.size)
                .translation(Vector2::new(object.position.x, object.position.y))
                .rotation(object.orientation.as_radians())
                .collision_groups(InteractionGroups::new(RAPIER_GROUP_BUILDING, Group::all()))
                .build()
        );
        self.objects.push(object);
        self.modified_colliders.push(collider);
    }

    // Cast a ray from a source node to target position,
    //  - if the ray hits intersection collider (ball with radius=snap-distance), retarget to the intersection point
    //  - if the ray hits network way (line segment), retarget to the intersection point
    pub fn find_snapping(&mut self, source: NodeIndex, target: Vec2) -> Option<(Vec2, Option<NodeIndex>)> {
        self.update_pipeline();

        // let source_collider = self.graph[source].snapping_collider;
        let source_pos = self.graph[source].position;
        let ray = Ray::new(
            Point2::new(source_pos.x, source_pos.y),
            Vector2::new(target.x - source_pos.x, target.y - source_pos.y).normalize(),
        );
        let predicate = |_handle: ColliderHandle, collider: &Collider| {
            // ignore node we are coming from
            // if handle == source_collider { return false; }
            if let ColliderUserData::NodeIndex(node_index) = ColliderUserData::from(collider.user_data) {
                if node_index == source {
                    return false;
                }
            }

            // ignore existing ways coming from the same source node
            if let ColliderUserData::EdgeIndex(edge_index) = ColliderUserData::from(collider.user_data) {
                let (start, end) = self.graph.edge_endpoints(edge_index).unwrap();
                if start == source || end == source {
                    return false;
                }
            }

            true
        };
        let filter = QueryFilter::default()
            // .exclude_collider(source_collider)
            // .groups(InteractionGroups::new(
            //     RAPIER_GROUP_INTERSECTION_SNAP,
            //     Group::all(),
            // ))
            .predicate(&predicate);

        let Some((collider_handle, toi)) = self.query_pipeline.cast_ray(
            &self.rigid_bodies,
            &self.colliders,
            &ray,
            target.distance(source_pos),
            true,
            filter,
        ) else {
            return Some((target, None));
        };

        let collider = self.colliders.get(collider_handle)?;
        let minimum_intersection_deviation = self.config.minimum_intersection_deviation.sin();

        match ColliderUserData::from(collider.user_data) {
            ColliderUserData::NodeIndex(node_index) => {
                let node = self.graph.node_weight(node_index)?;
                let new_target = node.position;

                // calculate minimum angle between this edge and all other edges connected to the node
                for edge_index in self.graph.edges(node_index) {
                    let (source, target) = self.graph.edge_endpoints(edge_index.id())?;
                    let start = self.graph[source].position;
                    let end = self.graph[target].position;
                    let new_dot = ((end - start).normalize_or_zero().dot((new_target - source_pos).normalize_or_zero())).abs();
                    if new_dot < minimum_intersection_deviation {
                        return None;
                    }
                }

                Some((new_target, Some(node_index)))
            }
            ColliderUserData::EdgeIndex(edge_index) => {
                // let (source, target) = self.graph.edge_endpoints(edge_index)?;
                // let start = self.graph[source].position;
                // let end = self.graph[target].position;
                // let intersection = segment_intersection(start, end, source, target)?;
                let intersection = Point2::new(source_pos.x, source_pos.y) + ray.dir * toi;
                let new_target = Vec2::new(intersection.x, intersection.y);

                // calculate minimum angle between this edge and existing edge
                let (source, target) = self.graph.edge_endpoints(edge_index)?;
                let start = self.graph[source].position;
                let end = self.graph[target].position;
                let new_dot = ((end - start).normalize_or_zero().dot((new_target - source_pos).normalize_or_zero())).abs();

                if new_dot < minimum_intersection_deviation {
                    return None;
                }

                Some((new_target, None))
            }
        }
    }

    pub fn calculate_placement(
        &mut self,
        mut shape_pos: Isometry2<f32>,
        shape: &dyn Shape,
        filter: QueryFilter,
        along_line: Option<Vector2<f32>>,
    ) -> Option<Translation2<f32>> {
        self.update_pipeline();

        let along_line = along_line.and_then(|v| v.try_normalize(1e-10));
        let mut intersections = HashSet::new();
        self.query_pipeline.intersections_with_shape(&self.rigid_bodies, &self.colliders, &shape_pos, shape, filter, |handle| {
            intersections.insert(handle);
            true
        });
        if intersections.is_empty() {
            return Some(shape_pos.translation);
        }

        let center = shape_pos.translation.transform_point(&Point2::origin());
        let (_collider, projection) = self.query_pipeline.project_point(&self.rigid_bodies, &self.colliders, &center, false, filter)?;
        let contact = projection.point;
        let mut shape_vel: Vector2<f32> = (if projection.is_inside { center - contact } else { contact - center }).try_normalize(1e-10)?;

        if let Some(along_line) = along_line {
            if shape_vel.dot(&along_line) < 0. {
                shape_vel = -along_line;
            } else {
                shape_vel = along_line;
            }
        }

        shape_pos.translation.vector -= shape_vel * 1000.;

        let (_collider, hit) = self.query_pipeline.cast_shape(
            &self.rigid_bodies,
            &self.colliders,
            &shape_pos,
            &shape_vel,
            shape,
            ShapeCastOptions::default(),
            QueryFilter::default().predicate(&|handle, _| intersections.contains(&handle)),
        )?;
        shape_pos.translation.vector += shape_vel * hit.time_of_impact - shape_vel * 0.1;

        if self.query_pipeline.intersection_with_shape(
            &self.rigid_bodies,
            &self.colliders,
            &shape_pos,
            shape,
            filter,
        ).is_some() {
            return None;
        }

        Some(shape_pos.translation)
    }

    fn update_pipeline(&mut self) {
        let modified_colliders = std::mem::take(&mut self.modified_colliders);
        if modified_colliders.is_empty() { return; }
        self.query_pipeline.update_incremental(&self.colliders, &modified_colliders, &[], true);
    }
}

struct CheckTask {
    source: petgraph::graph::NodeIndex,
    target: Vec2,
    way_type: WayType,
    priority: u32,
}

impl PartialEq for CheckTask {
    fn eq(&self, other: &Self) -> bool {
        self.priority == other.priority
    }
}

impl Eq for CheckTask {}

impl PartialOrd for CheckTask {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for CheckTask {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        other.priority.cmp(&self.priority)
    }
}

pub fn draw_segments(
    gizmos: &mut Gizmos,
    network: &RoadNetwork,
) {
    for node in network.graph.node_indices() {
        let position = network.graph[node].position;
        let color = if network.graph[node].can_snap { css::DARK_SLATE_BLUE } else { css::RED };
        // gizmos.circle_2d(position, 10., color);
        // gizmos.circle_2d(position, 9., color);
        gizmos.circle_2d(position, 5., color);
    }

    for edge in network.graph.edge_indices() {
        let (start, end) = network.graph.edge_endpoints(edge).unwrap();
        let start = network.graph[start].position;
        let end = network.graph[end].position;

        match network.graph[edge] {
            WayType::Highway => gizmos.line_2d(start, end, css::YELLOW),
            WayType::Normal => gizmos.line_2d(start, end, css::GRAY),
        }
    }

    for building in network.buildings.iter() {
        gizmos.rect_2d(
            Isometry2d::new(building.position, building.orientation),
            building.size,
            building.color,
        );
        gizmos.line_2d(
            building.position,
            building.position - (Vec2::X * building.offset_from_way).rotate((building.orientation.inverse()).sin_cos().into()),
            css::DARK_BLUE,
        );
        // gizmos.circle_2d(building.position, 20., css::GOLD);
        // gizmos.circle_2d(building.position, 19., css::GOLD);
        // gizmos.circle_2d(building.position, 18., css::GOLD);
        // gizmos.circle_2d(building.position, 17., css::GOLD);
    }

    for object in network.objects.iter() {
        gizmos.circle_2d(object.position, object.size, css::GREEN);
    }

    for checkpoint in network.checkpoints.iter() {
        gizmos.rounded_rect_2d(
            Isometry2d::new(*checkpoint, Rot2::degrees(45.)),
            Vec2::new(7., 7.),
            css::RED,
        );
        gizmos.rounded_rect_2d(
            Isometry2d::new(*checkpoint, Rot2::degrees(45.)),
            Vec2::new(6., 6.),
            css::RED,
        );
        gizmos.rounded_rect_2d(
            Isometry2d::new(*checkpoint, Rot2::degrees(45.)),
            Vec2::new(5., 5.),
            css::RED,
        );
    }
}

fn global_goals_generate(
    config: &GeneratorConfig,
    rand: &mut nanorand::WyRand,
    graph: &petgraph::Graph<NodeInfo, WayType, Undirected>,
    population: &noise::permutationtable::PermutationTable,
    prev_source_idx: petgraph::graph::NodeIndex,
    prev_target_idx: petgraph::graph::NodeIndex,
) -> Vec<CheckTask> {
    let mut tasks = Vec::new();

    let prev_source = graph[prev_source_idx].position;
    let prev_target = graph[prev_target_idx].position;

    let prev_type = graph.edges_connecting(prev_source_idx, prev_target_idx).next().unwrap().weight();

    let continue_straight = prev_target + (prev_target - prev_source).normalize_or_zero() * if *prev_type == WayType::Highway {
        config.highway_segment_length
    } else {
        config.normal_segment_length
    };
    let straight_pop = sample_population(population, prev_target);

    if *prev_type == WayType::Highway {
        let straight_angle_deviation = config.straight_angle_deviation;
        let random_straight = prev_target + (prev_target - prev_source).normalize_or_zero().rotate(
            Vec2::from_angle(rand.generate::<f32>() * straight_angle_deviation * 2. - straight_angle_deviation)
        ) * config.highway_segment_length;
        let random_pop = sample_population(population, random_straight);
        let road_pop = if random_pop > straight_pop {
            tasks.push(CheckTask {
                source: prev_target_idx,
                target: random_straight,
                way_type: *prev_type,
                priority: 0,
            });
            random_pop
        } else {
            tasks.push(CheckTask {
                source: prev_target_idx,
                target: continue_straight,
                way_type: *prev_type,
                priority: 0,
            });
            straight_pop
        };

        if road_pop > config.highway_branch_population_threshold {
            for direction in [-1, 1] {
                if rand.generate::<f32>() >= config.highway_branch_probability { continue; }

                let branch_angle_deviation = config.branch_angle_deviation;
                let random_angle = rand.generate::<f32>() * branch_angle_deviation * 2. - branch_angle_deviation;
                let branch = prev_target + (prev_target - prev_source).normalize_or_zero().rotate(
                    Vec2::from_angle(direction as f32 * std::f32::consts::FRAC_PI_2 + random_angle)
                ) * config.highway_segment_length;

                tasks.push(CheckTask {
                    source: prev_target_idx,
                    target: branch,
                    way_type: WayType::Highway,
                    priority: 0,
                });
            }
        }
    } else if straight_pop > config.normal_branch_population_threshold {
        tasks.push(CheckTask {
            source: prev_target_idx,
            target: continue_straight,
            way_type: *prev_type,
            priority: 0,
        });
    }

    if straight_pop > config.normal_branch_population_threshold {
        for direction in [-1, 1] {
            if rand.generate::<f32>() >= config.normal_branch_probability { continue; }

            let branch_angle_deviation = config.branch_angle_deviation;
            let random_angle = rand.generate::<f32>() * branch_angle_deviation * 2. - branch_angle_deviation;
            let branch = prev_target + (prev_target - prev_source).normalize_or_zero().rotate(
                Vec2::from_angle(direction as f32 * std::f32::consts::FRAC_PI_2 + random_angle)
            ) * config.normal_segment_length;

            tasks.push(CheckTask {
                source: prev_target_idx,
                target: branch,
                way_type: WayType::Normal,
                priority: if *prev_type == WayType::Highway {
                    config.normal_branch_time_delay_from_highway
                } else {
                    0
                },
            });
        }
    }

    tasks
}

pub fn sample_population(population: &noise::permutationtable::PermutationTable, at: Vec2) -> f32 {
    let at: [f64; 2] = (at.as_dvec2() / 102400. * 15.).into();
    open_simplex_2d(at.into(), population) as f32
}
