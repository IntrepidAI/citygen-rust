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

use bevy::{color::{palettes::css, Color}, gizmos::gizmos::Gizmos, math::{Rot2, Vec2, Vec2Swizzles}, utils::HashSet};
use nalgebra::{Isometry2, Point2, Translation2, Vector2};
use nanorand::Rng;
use noise::core::open_simplex::open_simplex_2d;
use petgraph::{graph::{EdgeIndex, NodeIndex}, visit::EdgeRef, Undirected};
use rapier2d::{parry::query::ShapeCastOptions, prelude::*};
// use rstar::primitives::GeomWithData;

// const SEGMENT_COUNT_LIMIT: usize = 20;
const BRANCH_ANGLE_DEVIATION: f32 = 3.0f32 * std::f32::consts::PI / 180.0f32;
const STRAIGHT_ANGLE_DEVIATION: f32 = 15.0f32 * std::f32::consts::PI / 180.0f32;
const MINIMUM_INTERSECTION_DEVIATION: f32 = 40.0f32 * std::f32::consts::PI / 180.0f32;
const NORMAL_SEGMENT_LENGTH: f32 = 100.; // 300
const HIGHWAY_SEGMENT_LENGTH: f32 = 150.; // 400
const NORMAL_BRANCH_PROBABILITY: f32 = 0.4;
const HIGHWAY_BRANCH_PROBABILITY: f32 = 0.05;
const NORMAL_BRANCH_POPULATION_THRESHOLD: f32 = 0.;
const HIGHWAY_BRANCH_POPULATION_THRESHOLD: f32 = 0.;
const NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY: u32 = 5;
const MAX_SNAP_DISTANCE: f32 = 30.; // 50

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
    // pub way: EdgeIndex,
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

pub struct RoadNetwork {
    pub graph: petgraph::Graph<NodeInfo, WayType, Undirected>,
    pub buildings: Vec<BuildingInfo>,
    // pub tree: rstar::RTree<GeomWithData<[f32; 2], NodeIndex>>,
    // rapier2d objects
    query_pipeline: QueryPipeline,
    rigid_bodies: RigidBodySet,
    colliders: ColliderSet,
    modified_colliders: Vec<ColliderHandle>,
}

impl Default for RoadNetwork {
    fn default() -> Self {
        RoadNetwork {
            graph: petgraph::Graph::new_undirected(),
            buildings: Vec::new(),
            // tree: rstar::RTree::new(),
            query_pipeline: QueryPipeline::new(),
            rigid_bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            modified_colliders: Vec::new(),
        }
    }
}

impl RoadNetwork {
    pub fn add_node(&mut self, position: Vec2) -> NodeIndex {
        let node = self.graph.add_node(NodeInfo { position, can_snap: true });
        // self.tree.insert(GeomWithData::new([position.x, position.y], node));
        let collider = self.colliders.insert(
            ColliderBuilder::ball(MAX_SNAP_DISTANCE)
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

    // Cast a ray from a source node to target position,
    //  - if the ray hits intersection collider (ball with radius=snap-distance), retarget to the intersection point
    //  - if the ray hits network way (line segment), retarget to the intersection point
    fn find_snapping(&mut self, source: NodeIndex, target: Vec2) -> Option<(Vec2, Option<NodeIndex>)> {
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
                    if new_dot < MINIMUM_INTERSECTION_DEVIATION.sin() {
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

                if new_dot < MINIMUM_INTERSECTION_DEVIATION.sin() {
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

pub fn generate_segments(segment_count_limit: usize) -> RoadNetwork {
    let mut rand = nanorand::WyRand::new_seed(1);
    let population = noise::permutationtable::PermutationTable::new(1);//rand.generate());
    let mut network = RoadNetwork::default();

    let root_node = network.add_node(Vec2::new(0.0, 0.0));
    let mut queue = BinaryHeap::new();

    queue.push(CheckTask {
        source: root_node,
        target: Vec2::new(HIGHWAY_SEGMENT_LENGTH, 0.),
        way_type: WayType::Highway,
        priority: 0,
    });
    queue.push(CheckTask {
        source: root_node,
        target: Vec2::new(-HIGHWAY_SEGMENT_LENGTH, 0.),
        way_type: WayType::Highway,
        priority: 0,
    });

    while let Some(mut task) = queue.pop() {
        if network.graph.edge_count() > segment_count_limit {
            break;
        }

        let target_index;
        let old_target = task.target;

        if let Some((new_target, new_index)) = local_constraints_apply(
            task.source,
            network.graph[task.source].position,
            task.target,
            &mut network,
        ) {
            task.target = new_target;
            target_index = new_index;
        } else {
            continue;
        }

        let target_index = if let Some(target_index) = target_index {
            if network.graph.find_edge(task.source, target_index).is_some() {
                continue;
            }

            target_index
        } else {
            network.add_node(task.target)
            // network.graph.add_node(NodeInfo { position: task.target, can_snap: true })
        };

        // network.tree.insert(GeomWithData::new([task.target.x, task.target.y], target_index));
        //network.graph.add_edge(task.source, target_index, task.way_type);
        network.add_way(task.source, target_index, task.way_type);

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

        for mut new_task in global_goals_generate(&mut rand, &network.graph, &population, task.source, target_index) {
            new_task.priority += 1 + task.priority;
            queue.push(new_task);
        }
    }

    let mut rand = nanorand::WyRand::new_seed(1);
    let edge_indices = network.graph.edge_indices().collect::<Vec<_>>();

    for edge_index in edge_indices {
        for _ in 0..100 {
            let (node1, node2) = network.graph.edge_endpoints(edge_index).unwrap();
            let mut node1 = &network.graph[node1];
            let mut node2 = &network.graph[node2];

            if rand.generate::<bool>() {
                (node1, node2) = (node2, node1);
            }

            let size = match rand.generate::<f32>() {
                0.0..0.2 => Vec2::new(35.438, 12.4346),
                0.2..0.4 => Vec2::new(37.788, 9.65812),
                0.4..0.7 => Vec2::new(23.2295, 16.0061),
                0.7..1.0 => Vec2::new(23.2295, 16.0061),
                _ => unreachable!(),
            };
            let size = size.yx(); // TODO: wrong orientation

            let t = rand.generate::<f32>();
            let position = node1.position + (node2.position - node1.position) * t;
            let offset = (node2.position - node1.position).normalize_or_zero().perp() * (size.y / 2. + 5.);
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
            network.update_pipeline();

            let filter = QueryFilter::default()
                .groups(InteractionGroups::new(Group::all(), RAPIER_GROUP_WAY_CENTERLINE | RAPIER_GROUP_BUILDING));
            let placement = network.calculate_placement(shape_pos, &shape, filter, Some(place_along_line));
            let mut placed = false;

            if let Some(placement) = placement {
                let position = Vec2::new(placement.x, placement.y);
                // inverse-lerp to calculate new t, make sure it's still inside segment
                let new_t = (position - node1_position).dot(node2_position - node1_position) / (node2_position - node1_position).length_squared();

                if (0.0..=1.0).contains(&new_t) {
                    network.add_building(BuildingInfo {
                        position,
                        orientation: angle.into(),
                        size,
                        color: css::DARK_MAGENTA.into(),
                    });
                    placed = true;
                }
            }

            if !placed {
                break;
            }
        }
    }

    network
}

pub fn draw_segments(
    gizmos: &mut Gizmos,
    network: &RoadNetwork,
) {
    for node in network.graph.node_indices() {
        let position = network.graph[node].position;
        let color = if network.graph[node].can_snap { css::GREEN } else { css::RED };
        // gizmos.circle_2d(position, 10., color);
        // gizmos.circle_2d(position, 9., color);
        gizmos.circle_2d(position, 8., color);
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
        gizmos.rect_2d(building.position, building.orientation, building.size, building.color);
        gizmos.line_2d(building.position, building.position + (Vec2::X * 20.).rotate((building.orientation.inverse()).sin_cos().into()), css::DARK_BLUE);
        // gizmos.circle_2d(building.position, 20., css::GOLD);
        // gizmos.circle_2d(building.position, 19., css::GOLD);
        // gizmos.circle_2d(building.position, 18., css::GOLD);
        // gizmos.circle_2d(building.position, 17., css::GOLD);
    }
}

pub fn local_constraints_apply(
    source_index: NodeIndex,
    source: Vec2,
    target: Vec2,
    network: &mut RoadNetwork,
) -> Option<(Vec2, Option<NodeIndex>)> {
    // let p1 = source;
    // let p2 = target;
    // dbg!("-----------------------");
    // dbg!(("start with", p1, p2));

    // if p1.distance_squared(p2) <= MAX_SNAP_DISTANCE * MAX_SNAP_DISTANCE {
    //     return None;
    // }

    // let p2_extended = p2 + (p2 - p1).normalize_or_zero() * MAX_SNAP_DISTANCE;
    // let pmin = p1.min(p2_extended) - HIGHWAY_SEGMENT_LENGTH * Vec2::ONE;
    // let pmax = p1.max(p2_extended) + HIGHWAY_SEGMENT_LENGTH * Vec2::ONE;

    // let envelope = rstar::AABB::from_corners(pmin.into(), pmax.into());
    // let mut new_target = None;
    // let mut new_target_dist = f32::MAX;

    // find existing segments intersecting with proposed segment,
    // if one exist, shorten the current segment up to intersection point
    // for point in network.tree.locate_in_envelope(&envelope) {
    //     let node1_position = network.graph[point.data].position;
    //     if node1_position == p1 { continue; }

    //     for other in network.graph.neighbors(point.data) {
    //         let node2_position = network.graph[other].position;
    //         if node2_position == p1 { continue; }

    //         let is = segment_intersection(p1, p2_extended, node1_position, node2_position);

    //         if let Some(is) = is {
    //             let dist = p1.distance_squared(is);
    //             if new_target.is_none() || dist < new_target_dist {
    //                 // dbg!("found better", is, dist);
    //                 new_target = Some((is, (node1_position, node2_position)));
    //                 new_target_dist = dist;
    //             }
    //         }
    //     }
    // }

    // let p2 = if let Some((is, (node1_position, node2_position))) = new_target {
    //     let angle = (is - p1).angle_between(node2_position - node1_position).abs();

    //     // #[allow(clippy::manual_range_contains)]
    //     // if angle < MINIMUM_INTERSECTION_DEVIATION || angle > std::f32::consts::PI - MINIMUM_INTERSECTION_DEVIATION {
    //     //     return None;
    //     // }

    //     is
    // } else {
    //     p2
    // };

    // let pmin = p1.min(p2) - MAX_SNAP_DISTANCE * Vec2::ONE;
    // let pmax = p1.max(p2) + MAX_SNAP_DISTANCE * Vec2::ONE;
    // let envelope = rstar::AABB::from_corners(pmin.into(), pmax.into());

    // let mut new_target = None;
    // let mut new_target_dist = f32::MAX;

    // find the closest node to the end of the segment,
    // retarget if it is within snapping distance
    // for point in network.tree.locate_in_envelope(&envelope) {
    //     if !network.graph[point.data].can_snap { continue }
    //     if *point.geom() == [p1.x, p1.y] { continue }

    //     let node1_position = network.graph[point.data].position;

    //     let dist = segment_point_distance_squared(p1, p2, node1_position);
    //     if dist <= MAX_SNAP_DISTANCE * MAX_SNAP_DISTANCE {
    //         let dist = p1.distance_squared(node1_position);
    //         if dist < new_target_dist {
    //             // dbg!(("found snap", node1_position, dist));
    //             new_target = Some((point.data, node1_position));
    //             new_target_dist = dist;
    //         }
    //     }
    // }
    network.find_snapping(source_index, target)
    // ;

    // if p2.distance_squared(p1) > MAX_SNAP_DISTANCE * MAX_SNAP_DISTANCE {
    //     Some((p2, node_index))
    // } else {
    //     None
    // }
}

fn global_goals_generate(
    rand: &mut nanorand::WyRand,
    graph: &petgraph::Graph<NodeInfo, WayType, Undirected>,
    population: &noise::permutationtable::PermutationTable,
    prev_source_idx: petgraph::graph::NodeIndex,
    prev_target_idx: petgraph::graph::NodeIndex,
) -> Vec<CheckTask> {
    let mut tasks = Vec::new();

    let prev_source = graph[prev_source_idx].position;
    let prev_target = graph[prev_target_idx].position;
// dbg!((prev_source, prev_target, prev_source_idx, prev_target_idx));

    let prev_type = graph.edges_connecting(prev_source_idx, prev_target_idx).next().unwrap().weight();

    let continue_straight = prev_target + (prev_target - prev_source).normalize_or_zero() * if *prev_type == WayType::Highway {
        HIGHWAY_SEGMENT_LENGTH
    } else {
        NORMAL_SEGMENT_LENGTH
    };
    let straight_pop = sample_population(population, prev_target);

    if *prev_type == WayType::Highway {
        let random_straight = prev_target + (prev_target - prev_source).normalize_or_zero().rotate(
            Vec2::from_angle(rand.generate::<f32>() * STRAIGHT_ANGLE_DEVIATION * 2. - STRAIGHT_ANGLE_DEVIATION)
        ) * HIGHWAY_SEGMENT_LENGTH;
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

        if road_pop > HIGHWAY_BRANCH_POPULATION_THRESHOLD {
            for direction in [-1, 1] {
                if rand.generate::<f32>() >= HIGHWAY_BRANCH_PROBABILITY { continue; }

                let random_angle = rand.generate::<f32>() * BRANCH_ANGLE_DEVIATION * 2. - BRANCH_ANGLE_DEVIATION;
                let branch = prev_target + (prev_target - prev_source).normalize_or_zero().rotate(
                    Vec2::from_angle(direction as f32 * std::f32::consts::FRAC_PI_2 + random_angle)
                ) * HIGHWAY_SEGMENT_LENGTH;

                tasks.push(CheckTask {
                    source: prev_target_idx,
                    target: branch,
                    way_type: WayType::Highway,
                    priority: 0,
                });
            }
        }
    } else if straight_pop > NORMAL_BRANCH_POPULATION_THRESHOLD {
        tasks.push(CheckTask {
            source: prev_target_idx,
            target: continue_straight,
            way_type: *prev_type,
            priority: 0,
        });
    }

    if straight_pop > NORMAL_BRANCH_POPULATION_THRESHOLD {
        for direction in [-1, 1] {
            if rand.generate::<f32>() >= NORMAL_BRANCH_PROBABILITY { continue; }

            let random_angle = rand.generate::<f32>() * BRANCH_ANGLE_DEVIATION * 2. - BRANCH_ANGLE_DEVIATION;
            let branch = prev_target + (prev_target - prev_source).normalize_or_zero().rotate(
                Vec2::from_angle(direction as f32 * std::f32::consts::FRAC_PI_2 + random_angle)
            ) * NORMAL_SEGMENT_LENGTH;

            tasks.push(CheckTask {
                source: prev_target_idx,
                target: branch,
                way_type: WayType::Normal,
                priority: if *prev_type == WayType::Highway {
                    NORMAL_BRANCH_TIME_DELAY_FROM_HIGHWAY
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
