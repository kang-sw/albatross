use std::collections::{BTreeSet, HashSet, VecDeque};

use albatross::{
    bitindex::BitIndexSet,
    bsp::{self, OptimizeParameter, TraceComponent, TraceShape, TreeElement},
    primitive::{AabbRect, VectorExt},
};
use egui::{Color32, Stroke};
use nalgebra::Vector2;
use web_time::Instant;

pub const BOID_SIZE: f32 = 0.2;
const PREDATOR_SIZE: f32 = 0.6;
const BOID_ARROW_LEN: f32 = 0.5;
const PREDATOR_ARROW_LEN: f32 = 1.0;
const BOID_COLOR: egui::Color32 = egui::Color32::WHITE;
const PREDATOR_COLOR: egui::Color32 = egui::Color32::RED;

struct BspContext {
    shape: TraceShape<[f32; 2]>,
}

impl Default for BspContext {
    fn default() -> Self {
        Self {
            shape: TraceShape::Sphere(BOID_SIZE),
        }
    }
}

#[derive(Clone)]
struct BspData {
    entity: hecs::Entity,
}

albatross::define_key!(
    struct TreeNodeIndex;
    struct ElementIndex;
);

impl bsp::Context for BspContext {
    type Vector = [f32; 2];
    type ElemKey = ElementIndex;
    type NodeKey = TreeNodeIndex;
    type LeafData = ();
    type Element = BspData;

    fn new_leaf_data(&mut self) -> Self::LeafData {}

    fn extent(&self, _elem: &TreeElement<Self>) -> TraceComponent<[f32; 2]> {
        TraceComponent {
            offset: [0.; 2],
            shape: self.shape,
        }
    }
}

#[derive(Clone, Copy)]
struct BoidKinetic {
    pos: Vector2<f32>,
    vel: Vector2<f32>,
}

struct BoidForce {
    acc: Vector2<f32>,
}

struct IsPredator(bool);

pub struct Model {
    ecs: hecs::World,
    bsp: bsp::Tree<BspContext>,

    rand: fastrand::Rng,

    pub tick_delta: f32,
    pub enable_tick: bool,

    pub area_radius: f32,
    pub view_radius: f32,

    pub near_radius: f32,

    pub cohesion_force: f32,
    pub align_force: f32,
    pub separation_force: f32,
    pub max_speed: f32,

    pub predator_avoidance: f32,

    pub tree_optimize: bsp::OptimizeParameter,

    pub stat_max_record: usize,

    stats: VecDeque<Stats>,
    hit_test: Option<([f32; 2], TraceShape<[f32; 2]>, f32)>,
}

#[derive(Default)]
pub struct Stats {
    pub tick: f64,
    pub avg_query: f64,
    pub avg_step: f64,
    pub elem_count: usize,
    pub optimize: f64,
    pub verify: f64,
}

impl Default for Model {
    fn default() -> Self {
        Self {
            ecs: Default::default(),
            bsp: Default::default(),
            rand: fastrand::Rng::with_seed(0),

            tick_delta: 1. / 60.,
            enable_tick: true,

            area_radius: 100.,
            view_radius: 3.,
            near_radius: 1.,
            cohesion_force: 2.,
            align_force: 1.,
            separation_force: 3.,
            max_speed: 10.,
            predator_avoidance: 150.,

            stat_max_record: 120,
            tree_optimize: bsp::OptimizeParameter::moderate(4).with(|x| {
                x.minimum_length = 2.;
                x.max_collapse_height = u16::MAX;
                x.square_split_axes = BitIndexSet::all();
            }),

            stats: Default::default(),
            hit_test: None,
        }
    }
}

/* --------------------------------------- Simulation --------------------------------------- */
impl Model {
    pub fn stats(&self) -> &VecDeque<Stats> {
        &self.stats
    }

    fn wr_stat(&mut self) -> &mut Stats {
        self.stats.back_mut().unwrap()
    }

    pub fn tick(&mut self) {
        let dt = if self.enable_tick {
            self.tick_delta
        } else {
            0.
        };

        if self.stats.len() >= self.stat_max_record {
            self.stats.pop_front();
        }

        self.stats.push_back(Default::default());

        let start_time = Instant::now();

        self.calc_force();
        self.step(dt as _);
        self.refresh_tree();

        self.wr_stat().tick = start_time.elapsed().as_secs_f64();
    }

    fn calc_force(&mut self) {
        let start_time;
        let mut query_count;
        let mut removed_entities = BTreeSet::new();

        {
            let mut q_elems = self
                .ecs
                .query::<(&BoidKinetic, &ElementIndex, &IsPredator)>();
            let q_elems = q_elems.view();

            let mut adjacent_kinetics = Vec::new();
            let mut predator_kinetics = Vec::new();

            start_time = Instant::now();
            query_count = 0;

            for (entity, force) in self.ecs.query::<&mut BoidForce>().into_iter() {
                query_count += 1;

                let (boid, tree_key, IsPredator(is_predator)) = q_elems.get(entity).unwrap();

                adjacent_kinetics.clear();
                predator_kinetics.clear();

                let view_radius = if *is_predator { 2. } else { 1. } * self.view_radius;
                self.bsp
                    .trace_sphere(&boid.pos.into(), view_radius, 0., |_, elem_id, elem| {
                        if elem_id == *tree_key {
                            return;
                        }

                        let (other_boid, _, IsPredator(other_is_predator)) =
                            q_elems.get(elem.entity).unwrap();

                        let distance = (boid.pos - other_boid.pos).norm();

                        if *is_predator && !*other_is_predator && distance < PREDATOR_SIZE {
                            predator_kinetics.push((elem.entity, other_boid, distance));
                        }

                        if !*is_predator && *other_is_predator {
                            predator_kinetics.push((elem.entity, other_boid, distance));
                        } else {
                            adjacent_kinetics.push((other_boid, distance));
                        }
                    });

                if adjacent_kinetics.is_empty() {
                    force.acc = Default::default();
                    continue;
                }

                // If it gets out of the ring; gravity affects
                let gravity = 0.06 * -boid.pos.normalize();

                // Calculate cohesion => Average point of adjacent boids.
                let avg = adjacent_kinetics
                    .iter()
                    .fold(Vector2::zeros(), |avg, (kin, ..)| avg + kin.pos)
                    / adjacent_kinetics.len() as f32;

                let cohesion_dir = avg - boid.pos;

                // Calculate align => Avaerage velocity of adjacent boids.
                let align_dir = adjacent_kinetics
                    .iter()
                    .fold(Vector2::zeros(), |avg, (kin, ..)| avg + kin.vel)
                    / adjacent_kinetics.len() as f32;

                // Calculate separation => Pushing force from adjacent boids
                let separation_dir = adjacent_kinetics
                    .iter()
                    .filter(|(_, distance)| {
                        *distance
                            < if *is_predator {
                                PREDATOR_SIZE
                            } else {
                                self.near_radius
                            }
                    })
                    .fold(Vector2::zeros(), |avg, (kin, distance)| {
                        avg + (boid.pos - kin.pos) * (1. / distance.max(0.0001))
                    });

                // Calculate predator avoidance
                let predator_force = if *is_predator {
                    // predator kinetics holds "Swallowed" entities.
                    removed_entities.extend(predator_kinetics.drain(..).map(|(e, ..)| e));

                    // Multiply doubled cohesion if it's predator
                    cohesion_dir + separation_dir * 5.
                } else {
                    // Avoid predators
                    let mut escape_acc = Vector2::zeros();

                    for (_, predator, _) in predator_kinetics.iter() {
                        let predator_dir = boid.pos - predator.pos;
                        escape_acc += predator_dir * self.predator_avoidance;
                    }

                    escape_acc
                };

                force.acc = gravity
                    + cohesion_dir * self.cohesion_force
                    + align_dir * self.align_force
                    + separation_dir * self.separation_force
                    + predator_force;
            }
        }

        for entity in removed_entities {
            let tree_key = *self.ecs.get::<&ElementIndex>(entity).unwrap();
            self.bsp.remove(tree_key);
            self.ecs.despawn(entity).unwrap();
        }

        self.wr_stat().avg_query = start_time.elapsed().as_secs_f64() / query_count as f64;
        self.wr_stat().elem_count = query_count;
    }

    fn step(&mut self, dt: f64) {
        let step_start = Instant::now();

        for (_entity, (kin, force, tree_key, predator)) in self
            .ecs
            .query::<(&mut BoidKinetic, &BoidForce, &ElementIndex, &IsPredator)>()
            .iter()
        {
            // Add some randomness on force
            let acc = {
                let norm = force.acc.norm();
                let dir = force.acc / norm.max(1e-6);
                dir * norm * (1. + fastrand::f32() * 0.1)
            };

            kin.vel += acc * dt as f32;

            if kin.pos.norm() > self.area_radius {
                let u_outer = kin.pos.normalize();

                // Apply friction
                kin.vel *= 0.4;

                // Contrain current position to world' boundary
                kin.pos = u_outer * self.area_radius;

                // Remove vertical outward velocity
                let vert_spd = kin.vel.dot(&u_outer).max(0.);
                kin.vel -= u_outer * vert_spd * 1.4;
                kin.vel -= u_outer * fastrand::f32() * 0.1;
            }

            let speed = kin.vel.norm();
            let max_speed = if predator.0 {
                self.max_speed * 1.5
            } else {
                self.max_speed
            };

            if speed > max_speed {
                kin.vel *= self.max_speed / speed;
            }

            kin.pos += kin.vel * dt as f32;

            self.bsp.get_mut(*tree_key).unwrap().set_pos(kin.pos.into());
        }

        self.wr_stat().avg_step = step_start.elapsed().as_secs_f64() / self.ecs.len() as f64;
    }

    fn refresh_tree(&mut self) {
        // Compare before / after optimize
        assert_eq!(
            self.ecs.len(),
            self.bsp.__debug_verify_tree_state().unwrap() as u32,
            "Invalid before optimize"
        );

        let optimize_start = Instant::now();
        self.bsp.optimize(&self.tree_optimize, |_| {});
        self.wr_stat().optimize = optimize_start.elapsed().as_secs_f64();

        let verify_start = Instant::now();
        assert_eq!(
            self.ecs.len(),
            self.bsp.__debug_verify_tree_state().unwrap() as u32,
            "Invalid after optimize"
        );
        self.wr_stat().verify = verify_start.elapsed().as_secs_f64();
    }

    pub fn spawn_boids(&mut self, count: usize, at: [f32; 2], predator: bool) {
        for _ in 0..count {
            let entity = self.ecs.spawn((
                BoidKinetic {
                    pos: {
                        let mut v = at;
                        v[0] += self.rand.f32() - 0.5;
                        v[1] += self.rand.f32() - 0.5;
                        v.into()
                    },
                    vel: [0; 2].map(|_| self.rand.f32() - 0.5).into(),
                },
                IsPredator(predator),
            ));
            let tree_key = self.bsp.insert(at, BspData { entity });
            self.ecs
                .insert(
                    entity,
                    (
                        tree_key,
                        BoidForce {
                            acc: Default::default(),
                        },
                    ),
                )
                .unwrap();
        }
    }

    pub(crate) fn collapse_all(&mut self) {
        self.bsp
            .optimize(&OptimizeParameter::collapse_all(), |_| ());
    }

    pub fn clear(&mut self) {
        self.ecs.clear();
        self.bsp.clear();
    }

    pub fn set_hit_test(&mut self, origin: [f32; 2], trace: TraceShape<[f32; 2]>, margin: f32) {
        self.hit_test = Some((origin, trace, margin));
    }

    pub fn clear_hit_test(&mut self) {
        self.hit_test = None;
    }

    pub fn extent_mut(&mut self) -> &mut TraceShape<[f32; 2]> {
        &mut self.bsp.context.shape
    }

    pub fn extent(&self) -> &TraceShape<[f32; 2]> {
        &self.bsp.context.shape
    }
}

/* ---------------------------------------- Rendering --------------------------------------- */

pub struct RenderOption {
    pub offset: [f32; 2],
    pub zoom: f32,
    pub draw_grid: bool,
}

impl Default for RenderOption {
    fn default() -> Self {
        Self {
            offset: Default::default(),
            zoom: 8.,
            draw_grid: true,
        }
    }
}

impl Model {
    pub fn draw(
        &self,
        ui: &mut egui::Ui,
        RenderOption {
            offset,
            zoom,
            draw_grid,
        }: &RenderOption,
    ) -> egui::Response {
        let ui_size = ui.available_size();
        let (resp, p) = ui.allocate_painter(ui_size, egui::Sense::click_and_drag());
        let offset = *offset;
        let zoom = (*zoom).max(0.001);

        let mut hit_test_grids = HashSet::new();
        let mut hit_test_elems = Vec::new();
        let mut hit_test_elapsed = 0.;

        let hit_test = self.hit_test.map(|(center, mut shape, margin)| {
            // Coordinates should be converted to world space
            let center = to_world(center.into(), offset, zoom);
            match &mut shape {
                TraceShape::Aabb(x) => *x = x.amp(1. / zoom),
                TraceShape::Sphere(rad) => *rad /= zoom,
                TraceShape::Capsule { dir, .. } => {
                    dir.s_len /= zoom;
                }
            }

            (center, shape, margin)
        });

        if let Some((center, shape, margin)) = hit_test {
            hit_test_grids.reserve(10);
            hit_test_elems.reserve(128);

            let start = Instant::now();
            self.bsp.trace(&center, &shape, margin, |node, elem, _| {
                hit_test_grids.insert(node);
                hit_test_elems.push(elem);
            });
            hit_test_elapsed = start.elapsed().as_secs_f64();
        }

        if *draw_grid {
            // Highlight hierarchy on cursor position.
            'highlight_cursor: {
                let Some(cursor_pos) = ui.input(|i| i.pointer.interact_pos()) else {
                    break 'highlight_cursor;
                };

                let cursor_pos: [f32; 2] = to_world(cursor_pos, offset, zoom);
                let mut all_rect = AabbRect::from_sphere([0., 0.], self.area_radius);

                if !all_rect.contains(&cursor_pos) {
                    break 'highlight_cursor;
                }

                let convert_rect = |rect: &AabbRect<[f32; 2]>| {
                    let min = to_screen((*rect.min()).into(), offset, zoom);
                    let max = to_screen((*rect.max()).into(), offset, zoom);

                    egui::Rect::from_min_max(min.into(), max.into())
                };
                let draw_rect = |rect: &AabbRect<[f32; 2]>, color: egui::Color32| {
                    p.rect_stroke(convert_rect(rect), 0.0, Stroke { width: 3.0, color });
                };

                let mut rects = Vec::new();
                let mut other_rects = Vec::new();

                let node = self.bsp.query_with_hierarchy(
                    self.bsp.root(),
                    &cursor_pos,
                    |split, is_plus| {
                        let mut other = all_rect;

                        if is_plus {
                            all_rect.apply_split_plus(split.axis(), split.value);
                            other.apply_split_minus(split.axis(), split.value);
                        } else {
                            all_rect.apply_split_minus(split.axis(), split.value);
                            other.apply_split_plus(split.axis(), split.value);
                        }

                        rects.push(all_rect);
                        other_rects.push(other);
                    },
                );

                let mut leaf_bound = *self.bsp.leaf_bound(node);
                leaf_bound.apply_intersection(&AabbRect::from_sphere([0., 0.], self.area_radius));

                rects.pop();

                for (base, rect) in rects.iter().enumerate() {
                    const PALLETE: &[[u8; 3]] = &[
                        [0, 255, 0],
                        [0, 255, 255],
                        [0, 0, 255],
                        [255, 0, 255],
                        [255, 255, 0],
                        [255, 0, 0],
                    ];
                    let [r, g, b] = PALLETE[base % PALLETE.len()];
                    let color = egui::Color32::from_rgb(r, g, b).gamma_multiply(0.04);

                    p.rect_filled(convert_rect(rect), 0., color);
                }

                other_rects.push(leaf_bound);

                // Render big text at position to visualize hierarchy

                for (depth, &rect) in other_rects.iter().enumerate().rev() {
                    let center = rect.center();
                    let mut size = rect.length(0).min(rect.length(1)) / 8.;
                    size *= zoom;

                    p.text(
                        to_screen(center.into(), offset, zoom).into(),
                        egui::Align2::CENTER_CENTER,
                        format!("{}", (depth + 1).min(other_rects.len() - 1)),
                        egui::FontId::proportional(size.clamp(14., 36.)),
                        egui::Color32::WHITE,
                    );
                }

                draw_rect(&leaf_bound, egui::Color32::WHITE);
            }

            // Render text on leaf nodes

            self.bsp
                .visit_leaves_with_depth(self.bsp.root(), |depth, leaf| {
                    let bound = self.bsp.leaf_bound(leaf);
                    let clamp_at = self.area_radius;
                    let min = bound.min().map(|x| x.clamp(-clamp_at, clamp_at));
                    let max = bound.max().map(|x| x.clamp(-clamp_at, clamp_at));

                    let min = to_screen(min.into(), offset, zoom);
                    let max = to_screen(max.into(), offset, zoom);

                    const STEPS: u16 = 16;
                    const COLOR_MAX: u8 = 128;
                    let red = (depth * COLOR_MAX as u16 / STEPS).min(128).max(STEPS) as u8;

                    // Grediate from green to red
                    let color = egui::Color32::from_rgb(red, 128 - red, 0);

                    p.debug_text(
                        min.into(),
                        egui::Align2::LEFT_TOP,
                        color.linear_multiply(2.),
                        format!(
                            "{:?}/{} id_{}v{}",
                            depth,
                            self.bsp.leaf_len(leaf),
                            leaf.0.as_ffi() & !0u32 as u64,
                            leaf.0.as_ffi() >> 32
                        ),
                    );

                    p.rect_stroke(
                        egui::Rect::from_min_max(min.into(), max.into()),
                        0.0,
                        Stroke { width: 1.0, color },
                    );
                });

            // Highlight tests that matches the shape exactly.

            if let Some((center, shape, margin)) = hit_test {
                self.bsp.query_shape(&center, &shape, margin, |leaf| {
                    let bound = self.bsp.leaf_bound(leaf);
                    let min = to_screen((*bound.min()).into(), offset, zoom);
                    let max = to_screen((*bound.max()).into(), offset, zoom);

                    p.rect(
                        egui::Rect::from_min_max(min.into(), max.into()),
                        0.0,
                        egui::Color32::from_rgba_unmultiplied(0, 0, 255, 21),
                        Stroke {
                            width: 1.0,
                            color: egui::Color32::BLUE,
                        },
                    );
                });
            }

            // Highlight tests that was actually hit

            for leaf in hit_test_grids.iter().cloned() {
                let mut bound = *self.bsp.leaf_bound(leaf);
                bound.apply_intersection(&AabbRect::from_sphere([0., 0.], self.area_radius));

                let min = to_screen((*bound.min()).into(), offset, zoom);
                let max = to_screen((*bound.max()).into(), offset, zoom);

                p.rect(
                    egui::Rect::from_min_max(min.into(), max.into()),
                    0.0,
                    egui::Color32::from_rgba_unmultiplied(255, 0, 0, 18),
                    Stroke {
                        width: 1.0,
                        color: egui::Color32::RED,
                    },
                );
            }
        }

        // Draw cage
        p.circle_stroke(
            to_screen([0., 0.].into(), offset, zoom).into(),
            self.area_radius * zoom,
            Stroke {
                color: egui::Color32::DARK_GRAY,
                width: 1.0,
            },
        );

        // Draw boids
        for (_entity, (boid, is_predator)) in self.ecs.query::<(&BoidKinetic, &IsPredator)>().iter()
        {
            let (color, size, arrow_len) = if is_predator.0 {
                (PREDATOR_COLOR, PREDATOR_SIZE, PREDATOR_ARROW_LEN)
            } else {
                (BOID_COLOR, BOID_SIZE, BOID_ARROW_LEN)
            };

            let pos = to_screen(boid.pos, offset, zoom);
            let arrow_dst = boid.pos + boid.vel.normalize() * arrow_len;
            let mut arrow_dst = to_screen(arrow_dst, offset, zoom);

            arrow_dst[0] -= pos[0];
            arrow_dst[1] -= pos[1];

            p.arrow(
                pos.into(),
                arrow_dst.into(),
                egui::Stroke { width: 1.0, color },
            );

            p.circle(
                pos.into(),
                size * zoom,
                color.gamma_multiply(0.5),
                Stroke { color, width: 1. },
            );
        }

        if let Some((origin, shape, ..)) = hit_test {
            // Render hit test region
            {
                let color_bg = egui::Color32::from_rgba_unmultiplied(255, 0, 255, 17);
                let stroke = egui::Stroke {
                    width: 1.,
                    color: Color32::from_rgb(255, 0, 255),
                };

                draw_trace_shape(&p, origin, shape, color_bg, stroke, [0.; 2], offset, zoom);
            }

            // Visualize query hit elements
            let shape = self.bsp.context.shape;

            for elem_id in hit_test_elems.iter().cloned() {
                let elem = &self.bsp[elem_id];

                draw_trace_shape(
                    &p,
                    *elem.pos(),
                    shape,
                    Color32::from_rgba_unmultiplied(255, 255, 0, 13),
                    egui::Stroke {
                        width: 1.,
                        color: Color32::YELLOW,
                    },
                    [0.; 2], // TODO: element shape offset
                    offset,
                    zoom,
                );
            }

            // Hit test result text

            let cursor_pos = ui.input(|i| i.pointer.interact_pos()).unwrap_or_default();
            p.debug_text(
                cursor_pos,
                egui::Align2::LEFT_TOP,
                egui::Color32::WHITE,
                format!("Hit test: {:.2}us", hit_test_elapsed * 1e6),
            );
        }

        resp
    }
}

#[allow(clippy::too_many_arguments)]
fn draw_trace_shape(
    p: &egui::Painter,
    mut origin: [f32; 2],
    shape: TraceShape<[f32; 2]>,
    color_bg: egui::Color32,
    stroke: egui::Stroke,
    shape_offset: [f32; 2],
    offset: [f32; 2],
    zoom: f32,
) {
    origin = origin.add(&shape_offset);

    match shape {
        TraceShape::Aabb(ext) => {
            let aabb = AabbRect::from_extent(origin, ext);
            let min = to_screen((*aabb.min()).into(), offset, zoom);
            let max = to_screen((*aabb.max()).into(), offset, zoom);

            p.rect(
                egui::Rect::from_min_max(min.into(), max.into()),
                0.,
                color_bg,
                stroke,
            );
        }
        TraceShape::Sphere(rad) => {
            let center = to_screen(origin.into(), offset, zoom);
            let rad = rad * zoom;

            p.circle(center.into(), rad, color_bg, stroke);
        }
        TraceShape::Capsule { dir, radius } => {
            // 1. Draw two circles connects start-end point
            // 2. Draw a line between two circles
            //    - This is a line connects two circles' center, and offset by
            //      radius to its orthogonal direction respectively.

            let start = to_screen(origin.into(), offset, zoom);
            let end = to_screen(origin.add(&dir.calc_v_dir()).into(), offset, zoom);

            let radius = radius * zoom;
            let v_offset = dir.u_dir().amp(radius.abs());

            let v_offset_1 = [v_offset[1], -v_offset[0]];
            let v_offset_2 = [-v_offset[1], v_offset[0]];

            p.circle(start.into(), radius, color_bg, stroke);
            p.circle(end.into(), radius, color_bg, stroke);

            for offset in [v_offset_1, v_offset_2] {
                let start = start.add(&offset);
                let end = end.add(&offset);

                p.line_segment([start, end].map(Into::into), stroke);
            }
        }
    }
}

pub fn to_world(screen: egui::Pos2, offset: [f32; 2], zoom: f32) -> [f32; 2] {
    [
        (screen[0] - offset[0]) / zoom,
        (screen[1] - offset[1]) / zoom,
    ]
}

pub fn to_screen(world: Vector2<f32>, offset: [f32; 2], zoom: f32) -> [f32; 2] {
    [world[0] * zoom + offset[0], world[1] * zoom + offset[1]]
}
