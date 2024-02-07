use std::collections::{BTreeSet, VecDeque};

use albatross::{bsp, primitive::AabbRect, ControlIntensity};
use egui::Stroke;
use nalgebra::Vector2;
use web_time::Instant;

const BOID_SIZE: f32 = 0.2;
const PREDATOR_SIZE: f32 = 0.6;
const BOID_ARROW_LEN: f32 = 0.5;
const PREDATOR_ARROW_LEN: f32 = 1.0;
const BOID_COLOR: egui::Color32 = egui::Color32::WHITE;
const PREDATOR_COLOR: egui::Color32 = egui::Color32::RED;

struct BspData {
    entity: hecs::Entity,
}

impl bsp::ElementData for BspData {
    type Vector = [f32; 2];
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
    bsp: bsp::Tree<BspData>,

    rand: fastrand::Rng,

    pub tick_delta: f32,

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
}

#[derive(Default)]
pub struct Stats {
    pub tick_time: f64,
    pub avg_query_time: f64,
    pub avg_proc_time: f64,
    pub avg_step_time: f64,
    pub elem_count: usize,
    pub optimize_time: f64,
    pub verify_time: f64,
}

impl Default for Model {
    fn default() -> Self {
        Self {
            ecs: Default::default(),
            bsp: Default::default(),
            rand: fastrand::Rng::with_seed(0),

            tick_delta: 1. / 60.,

            area_radius: 60.,
            view_radius: 2.,
            near_radius: 0.5,
            cohesion_force: 9.,
            align_force: 0.25,
            separation_force: 9.,
            max_speed: 10.,
            predator_avoidance: 25.,

            stat_max_record: 120,
            tree_optimize: bsp::OptimizeParameter::moderate(4).with(|x| {
                x.minimum_size = 1.;
                x.ideal_depth = u16::MAX;
                x.balance_coeff = ControlIntensity::Moderate;
                x.max_collapse_height = u16::MAX;
                x.node_height_coeff = ControlIntensity::Disable;
            }),

            stats: Default::default(),
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
        let dt = self.tick_delta;

        if self.stats.len() >= self.stat_max_record {
            self.stats.pop_front();
        }

        self.stats.push_back(Default::default());

        let start_time = Instant::now();

        self.calc_force();
        self.step(dt as _);
        self.refresh_tree();

        self.wr_stat().tick_time = start_time.elapsed().as_secs_f64();
    }

    fn calc_force(&mut self) {
        let start_time;
        let mut query_count;
        let mut query_time = 0.;
        let mut removed_entities = BTreeSet::new();

        {
            let mut q_elems = self
                .ecs
                .query::<(&BoidKinetic, &bsp::ElementIndex, &IsPredator)>();
            let q_elems = q_elems.view();

            let mut adjacent_kinetics = Vec::new();
            let mut predator_kinetics = Vec::new();

            start_time = Instant::now();
            query_count = 0;

            for (entity, force) in self.ecs.query::<&mut BoidForce>().into_iter() {
                query_count += 1;

                let (boid, tree_key, IsPredator(is_predator)) = q_elems.get(entity).unwrap();
                let region = AabbRect::new_circular(boid.pos.into(), self.view_radius);

                adjacent_kinetics.clear();
                predator_kinetics.clear();

                // Query all nearby boids
                let query_start = Instant::now();

                self.bsp.query_region(&region, |tree| {
                    for (elem_id, elem) in self.bsp.leaf_iter(tree) {
                        if elem_id == *tree_key {
                            continue;
                        }

                        let (other_boid, _, IsPredator(other_is_predator)) =
                            q_elems.get(elem.entity).unwrap();
                        let diff = other_boid.pos - boid.pos;
                        let distance = diff.norm();

                        let view_radius = self.view_radius * if *is_predator { 2. } else { 1. };
                        if distance > view_radius {
                            continue;
                        }

                        if *is_predator && !*other_is_predator && distance < PREDATOR_SIZE {
                            predator_kinetics.push((elem.entity, other_boid, distance));
                        }

                        if !*is_predator && *other_is_predator {
                            predator_kinetics.push((elem.entity, other_boid, distance));
                        } else {
                            adjacent_kinetics.push((other_boid, distance));
                        }
                    }
                });

                query_time += query_start.elapsed().as_secs_f64();
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
            let tree_key = *self.ecs.get::<&bsp::ElementIndex>(entity).unwrap();
            self.bsp.remove(tree_key);
            self.ecs.despawn(entity).unwrap();
        }

        self.wr_stat().avg_query_time = query_time / query_count as f64;
        self.wr_stat().avg_proc_time = start_time.elapsed().as_secs_f64() / query_count as f64;
        self.wr_stat().elem_count = query_count;
    }

    fn step(&mut self, dt: f64) {
        let step_start = Instant::now();

        for (_entity, (kin, force, tree_key, predator)) in self
            .ecs
            .query::<(
                &mut BoidKinetic,
                &BoidForce,
                &bsp::ElementIndex,
                &IsPredator,
            )>()
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

        self.wr_stat().avg_step_time = step_start.elapsed().as_secs_f64() / self.ecs.len() as f64;
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
        self.wr_stat().optimize_time = optimize_start.elapsed().as_secs_f64();

        let verify_start = Instant::now();
        assert_eq!(
            self.ecs.len(),
            self.bsp.__debug_verify_tree_state().unwrap() as u32,
            "Invalid after optimize"
        );
        self.wr_stat().verify_time = verify_start.elapsed().as_secs_f64();
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

        if *draw_grid {
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
                        color,
                        format!("{:?}/{}", depth, self.bsp.leaf_len(leaf)),
                    );

                    p.rect_stroke(
                        egui::Rect::from_min_max(min.into(), max.into()),
                        0.0,
                        Stroke { width: 1.0, color },
                    );
                });
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
            p.circle_filled(pos.into(), size * zoom, color);

            let arrow_dst = boid.pos + boid.vel.normalize() * arrow_len;
            let mut arrow_dst = to_screen(arrow_dst, offset, zoom);

            arrow_dst[0] -= pos[0];
            arrow_dst[1] -= pos[1];

            p.arrow(
                pos.into(),
                arrow_dst.into(),
                egui::Stroke { width: 1.0, color },
            );
        }

        // Draw boid sights
        for (_entity, (boid, predetor)) in self.ecs.query::<(&BoidKinetic, &IsPredator)>().iter() {
            let pos = to_screen(boid.pos, offset, zoom);

            let Some(cursor_pos) = ui.input(|i| i.pointer.hover_pos()) else {
                continue;
            };

            if egui::Pos2::from(pos).distance(cursor_pos) > 50. {
                continue;
            }

            p.circle_stroke(
                pos.into(),
                self.view_radius * zoom * if predetor.0 { 2. } else { 1. },
                Stroke {
                    color: egui::Color32::YELLOW,
                    width: 1.0,
                },
            );

            p.circle_stroke(
                pos.into(),
                self.near_radius * zoom,
                Stroke {
                    color: egui::Color32::RED,
                    width: 1.0,
                },
            );
        }

        resp
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
