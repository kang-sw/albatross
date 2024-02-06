#[derive(Default)]
pub struct TemplateApp {
    // Example stuff:
    model: boids::Model,
    render_opt: boids::RenderOption,
    spawn_count: usize,
}

impl TemplateApp {
    /// Called once before the first frame.
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Default::default()
    }
}

impl eframe::App for TemplateApp {
    /// Called each time the UI needs repainting, which may be many times per second.
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        // Simulate boids
        self.model.tick();

        let resp = egui::CentralPanel::default().show(ctx, |ui| {
            // Render boids / grids
            self.model.draw(ui, &self.render_opt);
        });

        if resp.response.clicked() {
            let pos = ctx.input(|i| i.pointer.interact_pos().unwrap());
            let pos = boids::to_world(pos, self.render_opt.offset, self.render_opt.zoom);

            self.model.spawn_boids(self.spawn_count.max(10), pos);
        }
        if resp.response.dragged() {
            let delta = resp.response.drag_delta();
            self.render_opt.offset[0] += delta.x / self.render_opt.zoom;
            self.render_opt.offset[1] += delta.y / self.render_opt.zoom;
        }

        egui::Window::new("Settings").show(ctx, |ui| {
            egui::CollapsingHeader::new("Boids")
                .default_open(true)
                .show(ui, |ui| {});

            egui::CollapsingHeader::new("Visualize")
                .default_open(true)
                .show(ui, |ui| {});

            powered_by_egui_and_eframe(ui);
        });
    }
}

fn powered_by_egui_and_eframe(ui: &mut egui::Ui) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.label("Powered by ");
        ui.hyperlink_to("egui", "https://github.com/emilk/egui");
        ui.label(" and ");
        ui.hyperlink_to(
            "eframe",
            "https://github.com/emilk/egui/tree/master/crates/eframe",
        );
        ui.label(".");
    });
}

mod boids {
    use std::collections::VecDeque;

    use albatross::{bsp, primitive::AabbRect};
    use nalgebra::Vector2;
    use web_time::Instant;

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

    pub struct Model {
        ecs: hecs::World,
        bsp: bsp::Tree<BspData>,

        rand: fastrand::Rng,

        pub area_radius: f32,
        pub view_radius: f32,

        pub near_radius: f32,

        pub cohesion_force: f32,
        pub align_force: f32,
        pub separation_force: f32,

        pub tree_optimize: bsp::OptimizeParameter,

        pub stat_max_record: usize,

        stats: VecDeque<Stats>,
    }

    #[derive(Default)]
    pub struct Stats {
        pub tick_time: f64,
        pub avg_query_time: f64,
        pub avg_step_time: f64,
        pub elem_count: usize,
        pub optimize_time: f64,
    }

    impl Default for Model {
        fn default() -> Self {
            Self {
                ecs: Default::default(),
                bsp: Default::default(),
                rand: fastrand::Rng::with_seed(0),

                area_radius: 100.,
                view_radius: 5.,
                near_radius: 0.5,
                cohesion_force: 0.1,
                align_force: 0.1,
                separation_force: 0.1,

                stat_max_record: 120,

                tree_optimize: bsp::OptimizeParameter::moderate(20),

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
            let dt = 1. / 60.;

            if self.stats.len() >= self.stat_max_record {
                self.stats.pop_front();
            }

            self.stats.push_back(Default::default());

            let start_time = Instant::now();

            self.calc_force();
            self.step(dt);
            self.refresh_tree();

            self.wr_stat().tick_time = start_time.elapsed().as_secs_f64();
        }

        fn calc_force(&mut self) {
            let start_time;
            let mut query_count;

            {
                let mut q_elems = self.ecs.query::<(&BoidKinetic, &bsp::ElementIndex)>();
                let q_elems = q_elems.view();

                let mut adjacent_kinetics = Vec::new();

                start_time = Instant::now();
                query_count = 0;

                for (entity, force) in self.ecs.query::<&mut BoidForce>().into_iter() {
                    query_count += 1;

                    let (boid, tree_key) = q_elems.get(entity).unwrap();
                    let region = AabbRect::new_circular(boid.pos.into(), self.view_radius);

                    adjacent_kinetics.clear();

                    // Query all nearby boids
                    self.bsp.query_region(&region, |tree| {
                        for (elem_id, elem) in self.bsp.leaf_iter(tree) {
                            if elem_id == *tree_key {
                                continue;
                            }

                            let (other_boid, _) = q_elems.get(elem.entity).unwrap();
                            let diff = other_boid.pos - boid.pos;
                            let distance = diff.norm();

                            if distance > self.view_radius {
                                continue;
                            }

                            adjacent_kinetics.push((other_boid, distance));
                        }
                    });

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
                        .filter(|(_, distance)| *distance < self.near_radius)
                        .fold(Vector2::zeros(), |avg, (kin, distance)| {
                            avg + (boid.pos - kin.pos) * (1. / distance.max(0.0001))
                        });

                    force.acc = cohesion_dir * self.cohesion_force
                        + align_dir * self.align_force
                        + separation_dir * self.separation_force;
                }
            }

            self.wr_stat().avg_query_time = start_time.elapsed().as_secs_f64() / query_count as f64;
            self.wr_stat().elem_count = query_count;
        }

        fn step(&mut self, dt: f64) {
            let step_start = Instant::now();

            for (_entity, (kin, force, tree_key)) in self
                .ecs
                .query::<(&mut BoidKinetic, &BoidForce, &bsp::ElementIndex)>()
                .iter()
            {
                kin.vel += force.acc * dt as f32;
                kin.pos += kin.vel * dt as f32;

                self.bsp.get_mut(*tree_key).unwrap().set_pos(kin.pos.into());
            }

            self.wr_stat().avg_step_time =
                step_start.elapsed().as_secs_f64() / self.ecs.len() as f64;
        }

        fn refresh_tree(&mut self) {
            let optimize_start = Instant::now();
            self.bsp.optimize(&self.tree_optimize, |_| {});
            self.wr_stat().optimize_time = optimize_start.elapsed().as_secs_f64();
        }

        pub fn spawn_boids(&mut self, count: usize, at: [f32; 2]) {
            for _ in 0..count {
                let entity = self.ecs.spawn((BoidKinetic {
                    pos: {
                        let mut v = at;
                        v[0] += self.rand.f32() - 0.5;
                        v[1] += self.rand.f32() - 0.5;
                        v.into()
                    },
                    vel: [0; 2].map(|_| self.rand.f32() - 0.5).into(),
                },));
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
                zoom: 1.,
                draw_grid: Default::default(),
            }
        }
    }

    const BOID_SIZE: f32 = 0.2;
    const BOID_ARROW_LEN: f32 = 0.5;
    const BOID_COLOR: egui::Color32 = egui::Color32::WHITE;

    impl Model {
        pub fn draw(
            &self,
            ui: &mut egui::Ui,
            RenderOption {
                offset,
                zoom,
                draw_grid,
            }: &RenderOption,
        ) {
            let (_resp, p) = ui.allocate_painter(ui.available_size(), egui::Sense::hover());
            let offset = *offset;
            let zoom = (*zoom).max(0.001);

            if *draw_grid {
                todo!()
            }

            for (_entity, boid) in self.ecs.query::<&BoidKinetic>().iter() {
                let pos = to_screen(boid.pos, offset, zoom);
                p.circle_filled(pos.into(), BOID_SIZE * zoom, BOID_COLOR);

                let arrow_dst = boid.pos + boid.vel.normalize() * BOID_ARROW_LEN * zoom;
                let arrow_dst = to_screen(arrow_dst, offset, zoom);

                p.arrow(
                    pos.into(),
                    arrow_dst.into(),
                    egui::Stroke {
                        width: 1.0,
                        color: BOID_COLOR,
                    },
                );
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
}
