#[derive(Default)]
pub struct TemplateApp {
    // Example stuff:
    model: boids::Model,
    render_bsp: bool,
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

        egui::CentralPanel::default().show(ctx, |ui| {
            // Render boids / grids
        });

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
    use albatross::{bsp, primitive::AabbRect};
    use nalgebra::Vector2;
    use web_time::Instant;

    struct BspData {
        entity: hecs::Entity,
    }

    impl bsp::ElementData for BspData {
        type Vector = [f32; 2];
    }

    struct BoidState {
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
    }

    impl Default for Model {
        fn default() -> Self {
            Self {
                ecs: Default::default(),
                bsp: Default::default(),
                rand: fastrand::Rng::with_seed(0),
                area_radius: 100.,
                view_radius: 5.,
            }
        }
    }

    /* --------------------------------------- Simulation --------------------------------------- */
    impl Model {
        pub fn tick(&mut self) {
            let dt = 1. / 60.;

            self.calc_force(dt);
            self.step(dt);
            self.update_tree(dt);
        }

        fn calc_force(&mut self, dt: f64) {
            let mut q_elems = self.ecs.query::<(&BoidState, &bsp::ElementIndex)>();
            let mut q_elems = q_elems.view();

            for (entity, force) in self.ecs.query::<&mut BoidForce>().into_iter() {
                let (boid, tree_key) = q_elems.get(entity).unwrap();
                let region = AabbRect::new_circular(boid.pos.into(), self.view_radius);

                self.bsp
                    .query_region(&region, |tree| for elem in self.bsp.leaf_iter(tree) {})
            }
        }

        fn step(&mut self, dt: f64) {}

        fn update_tree(&mut self, dt: f64) {}

        pub fn spawn_boids(&mut self, count: usize, at: [f32; 2]) {
            for _ in 0..count {
                let entity = self.ecs.spawn((BoidState {
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

    struct RenderOption {}

    impl Model {
        pub fn draw(&self, ui: &mut egui::Ui, offset: [f32; 2], zoom: f32) {}
    }
}
