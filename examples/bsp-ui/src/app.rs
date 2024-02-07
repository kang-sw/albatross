mod boids;

use egui::RichText;

#[derive(Default)]
pub struct TemplateApp {
    // Example stuff:
    model: boids::Model,
    render_opt: boids::RenderOption,
    spawning_predetor: bool,
    once: std::sync::OnceLock<()>,
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
        self.once.get_or_init(|| {
            let screen_size = ctx.available_rect().size();
            self.render_opt.offset = [screen_size.x / 2.0, screen_size.y / 2.0];
        });

        ctx.request_repaint();

        // Simulate boids
        self.model.tick();

        egui::CentralPanel::default().show(ctx, |ui| {
            // Render boids / grids
            let resp = self.model.draw(ui, &self.render_opt);

            if resp.dragged_by(egui::PointerButton::Secondary) {
                let pos = ctx.input(|i| i.pointer.interact_pos().unwrap());
                let pos = boids::to_world(pos, self.render_opt.offset, self.render_opt.zoom);

                self.model.spawn_boids(
                    if self.spawning_predetor { 1 } else { 10 },
                    pos,
                    self.spawning_predetor,
                );
            }
            if resp.dragged_by(egui::PointerButton::Primary) {
                let delta = resp.drag_delta();
                self.render_opt.offset[0] += delta.x;
                self.render_opt.offset[1] += delta.y;
            }
            if resp.hovered() {
                let zoom = ctx.input(|i| i.zoom_delta());
                self.render_opt.zoom *= zoom;
            }
            if resp.dragged_by(egui::PointerButton::Middle) {
                let delta = resp.drag_delta();
                self.render_opt.zoom *= 1.0 + (-delta.y + delta.x) / 400.0;
            }
        });

        egui::Window::new("Boid").show(ctx, |ui| {
            if ui.input(|i| i.key_pressed(egui::Key::Tab)) {
                self.spawning_predetor = !self.spawning_predetor;
            }

            if ui
                .selectable_label(
                    self.spawning_predetor,
                    format!(
                        "Spawning {}",
                        if self.spawning_predetor {
                            "Predetor"
                        } else {
                            "Boid"
                        }
                    ),
                )
                .clicked()
            {
                self.spawning_predetor = !self.spawning_predetor;
            }

            egui::CollapsingHeader::new("Stats")
                .default_open(true)
                .show(ui, |ui| {
                    let stat = self.model.stats().back().unwrap();
                    let label_value_pairs = [
                        ("Count", stat.elem_count.to_string()),
                        ("Tick Time", format!("{:.3} ms", stat.tick_time * 1000.)),
                        (
                            "Avg Query Time",
                            format!("{:.6} ms", stat.avg_query_time * 1000.),
                        ),
                        (
                            "Avg Step Time",
                            format!("{:.6} ms", stat.avg_step_time * 1000.),
                        ),
                        ("Elem Count", stat.elem_count.to_string()),
                        (
                            "Optimize Time",
                            format!("{:.6} ms", stat.optimize_time * 1000.),
                        ),
                        ("Verify Time", format!("{:.3} ms", stat.verify_time * 1000.)),
                    ];

                    for (label, value) in label_value_pairs.iter() {
                        ui.columns(2, |cols| {
                            cols[0].label(*label);
                            cols[1].monospace(RichText::new(value).color(egui::Color32::WHITE));
                        });
                    }
                });

            egui::CollapsingHeader::new("Boids")
                .default_open(true)
                .show(ui, |ui| {
                    let mut speed = self.model.tick_delta * 60.0;
                    let label_param_pairs = [
                        ("Simulation Speed", &mut speed),
                        ("Max Speed", &mut self.model.max_speed),
                        ("Predetor Avoidance", &mut self.model.predetor_avoidance),
                        ("Area Radius", &mut self.model.area_radius),
                        ("View Radius", &mut self.model.view_radius),
                        ("Near Radius", &mut self.model.near_radius),
                        ("Cohesion Force", &mut self.model.cohesion_force),
                        ("Align Force", &mut self.model.align_force),
                        ("Separation Force", &mut self.model.separation_force),
                    ];

                    for (label, param) in label_param_pairs {
                        ui.columns(2, |cols| {
                            cols[0].label(label);
                            cols[1].add(
                                egui::DragValue::new(param)
                                    .speed(0.01)
                                    .clamp_range(0.01..=1e3),
                            );
                        });
                    }

                    self.model.tick_delta = speed / 60.0;
                });

            egui::CollapsingHeader::new("Visualize")
                .default_open(true)
                .show(ui, |ui| {});

            guidance(ui);
        });
    }
}

fn guidance(ui: &mut egui::Ui) {
    ui.separator();

    ui.label("Write click and drag to spawn boids.");
    ui.label("Tap to switch boid type.");
    ui.label("Left click and drag to pan.");
    ui.label("Middle click and drag, or control + wheel to zoom.");

    ui.separator();

    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.label("Author: ");
        ui.hyperlink_to(
            "kang-sw",
            "https://github.com/kang-sw/mylib/tree/master/examples/bsp-ui",
        );
    });
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
