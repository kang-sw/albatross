mod boids;

use albatross::{
    bitindex::BitIndexSet,
    bsp::{OptimizeParameter, TraceShape},
    primitive::{AabbRect, DirectionSegment},
};
use egui::RichText;
use web_time::Instant;

use self::boids::BOID_SIZE;

#[derive(Default)]
pub struct TemplateApp {
    // Example stuff:
    model: boids::Model,
    render_opt: boids::RenderOption,
    spawning_predator: bool,
    once: std::sync::OnceLock<()>,
    tick_time: f64,
    collision_test: CollisionTestMode,
    collision_test_margin: f32,
    drag_start_pos: egui::Pos2,
}

#[derive(Default)]
enum CollisionTestMode {
    #[default]
    Aabb,
    Sphere,
    Capsule(f32),
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

            ctx.style_mut(|style| {
                style.visuals = egui::Visuals::dark();
            });

            self.model.spawn_boids(100, [0., 0.], false);
        });

        ctx.request_repaint();

        let start_time = Instant::now();

        // Simulate boids
        self.model.tick();

        egui::CentralPanel::default().show(ctx, |ui| {
            // Render boids / grids
            let resp = self.model.draw(ui, &self.render_opt);

            /* ------------------------------------ Spawning ------------------------------------ */

            if resp.dragged_by(egui::PointerButton::Secondary)
                || resp.clicked_by(egui::PointerButton::Secondary)
            {
                let was_click = resp.clicked_by(egui::PointerButton::Secondary);

                let pos = ctx.input(|i| i.pointer.interact_pos().unwrap());
                let pos = boids::to_world(pos, self.render_opt.offset, self.render_opt.zoom);

                self.model.spawn_boids(
                    if self.spawning_predator {
                        1
                    } else {
                        10 * if was_click { 10 } else { 1 }
                    },
                    pos,
                    self.spawning_predator,
                );
            }

            /* --------------------------------- Screen Control --------------------------------- */

            if resp.dragged_by(egui::PointerButton::Primary) {
                let delta = resp.drag_delta();
                self.render_opt.offset[0] += delta.x;
                self.render_opt.offset[1] += delta.y;
            }

            let mut new_zoom = if resp.hovered() {
                let zoom = ctx.input(|i| i.zoom_delta());
                (zoom != 1.).then_some(self.render_opt.zoom * zoom)
            } else {
                None
            };

            if new_zoom.is_none() {
                let delta = ctx.input(|i| i.raw_scroll_delta.y);
                if delta != 0.0 {
                    new_zoom = Some(self.render_opt.zoom.powf(1. + delta / 5000.0));
                }
            }

            if let Some(zoom) = new_zoom {
                let cursor_pos = ctx.input(|i| i.pointer.interact_pos().unwrap_or_default());

                // Cursor position's world position remain same before/after zooming
                let before_world =
                    boids::to_world(cursor_pos, self.render_opt.offset, self.render_opt.zoom);
                let after_world = boids::to_world(cursor_pos, self.render_opt.offset, zoom);

                self.render_opt.offset[0] -= (before_world[0] - after_world[0]) * zoom;
                self.render_opt.offset[1] -= (before_world[1] - after_world[1]) * zoom;
                self.render_opt.zoom = zoom;
            }

            /* ---------------------------- Collision Test Rendering ---------------------------- */

            if resp.drag_started_by(egui::PointerButton::Middle) {
                self.drag_start_pos = resp.interact_pointer_pos().unwrap();
            }

            if resp.dragged_by(egui::PointerButton::Middle) {
                let cur_pos = resp.interact_pointer_pos().unwrap();
                let (center, trace) = match self.collision_test {
                    CollisionTestMode::Aabb => {
                        // Change AABB to center + extent expression
                        let aabb = AabbRect::new(self.drag_start_pos.into(), cur_pos.into());
                        (aabb.center(), TraceShape::Aabb(aabb.extent()))
                    }
                    CollisionTestMode::Sphere => {
                        let delta = cur_pos - self.drag_start_pos;
                        (
                            self.drag_start_pos.into(),
                            TraceShape::Sphere(delta.length()),
                        )
                    }
                    CollisionTestMode::Capsule(radius) => {
                        let dir = DirectionSegment::new((cur_pos - self.drag_start_pos).into());

                        (
                            self.drag_start_pos.into(),
                            TraceShape::Capsule { dir, radius },
                        )
                    }
                };

                self.model
                    .set_hit_test(center, trace, self.collision_test_margin);
            } else {
                self.model.clear_hit_test();
            }
        });

        egui::Window::new("Boid").show(ctx, |ui| {
            ui.input(|i| {
                if i.key_pressed(egui::Key::Tab) {
                    self.spawning_predator = !self.spawning_predator;
                } else if i.key_pressed(egui::Key::Space) {
                    self.model.enable_tick = !self.model.enable_tick;
                }
            });

            egui::widgets::global_dark_light_mode_buttons(ui);

            ui.separator();

            egui::CollapsingHeader::new("Stats")
                .default_open(true)
                .show(ui, |ui| {
                    let stat = self.model.stats().back().unwrap();
                    let label_value_pairs = [
                        ("Count", stat.elem_count.to_string()),
                        ("All", format!("{:_>7.3} ms", self.tick_time * 1000.)),
                        ("Tick", format!("{:_>7.3} ms", stat.tick * 1000.)),
                        ("avg.Query", format!("{:_>7.3} µs", stat.avg_query * 1e6)),
                        ("avg.Step", format!("{:_>7.3} µs", stat.avg_step * 1e6)),
                        ("Optimize", format!("{:_>7.3} µs", stat.optimize * 1e6)),
                        ("Verify", format!("{:_>7.3} µs", stat.verify * 1e6)),
                    ];

                    for (label, value) in label_value_pairs.iter() {
                        ui.columns(2, |cols| {
                            cols[0].label(*label);
                            cols[1].monospace(RichText::new(value).color(egui::Color32::WHITE));
                        });
                    }
                });

            egui::CollapsingHeader::new("Test")
                .default_open(true)
                .show(ui, |ui| {
                    ui.columns(3, |c| {
                        if c[0]
                            .radio(
                                matches!(self.collision_test, CollisionTestMode::Aabb),
                                "AABB",
                            )
                            .clicked()
                        {
                            self.collision_test = CollisionTestMode::Aabb;
                        } else if c[1]
                            .radio(
                                matches!(self.collision_test, CollisionTestMode::Sphere),
                                "Sphere",
                            )
                            .clicked()
                        {
                            self.collision_test = CollisionTestMode::Sphere;
                        } else if c[2]
                            .radio(
                                matches!(self.collision_test, CollisionTestMode::Capsule(_)),
                                "Capsule",
                            )
                            .clicked()
                        {
                            self.collision_test = CollisionTestMode::Capsule(1.0);
                        }
                    });

                    if let CollisionTestMode::Capsule(ref mut rad) = self.collision_test {
                        ui.columns(2, |c| {
                            c[0].label("Capsule Radius");
                            c[1].add(egui::DragValue::new(rad).speed(0.01).clamp_range(0.0..=1e3));
                        })
                    }

                    ui.columns(2, |c| {
                        c[0].label("Collision Query Margin");
                        c[1].add(
                            egui::DragValue::new(&mut self.collision_test_margin)
                                .speed(0.05)
                                .clamp_range(0.0..=1e3),
                        );
                    })
                });

            egui::CollapsingHeader::new("Boids")
                .default_open(true)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if ui.button("Clear").clicked() {
                            self.model.clear();
                        }

                        ui.checkbox(&mut self.spawning_predator, "Spawn Predator?");
                        ui.checkbox(&mut self.model.enable_tick, "Enable Simulation?");
                    });

                    let mut speed = self.model.tick_delta * 60.0;
                    let label_param_pairs = [
                        ("Simulation Speed", &mut speed),
                        ("Max Speed", &mut self.model.max_speed),
                        ("Predator Avoidance", &mut self.model.predator_avoidance),
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

            egui::CollapsingHeader::new("Bsp")
                .default_open(true)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        if ui.button("Collapse All!").clicked() {
                            self.model.collapse_all();
                        }

                        ui.checkbox(&mut self.render_opt.draw_grid, "Draw Grid?");
                    });

                    ui.separator();

                    // Collision shape
                    {
                        ui.label("Trace Shape");
                        ui.add_space(3.);

                        let ext = *self.model.extent();
                        ui.columns(3, |c| {
                            let new_ext = if c[0]
                                .radio(matches!(ext, TraceShape::Aabb(..)), "AABB")
                                .clicked()
                            {
                                Some(TraceShape::Aabb([BOID_SIZE * 2., BOID_SIZE * 2.]))
                            } else if c[1]
                                .radio(matches!(ext, TraceShape::Sphere(..)), "Sphere")
                                .clicked()
                            {
                                Some(TraceShape::Sphere(BOID_SIZE))
                            } else if c[2]
                                .radio(matches!(ext, TraceShape::Capsule { .. }), "Capsule")
                                .clicked()
                            {
                                Some(TraceShape::Capsule {
                                    dir: DirectionSegment::new([0., BOID_SIZE * 2.]),
                                    radius: BOID_SIZE,
                                })
                            } else {
                                None
                            };

                            if let Some(ext) = new_ext {
                                *self.model.extent_mut() = ext;
                            }
                        });

                        match *self.model.extent() {
                            TraceShape::Aabb([mut x, mut y]) => {
                                let mut changed = false;
                                for (label, value) in [("x", &mut x), ("y", &mut y)] {
                                    changed |= ui.columns(2, |cols| {
                                        cols[0].label(label);
                                        cols[1]
                                            .add(egui::DragValue::new(value).speed(0.001))
                                            .changed()
                                    });
                                }

                                if changed {
                                    *self.model.extent_mut() = TraceShape::Aabb([x, y]);
                                }
                            }
                            TraceShape::Sphere(mut rad) => {
                                let changed = ui.columns(2, |cols| {
                                    cols[0].label("Radius");
                                    cols[1]
                                        .add(egui::DragValue::new(&mut rad).speed(0.001))
                                        .changed()
                                });

                                if changed {
                                    *self.model.extent_mut() = TraceShape::Sphere(rad);
                                }
                            }
                            TraceShape::Capsule { dir, mut radius } => {
                                let mut changed = false;
                                let [mut x, mut y] = dir.calc_v_dir();

                                for (label, value) in [("x", &mut x), ("y", &mut y)] {
                                    changed |= ui.columns(2, |cols| {
                                        cols[0].label(label);
                                        cols[1]
                                            .add(egui::DragValue::new(value).speed(0.001))
                                            .changed()
                                    });
                                }

                                changed |= ui.columns(2, |cols| {
                                    cols[0].label("Radius");
                                    cols[1]
                                        .add(egui::DragValue::new(&mut radius).speed(0.001))
                                        .changed()
                                });

                                if changed {
                                    *self.model.extent_mut() = TraceShape::Capsule {
                                        dir: DirectionSegment::new([x, y]),
                                        radius,
                                    };
                                }
                            }
                        }
                    }

                    ui.separator();

                    let OptimizeParameter {
                        split_threshold,
                        collapse_threshold,
                        max_collapse_height,
                        balancing,
                        split_strategy,
                        minimum_length: minimum_size,
                        suboptimal_split_count: short_axis_fallback,
                        square_split_axes,
                        snap_size,
                        balancing_start_height,
                        ..
                    } = &mut self.model.tree_optimize;

                    let label_params_normal = [("Balancing", balancing)];
                    for (label, params) in label_params_normal {
                        ui.columns(2, |cols| {
                            cols[0].label(label);
                            cols[1].add(egui::Slider::new(params, 0.0..=1.0))
                        });
                    }

                    // for (label, param) in label_params {
                    //     ui.columns(2, |cols| {
                    //         cols[0].label(label);

                    //         cols[1].columns(3, |cols| {
                    //             use albatross::ControlIntensity::*;

                    //             cols[0].selectable_value(param, Disable, "Disable");
                    //             cols[1].selectable_value(param, Moderate, "Moderate");
                    //             cols[2].selectable_value(param, Extreme, "Extreme");
                    //         });
                    //     });
                    // }

                    let label_value_pairs = [
                        ("Split Threshold", split_threshold),
                        ("Collapse Threshold", collapse_threshold),
                    ];

                    for (label, value) in label_value_pairs {
                        ui.columns(2, |cols| {
                            cols[0].label(label);
                            cols[1].add(egui::DragValue::new(value).speed(1).clamp_range(0..=100));
                        });
                    }

                    let mut short_axis_fallback_u16 = *short_axis_fallback as u16;
                    let label_value_pairs = [
                        ("Max Collapse Height", max_collapse_height),
                        ("Axis Find Fallback", &mut short_axis_fallback_u16),
                        ("Balancing Start Height", balancing_start_height),
                    ];

                    for (label, value) in label_value_pairs {
                        ui.columns(2, |cols| {
                            cols[0].label(label);
                            cols[1].add(egui::DragValue::new(value).speed(1).clamp_range(0..=100));
                        });
                    }

                    *short_axis_fallback = short_axis_fallback_u16 as u8;

                    let mut collapse_all = false;
                    ui.columns(2, |cols| {
                        use albatross::bsp::SplitStrategy::*;
                        let prev = split_strategy.clone();

                        cols[0].label("Split Strategy");
                        cols[1].columns(3, |cols| {
                            cols[0].selectable_value(split_strategy, Average, "Average");
                            cols[1].selectable_value(
                                split_strategy,
                                ClusterMedian,
                                "Cluster Median",
                            );
                            cols[2].selectable_value(
                                split_strategy,
                                SpatialMedian,
                                "Spatial Median",
                            );
                        });

                        if prev != *split_strategy {
                            collapse_all = true;
                        }
                    });
                    let label_value_pairs =
                        [("Minimum Size", minimum_size), ("Snap Size", snap_size)];

                    for (label, value) in label_value_pairs {
                        ui.columns(2, |cols| {
                            cols[0].label(label);
                            cols[1]
                                .add(egui::DragValue::new(value).speed(0.01).clamp_range(0..=100));
                        });
                    }

                    {
                        let mut square_shape = !square_split_axes.is_empty();
                        if ui
                            .checkbox(&mut square_shape, "Square Split Axes?")
                            .changed()
                        {
                            *square_split_axes = if square_shape {
                                BitIndexSet::all()
                            } else {
                                BitIndexSet::empty()
                            };

                            collapse_all = true;
                        }
                    }

                    if collapse_all {
                        self.model.collapse_all();
                    }
                });

            guidance(ui);
        });

        self.tick_time = start_time.elapsed().as_secs_f64();
    }
}

fn guidance(ui: &mut egui::Ui) {
    ui.separator();

    ui.label("Right click and drag to spawn boids.");
    ui.label("Tap t~o switch boid type.");
    ui.label("Left click and drag to pan.");
    ui.label("Wheel, or control + wheel to zoom.");

    ui.separator();

    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 0.0;
        ui.label("Author: ");
        ui.hyperlink_to("kang-sw", "https://github.com/kang-sw");
        ui.label("  (");
        ui.hyperlink_to(
            "source code",
            "https://github.com/kang-sw/mylib/tree/master/examples/bsp-ui",
        );
        ui.label(")");
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
