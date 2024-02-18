use crate::{
    collision,
    primitive::{AabbRect, AxisIndex, LineSegment, NumExt, Number, Vector, VectorExt as _},
};

use super::{Context, TraceShape, Tree, TreeElement};

impl<T: Context> Tree<T> {
    pub fn query_shape(
        &self,
        p_pivot: &T::Vector,
        shape: &TraceShape<T::Vector>,
        query_margin: <T::Vector as Vector>::Num,
        visit_leaf: impl FnMut(T::NodeKey),
    ) {
        match shape {
            TraceShape::Aabb(ext) => {
                let region = AabbRect::new_extent(*p_pivot, *ext).extended_by_all(query_margin);
                self.query_region(&region, visit_leaf);
            }
            TraceShape::Sphere(rad) => {
                let region = AabbRect::new_circular(*p_pivot, *rad).extended_by_all(query_margin);
                self.query_region(&region, visit_leaf);
            }
            TraceShape::CapsuleOrCylinder { dir, radius } => {
                let radius = radius.abs();
                let cutter = create_line_region_cutter(
                    *p_pivot,
                    p_pivot.add(&dir.calc_v_dir()),
                    radius + query_margin,
                );
                let region = AabbRect::new(*p_pivot, p_pivot.add(&dir.calc_v_dir()))
                    .extended_by_all(radius + query_margin);
                self.query_region_with_cutter(&region, visit_leaf, cutter);
            }
        }
    }

    pub fn trace(
        &self,
        p_pivot: &T::Vector,
        shape: &TraceShape<T::Vector>,
        query_margin: <T::Vector as Vector>::Num,
        visit: impl FnMut(T::NodeKey, T::ElemKey, &TreeElement<T>),
    ) {
        match shape {
            TraceShape::Aabb(ext) => {
                let region = AabbRect::new_extent(*p_pivot, *ext);
                self.trace_aabb(&region, query_margin, visit);
            }
            TraceShape::Sphere(rad) => {
                self.trace_sphere(p_pivot, *rad, query_margin, visit);
            }
            TraceShape::CapsuleOrCylinder { dir, radius } => {
                let p_end = p_pivot.add(&dir.calc_v_dir());
                if *radius > <T::Vector as Vector>::Num::ZERO {
                    self.trace_capsule(p_pivot, &p_end, *radius, query_margin, visit);
                } else {
                    self.trace_cylinder(p_pivot, &p_end, radius.neg(), query_margin, visit);
                }
            }
        }
    }

    pub fn trace_capsule(
        &self,
        p_start: &T::Vector,
        p_end: &T::Vector,
        s_radius: <T::Vector as Vector>::Num,
        query_margin: <T::Vector as Vector>::Num,
        mut visit: impl FnMut(T::NodeKey, T::ElemKey, &TreeElement<T>),
    ) {
        let line = LineSegment::new(*p_start, *p_end);

        if line.s_norm.is_zero() {
            // treat it as a sphere
            self.trace_sphere(p_start, s_radius, query_margin, visit);
            return;
        }

        let region = AabbRect::new(*p_start, *p_end).extended_by_all(s_radius + query_margin);
        let cutter = create_line_region_cutter(*p_start, *p_end, s_radius + query_margin);

        let visitor = move |node: T::NodeKey| {
            for (elem_id, elem) in self.leaf_iter(node) {
                let com = self.context.extent(elem);
                let elem_pos = elem.pos.add(&com.offset);

                let matches = match com.shape {
                    TraceShape::Sphere(rad) => {
                        collision::check::capsule_sphere(&line, s_radius, &elem_pos, rad)
                    }
                    TraceShape::Aabb(ext) => {
                        collision::check::capsule_aabb_ce(&line, s_radius, &elem_pos, &ext)
                    }
                    TraceShape::CapsuleOrCylinder { dir, radius } => {
                        if radius.is_negative() == false {
                            collision::check::capsule_capsule(
                                &line,
                                s_radius,
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius,
                            )
                        } else {
                            collision::check::cylinder_capsule(
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius.neg(),
                                &line,
                                s_radius,
                            )
                        }
                    }
                };

                if matches {
                    visit(node, elem_id, elem);
                }
            }
        };

        self.query_region_with_cutter(&region, visitor, cutter);
    }

    pub fn trace_sphere(
        &self,
        p_center: &T::Vector,
        s_radius: <T::Vector as Vector>::Num,
        query_margin: <T::Vector as Vector>::Num,
        mut visit: impl FnMut(T::NodeKey, T::ElemKey, &TreeElement<T>),
    ) {
        let region = AabbRect::new_circular(*p_center, s_radius + query_margin);
        self.query_region(&region.extended_by_all(query_margin), |node| {
            for (elem_id, elem) in self.leaf_iter(node) {
                let com = self.context.extent(elem);
                let elem_pos = elem.pos.add(&com.offset);

                let matches = match com.shape {
                    TraceShape::Sphere(elem_rad) => {
                        collision::check::sphere_sphere(p_center, s_radius, &elem_pos, elem_rad)
                    }
                    TraceShape::Aabb(ext) => {
                        let aabb = AabbRect::new_extent(elem_pos, ext);
                        collision::check::aabb_sphere(&aabb, p_center, s_radius)
                    }
                    TraceShape::CapsuleOrCylinder { dir, radius } => {
                        if radius.is_negative() == false {
                            collision::check::capsule_sphere(
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius,
                                p_center,
                                s_radius,
                            )
                        } else {
                            collision::check::cylinder_sphere(
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius.neg(),
                                p_center,
                                s_radius,
                            )
                        }
                    }
                };

                if matches {
                    visit(node, elem_id, elem);
                }
            }
        });
    }

    pub fn trace_aabb(
        &self,
        region: &AabbRect<T::Vector>,
        query_margin: <T::Vector as Vector>::Num,
        mut visit: impl FnMut(T::NodeKey, T::ElemKey, &TreeElement<T>),
    ) {
        let q_center = region.center();
        let q_extent = region.extent();
        self.query_region(&region.extended_by_all(query_margin), move |node| {
            for (elem_id, elem) in self.leaf_iter(node) {
                let com = self.context.extent(elem);
                let elem_pos = elem.pos.add(&com.offset);

                let matches = match com.shape {
                    TraceShape::Sphere(rad) => {
                        collision::check::aabb_sphere(region, &elem_pos, rad)
                    }
                    TraceShape::Aabb(ext) => {
                        let aabb = AabbRect::new_extent(elem_pos, ext);
                        collision::check::aabb_aabb(region, &aabb)
                    }
                    TraceShape::CapsuleOrCylinder { dir, radius } => {
                        if radius.is_negative() == false {
                            collision::check::capsule_aabb_ce(
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius,
                                &q_center,
                                &q_extent,
                            )
                        } else {
                            collision::check::cylinder_aabb_ce(
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius.neg(),
                                &q_center,
                                &q_extent,
                            )
                        }
                    }
                };

                if matches {
                    visit(node, elem_id, elem);
                }
            }
        });
    }

    /// Trace cylindrical shape from `p_start` to `p_end`. It is inherently a capsule
    /// trace, but with
    pub fn trace_cylinder(
        &self,
        p_start: &T::Vector,
        p_end: &T::Vector,
        s_radius: <T::Vector as Vector>::Num,
        query_margin: <T::Vector as Vector>::Num,
        mut visit: impl FnMut(T::NodeKey, T::ElemKey, &TreeElement<T>),
    ) {
        let line = LineSegment::new(*p_start, *p_end);
        let region = AabbRect::new(*p_start, *p_end).extended_by_all(s_radius + query_margin);
        let cutter = create_line_region_cutter(*p_start, *p_end, s_radius + query_margin);

        let visitor = move |node: T::NodeKey| {
            for (elem_id, elem) in self.leaf_iter(node) {
                let com = self.context.extent(elem);
                let elem_pos = elem.pos.add(&com.offset);

                let matches = match com.shape {
                    TraceShape::Sphere(rad) => {
                        collision::check::cylinder_sphere(&line, s_radius, &elem_pos, rad)
                    }
                    TraceShape::Aabb(ext) => {
                        collision::check::cylinder_aabb_ce(&line, s_radius, &elem_pos, &ext)
                    }
                    TraceShape::CapsuleOrCylinder { dir, radius } => {
                        if radius.is_negative() == false {
                            collision::check::cylinder_capsule(
                                &line,
                                s_radius,
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius,
                            )
                        } else {
                            collision::check::cylinder_cylinder(
                                &LineSegment::from_capsule(elem_pos, dir),
                                radius.neg(),
                                &line,
                                s_radius,
                            )
                        }
                    }
                };

                if matches {
                    visit(node, elem_id, elem);
                }
            }
        };

        self.query_region_with_cutter(&region, visitor, cutter);
    }

    // trace_cylinder?

    // trace_obb =>  ?? Should we? rotation matters ...
}

pub fn create_line_region_cutter<V: Vector>(
    start: V,
    end: V,
    query_margin: <V as Vector>::Num,
) -> impl Fn(&AabbRect<V>, AxisIndex, V::Num) -> [AabbRect<V>; 2] {
    let v_dir = end.sub(&start);
    let dir_norm = v_dir.norm();
    let u_dir = v_dir.amp(dir_norm.inv());

    // Cache thetas for each axis
    let v_mergins = V::from_fn(|i| {
        let unit = V::unit(i);
        let cos_theta = unit.dot(&u_dir);

        // We don't need sign of sin_theta; just calculate abs quickly.
        let sin = (V::Num::ONE - cos_theta.sqr()).sqrt();
        query_margin
            * if sin.is_zero() {
                // Just use a large value to avoid division by zero.
                V::Num::from_int(1000)
            } else {
                sin.inv().abs()
            }
    });

    let base_region = AabbRect::new(start, end).extended_by_all(query_margin);

    move |_, axis, value| {
        // Cut single axis of plane affects the rest when the shape is linear.
        let v_start = start[axis];
        let v_end = end[axis];

        let base = v_end - v_start;

        let t = if base.is_zero() {
            Number::ONE
        } else {
            (value - v_start) / base
        };

        let p_pvt = start.add(&v_dir.amp(t));
        let [p_minus, p_plus] = if v_start <= v_end {
            [start, end]
        } else {
            [end, start]
        };

        // When slicing through a thick plane, the slanted cross-section extends beyond
        // the query margin.
        //
        // Specifically, for the angle theta between the axis-aligned hyperplane and the
        // line segment, the length increases by margin / sin(theta). Hence, we expand the
        // dimension along the axis by the query margin, and the other dimensions by
        // query_margin / sin(theta), and then intersect this expanded area with the base
        // range.

        [AabbRect::new(p_minus, p_pvt), AabbRect::new(p_pvt, p_plus)].map(|mut x| {
            for i in 0..V::D {
                if i == axis {
                    x.extend_axis(axis, query_margin)
                } else {
                    x.extend_axis(i, v_mergins[i])
                }
            }

            x.intersection(&base_region)
        })
    }
}
