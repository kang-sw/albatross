use crate::{
    collision,
    primitive::{AabbRect, AxisIndex, LineSegment, NumExt, Number, Vector, VectorExt as _},
};

use super::{Element, TraceShape, Tree};

impl<T: Element> Tree<T> {
    pub fn trace_capsule(
        &self,
        p_start: &T::Vector,
        p_end: &T::Vector,
        radius: <T::Vector as Vector>::Num,
        query_margin: <T::Vector as Vector>::Num,
        mut visit: impl FnMut(T::NodeKey, T::ElemKey),
    ) {
        let line = LineSegment::new(*p_start, *p_end);

        if line.s_norm.is_zero() {
            // treat it as a sphere
            self.trace_sphere(p_start, radius, query_margin, visit);
            return;
        }

        let region = AabbRect::new(*p_start, *p_end).extended_by_all(radius + query_margin);
        let cutter = create_line_region_cutter(*p_start, *p_end, radius + query_margin);

        let visitor = move |node: T::NodeKey| {
            for (elem_id, elem) in self.leaf_iter(node) {
                let matches = match elem.data.extent() {
                    TraceShape::Sphere(radius) => {
                        collision::check::capsule_sphere(&line, radius, &elem.pos, radius)
                    }
                    TraceShape::Aabb(ext) => {
                        collision::check::capsule_center_extent(&line, radius, &elem.pos, &ext)
                    }
                    TraceShape::Capsule { dir, radius } => {
                        let elem_line = LineSegment::from_capsule(*p_start, dir);
                        collision::check::capsule_capsule(&line, radius, &elem_line, radius)
                    }
                };

                if matches {
                    visit(node, elem_id);
                }
            }
        };

        self.query_region_with_cutter(&region, visitor, cutter);
    }

    pub fn trace_sphere(
        &self,
        center: &T::Vector,
        radius: <T::Vector as Vector>::Num,
        query_margin: <T::Vector as Vector>::Num,
        mut visit: impl FnMut(T::NodeKey, T::ElemKey),
    ) {
        let region = AabbRect::new_circular(*center, radius + query_margin);
        self.query_region(&region.extended_by_all(query_margin), |node| {
            for (elem_id, elem) in self.leaf_iter(node) {
                let matches = match elem.data.extent() {
                    TraceShape::Sphere(elem_rad) => {
                        collision::check::sphere_sphere(center, radius, &elem.pos, elem_rad)
                    }
                    TraceShape::Aabb(ext) => {
                        let aabb = AabbRect::new_extent(elem.pos, ext);
                        collision::check::aabb_sphere(&aabb, center, radius)
                    }
                    TraceShape::Capsule { dir: line, radius } => {
                        let elem_line = LineSegment::from_capsule(elem.pos, line);
                        collision::check::capsule_sphere(&elem_line, radius, center, radius)
                    }
                };

                if matches {
                    visit(node, elem_id);
                }
            }
        });
    }

    pub fn trace_aabb(
        &self,
        region: &AabbRect<T::Vector>,
        query_margin: <T::Vector as Vector>::Num,
        mut visit: impl FnMut(T::NodeKey, T::ElemKey),
    ) {
        self.query_region(&region.extended_by_all(query_margin), move |node| {
            for (elem_id, elem) in self.leaf_iter(node) {
                let matches = match elem.data.extent() {
                    TraceShape::Sphere(rad) => {
                        collision::check::aabb_sphere(region, &elem.pos, rad)
                    }
                    TraceShape::Aabb(ext) => {
                        let aabb = AabbRect::new_extent(elem.pos, ext);
                        collision::check::aabb_aabb(region, &aabb)
                    }
                    TraceShape::Capsule { dir: line, radius } => {
                        let elem_line = LineSegment::from_capsule(elem.pos, line);
                        collision::check::capsule_center_extent(
                            &elem_line,
                            radius,
                            &region.center(),
                            &region.extent(),
                        )
                    }
                };

                if matches {
                    visit(node, elem_id);
                }
            }
        });
    }
}

pub fn create_line_region_cutter<V: Vector>(
    start: V,
    end: V,
    query_margin: <V as Vector>::Num,
) -> impl Fn(&AabbRect<V>, AxisIndex, V::Num) -> [AabbRect<V>; 2] {
    let dir = end.sub(&start);

    move |rect, axis, value| {
        // TODO: Cut single axis of plane affects the rest when the shape is linear.
        let v_start = start[axis];
        let v_end = end[axis];

        let base = v_end - v_start;

        let t = if base.is_zero() {
            Number::ONE
        } else {
            (value - v_start) / (v_end - v_start)
        };

        let p_t = start.add(&dir.amp(t));
        let [p_minus, p_plus] = if v_start <= v_end {
            [start, end]
        } else {
            [end, start]
        };

        [
            AabbRect::new(p_minus, p_t),
            AabbRect::new(p_t, p_plus).extended_by_all(query_margin),
        ]
        .map(|x| x.extended_by_all(query_margin).intersection(rect))
    }
}
