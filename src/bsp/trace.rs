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
                    ext @ (TraceShape::Dot | TraceShape::Sphere(_)) => {
                        let radius = match ext {
                            TraceShape::Dot => Number::ZERO,
                            TraceShape::Sphere(r) => r,
                            _ => unreachable!(),
                        };

                        collision::intersects::capsule_sphere(&line, radius, &elem.pos, radius)
                    }
                    TraceShape::Aabb(ext) => {
                        collision::intersects::capsule_center_extent(&line, radius, &elem.pos, &ext)
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
                    TraceShape::Dot => center.dist_sqr(&elem.pos) <= radius * radius,
                    TraceShape::Sphere(rad) => center.dist_sqr(&elem.pos) <= (rad + radius).sqr(),
                    TraceShape::Aabb(ext) => {
                        let aabb = AabbRect::new_extent(elem.pos, ext);
                        aabb.intersects_sphere(center, radius)
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
                    TraceShape::Dot => region.contains(&elem.pos),
                    TraceShape::Sphere(rad) => region.intersects_sphere(&elem.pos, rad),
                    TraceShape::Aabb(ext) => {
                        let aabb = AabbRect::new_extent(elem.pos, ext);
                        region.intersects(&aabb)
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
