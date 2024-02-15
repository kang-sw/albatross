use crate::primitive::{AabbRect, AxisIndex, NumExt, Number, NumberCommon, Vector, VectorExt as _};

use super::{Element, TraceShape, Tree};

impl<T: Element> Tree<T> {
    pub fn trace_capsule(
        &self,
        start: &T::Vector,
        end: &T::Vector,
        radius: <T::Vector as Vector>::Num,
        query_margin: <T::Vector as Vector>::Num,
        visit: impl FnMut(T::NodeKey, T::ElemKey),
    ) {
        let region = AabbRect::new(*start, *end).extended_by_all(radius + query_margin);

        self.query_region_with_cutter(
            &region,
            |node| {
                for (elem_id, elem) in self.leaf_iter(node) {
                    // TODO: Implement capsule trace

                    elem.data.extent();
                    elem.pos;
                }
            },
            create_line_region_cutter(*start, *end, radius + query_margin),
        );
    }

    pub fn trace_sphere(
        &self,
        center: &T::Vector,
        radius: <T::Vector as Vector>::Num,
        query_margin: <T::Vector as Vector>::Num,
        visit: impl FnMut(T::NodeKey, T::ElemKey),
    ) {
        let region = AabbRect::new_circular(*center, radius + query_margin);
        self.query_region(&region.extended_by_all(query_margin), |node| {
            for (elem_id, elem) in self.leaf_iter(node) {
                todo!()
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
                        let aabb = AabbRect::new_rectangular(elem.pos, ext);
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
    move |rect, axis, value| {
        // TODO: Cut single axis of plane affects the rest when the shape is linear.

        todo!()
    }
}
