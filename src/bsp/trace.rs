use crate::primitive::{AabbRect, AxisIndex, LineSegment, NumExt, Number, Vector, VectorExt as _};

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

        let p_line_min = p_start.min_values(p_end);
        let p_line_max = p_start.max_values(p_end);

        if line.s_norm.is_zero() {
            // treat it as a sphere
            self.trace_sphere(p_start, radius, query_margin, visit);
            return;
        }

        let region = AabbRect::new(*p_start, *p_end).extended_by_all(radius + query_margin);
        let cutter = create_line_region_cutter(*p_start, *p_end, radius + query_margin);

        let visitor = move |node: T::NodeKey| {
            for (elem_id, elem) in self.leaf_iter(node) {
                // Project the direction vector SP between the origin S and the
                // target point P onto the trace line, multiply it by a clamped
                // value between 0 and the norm to find the point H on the line
                // segment SE that is closest to P (i.e., the foot of the
                // perpendicular). Then, check if the distance between P and H
                // falls within the radius.

                let ext = elem.data.extent();
                let s_rad_sum_sqr = match ext {
                    TraceShape::Dot => radius.sqr(),
                    TraceShape::Sphere(elem_rad) => (elem_rad + radius).sqr(),
                    TraceShape::Aabb(ref ext) => {
                        //
                        // Tries to avoid calculating `norm` of the extent as much as
                        // possible ... What we want here is filtering out following
                        // expression:
                        //
                        //      D^2 > (R_aabb + R_capsule)^2
                        //          = R_aabb^2 + 2*R_aabb*R_capsule + R_capsule^2
                        //
                        // However, here, we want to avoid calculating norm of the AABB
                        // extent using sqrt, therefore we what we need here is
                        //
                        //      D^2 > R_aabb^2 + R_capsule^2 + Expr
                        //        where Expr <= 2*R_aabb*R_capsule
                        //
                        // Here we simply replace R_aabb as the longest extent of the of
                        // AABB(l), which is guaranteed to be shorter than the actual
                        // radius, but still gives reasonable approximation and satisfies
                        // the above condition:
                        //
                        //      Expr = 2*l*R_capsule <= 2*R_aabb*R_capsule
                        //
                        let two = <T::Vector as Vector>::Num::from_int(2);
                        let half_ext = ext.amp(two.inv());
                        let r_aabb_sqr = half_ext.norm_sqr();
                        let l = half_ext.max_component();

                        // This is more tolerant approximation of the actual squared
                        // distance. In case of AABB, it's not necessary to be precise
                        // in this step as we're going to double-check it later.
                        r_aabb_sqr + radius.sqr() + l * radius * two
                    }
                };

                if line.distance_from_sqr(&elem.pos) > s_rad_sum_sqr {
                    // Bounding sphere disjoints; early return
                    continue;
                }

                if let TraceShape::Aabb(ext) = ext {
                    // In case of AABB; apply more precise checks.

                    // 0. Since this operation is relatively heavy, it first performs
                    //    preliminary filtering through a radius check. (line-bounding
                    //    sphere distance)
                    // 1. Each hyperplane is defined by a minimum point and a maximum
                    //    point, denoted as P, and the axis direction as n. Therefore, in
                    //    2D there are 4 hyperplanes, in 3D there are 6, and in N
                    //    dimensions, there are N*2 hyperplanes generated.
                    // 2. Calculation of the collision point between each face
                    //    (hyperplane) of the AABB shape and the line segment. (If t
                    //    exceeds 1, it means no collision occurred; otherwise, clamp
                    //    within a reasonable range)
                    // 3. Clamp the collision point to the values excluding the main axis
                    //    on the hyperplane and check if it touches internally. (However,
                    //    if t is out of the normal range, it absolutely does not touch)
                    //    If it does touch, return early due to collision.
                    // 4. If it does not touch internally, calculate the distance from the
                    //    clamped point to the line segment again. Repeat for all
                    //    hyperplanes, and the smallest value becomes the distance.

                    // 0. 이 연산은 상대적으로 무겁기 때문에, 우선 앞의 radius check를 통해
                    //    1차적으로 필터링 수행. (위의 line-distance 체크)
                    // 1. 각 초평면은 최소점, 최대점을 P로 두고, 축 방향을 n으로 한다.
                    //    따라서, 2D에서는 4개, 3D에서는 6개, N에서는 N*2개의 초평면이
                    //    발생.
                    // 2. AABB 도형의 각 면(초평면)과 라인 세그먼트의 충돌점 계산. (t 가 1
                    //    넘어가면; 충돌하지 않았다면 적정 범위에서 clamp)
                    // 3. 충돌점을 초평면에서 주 축을 제외한 값을 clamp, 내부에 접하는지
                    //    확인한다. (단, t가 normal range 벗어낫다면 절대 x) 만약
                    //    내접한다면 충돌로 early return.
                    // 4. 내접하지 않는다면, clamp 완료된 점에서 다시 선분과의 거리를
                    //    계산한다. 전체 초평면에 대해 반복하고, 최소값이 거리가 된다.

                    let half_ext = ext.amp(<T::Vector as Vector>::Num::from_int(2).inv());
                    let v_min = elem.pos.sub(&half_ext);
                    let v_max = elem.pos.add(&half_ext);

                    for v_plane_p in [v_min, v_max] {
                        for i in 0..T::Vector::D {
                            let u_plane_n = T::Vector::unit(i);

                            let dot_n_d = u_plane_n.dot(line.u_d());
                            let mut v_c = if dot_n_d.is_zero() {
                                let mut t = u_plane_n.dot(&v_plane_p.sub(p_start));
                                t = t / dot_n_d;

                                // Here we don't clamp `t` in line segment range; as we're
                                // calculating the point laid on plane.
                                p_start.add(&line.u_d().amp(t))
                            } else {
                                // Any point is allowed.
                                v_plane_p
                            };

                            // The collision point MUST be on the plane.
                            debug_assert!((v_c[i] - v_plane_p[i]).to_f64().abs() < 1e-6);

                            // Clamp collision point in range
                            for k in 0..T::Vector::D {
                                if k != i {
                                    // 1. Clamp to the line segment range first
                                    v_c[k] = v_c[k].clamp(p_line_min[k], p_line_max[k]);

                                    // 2. Then clamp to the plane range
                                    v_c[k] = v_c[k].clamp(v_min[k], v_max[k]);
                                }
                            }

                            // Calculate distance between the line and the collision
                            let dist_sqr = line.distance_from_sqr(&v_c);

                            if dist_sqr <= radius.sqr() {
                                // Any of the hyperplane intersects with the capsule
                                visit(node, elem_id);
                                return;
                            }
                        }
                    }
                } else {
                    // All other cases
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
