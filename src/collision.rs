pub mod intersects {
    use crate::primitive::{AabbRect, LineSegment, NumExt, Number, Vector, VectorExt};

    #[inline]
    pub fn sphere_sphere<V: Vector>(c_1: &V, r_1: V::Num, c_2: &V, r_2: V::Num) -> bool {
        c_1.dist_sqr(c_2) <= (r_1 + r_2).sqr()
    }

    #[inline]
    pub fn aabb_sphere<V: Vector>(rect: &AabbRect<V>, c: &V, r: V::Num) -> bool {
        rect.intersects_sphere(c, r)
    }

    #[inline]
    pub fn capsule_sphere<V: Vector>(
        capsule_line: &LineSegment<V>,
        capsule_r: V::Num,
        c: &V,
        r: V::Num,
    ) -> bool {
        capsule_line.distance_from_sqr(c) < (capsule_r + r).sqr()
    }

    pub fn capsule_center_extent<V: Vector>(
        capsule_line: &LineSegment<V>,
        capsule_r: V::Num,
        center: &V,
        extent: &V,
    ) -> bool {
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
        let two = <V::Num as Number>::from_int(2);
        let half_ext = extent.amp(two.inv());

        {
            let r_aabb_sqr = half_ext.norm_sqr();
            let l = half_ext.max_component();

            // This is more tolerant approximation of the actual squared
            // distance. In case of AABB, it's not necessary to be precise
            // in this step as we're going to double-check it later.
            let approx_dist_sqr = r_aabb_sqr + capsule_r.sqr() + l * capsule_r * two;

            if approx_dist_sqr < capsule_line.distance_from_sqr(center) {
                // Bounding sphere disjoints; early return. This false return always
                // valid; i.e. There's no false negative.
                return false;
            }
        };

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

        let v_min = center.sub(&half_ext);
        let v_max = center.add(&half_ext);
        let line = capsule_line;
        let line_p_end = line.calc_p_end();

        for v_plane_p in [v_min, v_max] {
            for i in 0..V::D {
                let u_plane_n = V::unit(i);

                let dot_n_d = u_plane_n.dot(line.u_d());
                let mut v_c = if dot_n_d.is_zero() {
                    let mut t = u_plane_n.dot(&v_plane_p.sub(&line.p_start));
                    t = t / dot_n_d;

                    // Here we don't clamp `t` in line segment range; as we're
                    // calculating the point laid on plane.
                    line.p_start.add(&line.u_d().amp(t))
                } else {
                    // Any point is allowed.
                    v_plane_p
                };

                // The collision point MUST be on the plane.
                debug_assert!((v_c[i] - v_plane_p[i]).to_f64().abs() < 1e-6);

                // Clamp collision point in range
                for k in 0..V::D {
                    if k != i {
                        // 1. Clamp to the line segment range first
                        v_c[k] = v_c[k].clamp_unordered(line.p_start[k], line_p_end[k]);

                        // 2. Then clamp to the plane range
                        v_c[k] = v_c[k].clamp(v_min[k], v_max[k]);
                    }
                }

                // Calculate distance between the line and the collision
                let dist_sqr = line.distance_from_sqr(&v_c);

                if dist_sqr <= capsule_r.sqr() {
                    // Any of the hyperplane intersects with the capsule
                    return true;
                }
            }
        }

        false
    }

    #[inline]
    pub fn capsule_capsule<V: Vector>(
        c1_line: &LineSegment<V>,
        c1_r: V::Num,
        c2_line: &LineSegment<V>,
        c2_r: V::Num,
    ) -> bool {
        todo!()
    }
}

#[cfg(test)]
mod __tests {
    use super::intersects;

    #[test]
    fn test_sphere_sphere() {
        // Fully intersected
        assert!(intersects::sphere_sphere(
            &[0.0, 0.0],
            1.0,
            &[0.0, 0.0],
            1.0
        ));

        // Partially intersected
        assert!(intersects::sphere_sphere(
            &[0.0, 1.0],
            1.0,
            &[0.0, 0.0],
            1.0
        ));

        // Joint
        assert!(intersects::sphere_sphere(
            &[0.0, 2.0],
            1.0,
            &[0.0, 0.0],
            1.0
        ));

        // Disjoint
        assert!(!intersects::sphere_sphere(
            &[0.0, 3.0],
            1.0,
            &[0.0, 0.0],
            1.0
        ));
    }
}
