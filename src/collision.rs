pub mod check {
    use tap::Tap;

    use crate::{
        collision::distance,
        primitive::{AabbRect, LineSegment, NumExt, Number, PositionalPlane, Vector, VectorExt},
    };

    pub fn sphere_sphere<V: Vector>(c_1: &V, r_1: V::Num, c_2: &V, r_2: V::Num) -> bool {
        c_1.distance_sqr(c_2) <= (r_1 + r_2).sqr()
    }

    pub fn aabb_sphere<V: Vector>(rect: &AabbRect<V>, c: &V, r: V::Num) -> bool {
        rect.intersects_sphere(c, r)
    }

    pub fn aabb_aabb<V: Vector>(rect_1: &AabbRect<V>, rect_2: &AabbRect<V>) -> bool {
        rect_1.intersects(rect_2)
    }

    pub fn capsule_sphere<V: Vector>(
        capsule_line: &LineSegment<V>,
        capsule_r: V::Num,
        c: &V,
        r: V::Num,
    ) -> bool {
        capsule_line.dist_point_sqr(c) < (capsule_r + r).sqr()
    }

    pub fn capsule_aabb_ce<V: Vector>(
        capsule_line: &LineSegment<V>,
        capsule_r: V::Num,
        center: &V,
        extent: &V,
    ) -> bool {
        match check_capsule_aabb_conservative(capsule_line, capsule_r, center, extent) {
            Ok(early_return) => early_return,
            Err(aabb) => distance::visit_line_aabb_sqr(capsule_line, &aabb, |point, _| {
                let dist_sqr = capsule_line.dist_point_sqr(&point);
                (dist_sqr <= capsule_r.sqr()).then_some(())
            })
            .is_some(),
        }
    }

    fn check_capsule_aabb_conservative<V: Vector>(
        capsule_line: &LineSegment<V>,
        capsule_r: V::Num,
        center: &V,
        extent: &V,
    ) -> Result<bool, AabbRect<V>> {
        debug_assert!(extent.min_component() >= V::Num::ZERO);

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
        //        where Expr >= 2*R_aabb*R_capsule, to prevent false negative.
        //
        let two = <V::Num as Number>::from_int(2);
        let half_ext = extent.amp(two.inv());

        {
            let r_aabb_sqr = half_ext.norm_sqr();

            // The maximum component `l` is always shorter than actual AABB radius,
            // however, the radius never gets longer than `l*sqrt(2)` of longest
            // component.
            //
            // This heuristic becomes most effective when the rectangle becomes
            // hyper-cubic, as the radius likely to be same with the longest components'
            // sqrt(2) multiplicand.
            let sqrt_2 = V::Num::from_f64(1.41421356238);
            let l = half_ext.max_component() * sqrt_2;

            // This is more tolerant approximation of the actual squared
            // distance. In case of AABB, it's not necessary to be precise
            // in this step as we're going to double-check it later.
            let expr = two * l * capsule_r;

            let approx_thres_sqr = r_aabb_sqr + capsule_r.sqr() + expr;
            let distance_sqr = capsule_line.dist_point_sqr(center);

            // D^2 > R_aabb^2 + R_capsule^2 + expr
            if distance_sqr > approx_thres_sqr {
                // Bounding sphere disjoints; early return. This false return always
                // valid; i.e. There's no false negative.
                return Ok(false);
            }
        };

        let v_min = center.sub(&half_ext);
        let v_max = center.add(&half_ext);
        let line = capsule_line;
        let aabb = unsafe { AabbRect::new_unchecked(v_min, v_max) };

        // SAFETY: min, max is valid
        if aabb.contains(&line.p_start) {
            // Since the logic below only catches hyperplane contacts, the full inclusion
            // can't be handled only with following algorithm. Therefore, here we firstly
            // check if both points are within the boundary.
            //
            // If any of the point is inside the box, it can safely be treated as hit.
            return Ok(true);
        }

        // Can't determine the result; need to check further.
        Err(aabb)
    }

    pub fn capsule_capsule<V: Vector>(
        c1_line: &LineSegment<V>,
        c1_r: V::Num,
        c2_line: &LineSegment<V>,
        c2_r: V::Num,
    ) -> bool {
        let [near_1, near_2] = c1_line.nearest_pair(c2_line);
        near_1.distance_sqr(&near_2) <= (c1_r + c2_r).sqr()
    }

    pub fn cylinder_sphere<V: Vector>(
        cy_line: &LineSegment<V>,
        cy_radius: V::Num,
        sph_center: &V,
        sph_radius: V::Num,
    ) -> bool {
        let dist_sqr = cy_line.dist_point_sqr(sph_center);

        if dist_sqr > (cy_radius + sph_radius).sqr() {
            // Farer than maximum allowed distance
            return false;
        }

        check_cylinder_hyperplane_with_sphere(cy_line, sph_center, sph_radius).is_ok()
    }

    struct NoIntersection;

    /// Returns the hyperplane if sphere is outside of any of the cylinder's hyperplane.
    fn check_cylinder_hyperplane_with_sphere<V: Vector>(
        cy_line: &LineSegment<V>,
        sph_center: &V,
        sph_radius: V::Num,
    ) -> Result<Option<PositionalPlane<V>>, NoIntersection> {
        // Check two hyperplanes of the cylinder, compare it with given radius.

        // hp bottom,
        let hp1 = PositionalPlane::from_line(cy_line);
        let hp2 = { hp1 }.tap_mut(|x| x.p = cy_line.calc_p_end());

        // Distance 1 is negated since its normal is heading to inside of the cylinder.
        let s_dist_1 = hp1.signed_distance(sph_center).neg();
        let s_dist_2 = hp2.signed_distance(sph_center);

        // Both distance are negative => it's just inside.
        if s_dist_1 <= sph_radius && s_dist_2 <= sph_radius {
            if s_dist_1.is_positive() {
                // `hp1` normal is heading to inside of the cylinder.
                Ok(Some(hp1.flipped()))
            } else if s_dist_2.is_positive() {
                Ok(Some(hp2))
            } else {
                // It's just between the hyperplanes.
                Ok(None)
            }
        } else {
            Err(NoIntersection)
        }
    }

    pub fn cylinder_capsule<V: Vector>(
        cy_line: &LineSegment<V>,
        cy_radius: V::Num,
        cap_line: &LineSegment<V>,
        cap_radius: V::Num,
    ) -> bool {
        // Check capsule first
        let [v_near_cy, v_near_cap] = cy_line.nearest_pair(cap_line);

        if v_near_cy.distance_sqr(&v_near_cap) > (cy_radius + cap_radius).sqr() {
            return false;
        }

        // Then check cylinder
        check_cylinder_hyperplane_with_sphere(cy_line, &v_near_cap, cap_radius).is_ok()
    }

    pub fn cylinder_cylinder<V: Vector>(
        cy1_line: &LineSegment<V>,
        cy1_radius: V::Num,
        cy2_line: &LineSegment<V>,
        cy2_radius: V::Num,
    ) -> bool {
        // Check capsule first
        let [v_near_cy1, v_near_cy2] = cy1_line.nearest_pair(cy2_line);

        if v_near_cy1.distance_sqr(&v_near_cy2) > (cy1_radius + cy2_radius).sqr() {
            return false;
        }

        // FIXME: Suboptimal + inaccurate implementation.

        check_cylinder_hyperplane_with_sphere(cy1_line, &v_near_cy2, cy2_radius)
            .and_then(|_| check_cylinder_hyperplane_with_sphere(cy2_line, &v_near_cy1, cy1_radius))
            .is_ok()
    }

    pub fn cylinder_aabb_ce<V: Vector>(
        _cy_line: &LineSegment<V>,
        _cy_radius: V::Num,
        _center: &V,
        _extent: &V,
    ) -> bool {
        // TODO: Implement this!
        false
    }
}

pub mod distance {
    use crate::primitive::{
        AabbRect, AxisIndex, Hyperplane, LineSegment, NumExt as _, Number, Vector, VectorExt as _,
    };

    pub fn line_aabb_nearest<V: Vector>(
        line: &LineSegment<V>,
        aabb: &AabbRect<V>,
    ) -> (V, (bool, AxisIndex), V::Num) {
        let mut min_dist_sqr = V::Num::MAXIMUM;
        let mut info = (V::zero(), Default::default());

        visit_line_aabb_sqr(line, aabb, |point, sign_index| {
            let dist_sqr = line.dist_point_sqr(&point);

            if dist_sqr < min_dist_sqr {
                min_dist_sqr = dist_sqr;
                info = (point, sign_index);
            }

            None::<()>
        });

        (info.0, info.1, min_dist_sqr)
    }

    pub fn visit_line_aabb_sqr<V: Vector, R>(
        line: &LineSegment<V>,
        aabb: &AabbRect<V>,
        mut visit_with_plane_dist_sqr: impl FnMut(V, (bool, AxisIndex)) -> Option<R>,
    ) -> Option<R> {
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

        let v_min = aabb.min();
        let v_max = aabb.max();
        let line_p_end = line.calc_p_end();

        for (sign, v_plane_p) in [(false, v_min), (true, v_max)] {
            for i in 0..V::D {
                // SAFETY: V::unit(i) is always valid normal vector.
                let plane = unsafe { Hyperplane::new_unchecked(V::unit(i), v_plane_p[i]) };

                // We're looking for the nearest point on the plane from the line. For
                // this, the contact point will be clamped within the plane and the line
                // range. Therefore, even when line and plane is parallel, it's just okay
                // to returning any point of the line.
                let mut v_c = plane.contact_point(line).unwrap_or(line.p_start);

                // Clamp collision point in range.
                for k in 0..V::D {
                    if k != i {
                        // 1. Clamp to the line segment range first
                        v_c[k] = v_c[k].clamp_unordered(line.p_start[k], line_p_end[k]);

                        // 2. Then clamp to the plane range
                        v_c[k] = v_c[k].clamp(v_min[k], v_max[k]);
                    } else {
                        // Force the collision point to be on the plane. Usually it's satisfied
                        // without any additional operation, however, the above `dot_n_d` can be
                        // zero which makes the `t` to be NaN.
                        v_c[k] = v_plane_p[i];
                    }
                }

                // Implements early return. Condition may optimize away when inlined.
                if let Some(x) = visit_with_plane_dist_sqr(v_c, (sign, i)) {
                    return Some(x);
                }
            }
        }

        None
    }
}

#[cfg(test)]
mod __tests {
    use super::check;

    #[test]
    fn test_sphere_sphere() {
        // Fully intersected
        assert!(check::sphere_sphere(&[0.0, 0.0], 1.0, &[0.0, 0.0], 1.0));

        // Partially intersected
        assert!(check::sphere_sphere(&[0.0, 1.0], 1.0, &[0.0, 0.0], 1.0));

        // Joint
        assert!(check::sphere_sphere(&[0.0, 2.0], 1.0, &[0.0, 0.0], 1.0));

        // Disjoint
        assert!(!check::sphere_sphere(&[0.0, 3.0], 1.0, &[0.0, 0.0], 1.0));
    }
}
