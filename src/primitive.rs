/* ---------------------------------------------------------------------------------------------- */
/*                                             TRAITS                                             */
/* ---------------------------------------------------------------------------------------------- */

use std::ops::{Add, Div, Index, IndexMut, Mul, Sub};

use tap::Tap;

pub trait Number:
    'static
    + Sized
    + Copy
    + PartialOrd
    + Add<Output = Self>
    + Mul<Output = Self>
    + Sub<Output = Self>
    + Div<Output = Self>
    + std::fmt::Debug
{
    const MINIMUM: Self;
    const MAXIMUM: Self;
    const ONE: Self;
    const ZERO: Self;

    fn to_f64(&self) -> f64;
    fn from_f64(value: f64) -> Self;
    fn from_int(value: i64) -> Self;

    fn sqrt(self) -> Self {
        unimplemented!()
    }

    fn rsqrt(self) -> Self {
        self.sqrt().inv()
    }

    fn acos(self) -> Self {
        unimplemented!()
    }

    fn sin(self) -> Self {
        unimplemented!()
    }
}

pub trait Vector:
    Clone
    + Copy
    + Sized
    + Index<usize, Output = Self::Num>
    + IndexMut<usize, Output = Self::Num>
    + std::fmt::Debug
{
    type Num: Number;
    const D: AxisIndex;

    fn zero() -> Self;

    /// XXX:
    fn zero_f64() -> impl Vector<Num = f64>;
}

pub type AxisIndex = usize;

/* -------------------------------------------- Exts -------------------------------------------- */

pub trait NumExt: Number {
    fn min_value(self, other: Self) -> Self {
        if self < other {
            self
        } else {
            other
        }
    }

    fn max_value(self, other: Self) -> Self {
        if self > other {
            self
        } else {
            other
        }
    }

    fn neg(self) -> Self {
        Self::ZERO - self
    }

    fn abs(self) -> Self {
        if self < Self::ZERO {
            self.neg()
        } else {
            self
        }
    }

    fn clamp(self, min: Self, max: Self) -> Self {
        self.min_value(max).max_value(min)
    }

    fn clamp_0_1(self) -> Self {
        self.clamp(Self::ZERO, Self::ONE)
    }

    fn clamp_unordered(self, a: Self, b: Self) -> Self {
        let [min, max] = if a < b { [a, b] } else { [b, a] };
        self.clamp(min, max)
    }

    fn sqr(self) -> Self {
        self * self
    }

    fn is_zero(self) -> bool {
        self == Self::ZERO
    }

    fn is_negative(self) -> bool {
        self < Self::ZERO
    }

    fn is_positive(self) -> bool {
        self > Self::ZERO
    }

    fn inv(self) -> Self {
        Self::ONE / self
    }
}

impl<T: Number> NumExt for T {}

pub trait VectorExt: Vector {
    fn from_fn(set: impl Fn(AxisIndex) -> Self::Num) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v[i] = set(i);
        }
        v
    }

    fn neg(&self) -> Self {
        Self::from_fn(|i| self[i].neg())
    }

    fn minimum() -> Self {
        Self::from_fn(|_| Self::Num::MINIMUM)
    }

    fn maximum() -> Self {
        Self::from_fn(|_| Self::Num::MAXIMUM)
    }

    fn max_component(&self) -> Self::Num {
        let mut max = self[0];
        for i in 1..Self::D {
            max = max.max_value(self[i]);
        }
        max
    }

    fn min_component(&self) -> Self::Num {
        let mut min = self[0];
        for i in 1..Self::D {
            min = min.min_value(self[i]);
        }
        min
    }

    fn unit(axis: AxisIndex) -> Self {
        let mut v = Self::zero();
        v[axis] = Self::Num::ONE;
        v
    }

    fn min_values(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i].min_value(other[i]))
    }

    fn max_values(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i].max_value(other[i]))
    }

    fn splat(&self, value: Self::Num) -> Self {
        Self::from_fn(|_| value)
    }

    fn sub(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i] - other[i])
    }

    fn add(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i] + other[i])
    }

    fn elem_mul(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i] * other[i])
    }

    fn elem_div(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i] / other[i])
    }

    fn amp(&self, value: Self::Num) -> Self {
        Self::from_fn(|i| self[i] * value)
    }

    fn div(&self, value: Self::Num) -> Self {
        Self::from_fn(|i| self[i] / value)
    }

    fn norm(&self) -> Self::Num {
        self.norm_sqr().sqrt()
    }

    fn norm_sqr(&self) -> Self::Num {
        self.dot(self)
    }

    /// Only meaningful when `self` is normal
    fn proj(&self, other: &Self) -> Self {
        self.amp(self.dot(other))
    }

    fn sum(&self) -> Self::Num {
        let mut sum = Self::Num::ZERO;
        for i in 0..Self::D {
            sum = sum + self[i];
        }
        sum
    }

    fn distance(&self, other: &Self) -> Self::Num {
        self.distance_sqr(other).sqrt()
    }

    fn distance_sqr(&self, other: &Self) -> Self::Num {
        (self.sub(other)).norm_sqr()
    }

    fn dot(&self, other: &Self) -> Self::Num {
        let mut sum = Self::Num::ZERO;
        for i in 0..Self::D {
            sum = sum + self[i] * other[i];
        }
        sum
    }
}

impl<T: Vector> VectorExt for T {}

/* ------------------------------------------ Defaults ------------------------------------------ */

impl<T: Number, const D: usize> Vector for [T; D] {
    type Num = T;
    const D: AxisIndex = D;

    fn zero() -> Self {
        [T::ZERO; D]
    }

    fn zero_f64() -> impl Vector<Num = f64> {
        [0.0; D]
    }
}

#[cfg(feature = "nalgebra")]
impl<T: Number, const D: usize> Vector for nalgebra::SVector<T, D> {
    type Num = T;
    const D: AxisIndex = D;

    fn zero() -> Self {
        [T::ZERO; D].into()
    }

    fn zero_f64() -> impl Vector<Num = f64> {
        [0.0; D]
    }
}

#[doc(hidden)]
mod _impl_fixed {
    use super::Number;

    #[cfg(feature = "fixed")]
    use fixed::*;

    macro_rules! define_minmax {
        /* --------------------------------------- Basics --------------------------------------- */

        (i, $($ty:ty), *) => {
            $(define_minmax!(base, $ty);)*
        };

        (u, $($ty:ty), *) => {
            $(define_minmax!(base, $ty);)*
        };

        (f, $($ty:ty), *) => {
            $(define_minmax!(base, $ty, def_float, def_rsqrt);)*
        };

        /* -------------------------------------- Utilities ------------------------------------- */

        (base, $ty:ty $(,$args:tt)*) => {
            impl Number for $ty {
                const MINIMUM: Self = Self::MIN;
                const MAXIMUM: Self = Self::MAX;
                const ONE: Self = 1 as _;
                const ZERO: Self = 0 as _;

                fn to_f64(&self) -> f64 {
                    *self as f64
                }

                fn from_f64(value: f64) -> Self {
                    value as Self
                }

                fn from_int(value: i64) -> Self {
                    value as Self
                }

                $(define_minmax!($args);)*
            }
        };

        (fx, $ty:ident <$t:ident>, $tr:ident $(, $arg:tt)*) => {
            #[cfg(feature = "fixed")]
            impl<$t: fixed::types::extra::$tr> Number for $ty<$t> {
                const MINIMUM: Self = Self::MIN;
                const MAXIMUM: Self = Self::MAX;
                const ONE: Self = Self::const_from_int(1);
                const ZERO: Self = Self::const_from_int(0);

                fn to_f64(&self) -> f64 {
                    (*self).to_num()
                }

                fn from_f64(value: f64) -> Self {
                    Self::from_num(value)
                }

                fn from_int(value: i64) -> Self {
                    Self::const_from_int(value as _)
                }

                define_minmax!($($arg)*);
            }
        };

        (def_float) => {
            fn sqrt(self) -> Self {
                self.sqrt()
            }
            fn acos(self) -> Self {
                self.acos()
            }
            fn sin(self) -> Self {
                self.sin()
            }
        };

        (def_rsqrt) => {
            fn rsqrt(self) -> Self {
                // TODO: Find more efficient way to calculate rsqrt
                1. / self.sqrt()
            }
        }
    }

    define_minmax!(i, i8, i16, i32, i64, i128, isize);
    define_minmax!(u, u8, u16, u32, u64, u128, usize);
    define_minmax!(f, f32, f64);

    define_minmax!(fx, FixedI8<T>, LeEqU8);
    define_minmax!(fx, FixedU8<T>, LeEqU8);
    define_minmax!(fx, FixedI16<T>, LeEqU16);
    define_minmax!(fx, FixedU16<T>, LeEqU16);
    define_minmax!(fx, FixedI32<T>, LeEqU32);
    define_minmax!(fx, FixedU32<T>, LeEqU32);
    define_minmax!(fx, FixedI64<T>, LeEqU64);
    define_minmax!(fx, FixedU64<T>, LeEqU64);
    define_minmax!(fx, FixedI128<T>, LeEqU128);
    define_minmax!(fx, FixedU128<T>, LeEqU128);
}

/* ---------------------------------------------------------------------------------------------- */
/*                                         AABB RECTANGLE                                         */
/* ---------------------------------------------------------------------------------------------- */

#[derive(Clone, Copy, Default, Debug)]
pub struct AabbRect<V: Vector> {
    min: V,
    max: V,
}

impl<V: Vector> AabbRect<V> {
    /// Creates a new `AabbRect` with the given minimum and maximum vectors.
    ///
    /// The minimum and maximum vectors are adjusted to ensure that the minimum
    /// values are less than or equal to the maximum values in each dimension.
    ///
    /// # Arguments
    ///
    /// * `min` - The minimum vector.
    /// * `max` - The maximum vector.
    ///
    /// # Returns
    ///
    /// A new `AabbRect` with the adjusted minimum and maximum vectors.
    pub fn from_points(mut p1: V, mut p2: V) -> Self {
        for i in 0..V::D {
            let a = &mut p1[i];
            let b = &mut p2[i];

            if a > b {
                std::mem::swap(a, b);
            }
        }

        Self { min: p1, max: p2 }
    }

    /// Build new rectangle from circular components.
    pub fn from_sphere(center: V, radius: V::Num) -> Self {
        assert!(radius >= V::Num::ZERO);

        let mut min = center;
        let mut max = center;

        for i in 0..V::D {
            min[i] = min[i] - radius;
            max[i] = max[i] + radius;
        }

        Self { min, max }
    }

    pub fn from_extent(center: V, extent: V) -> Self {
        Self::from_half_extent(center, extent.div(V::Num::from_int(2)))
    }

    pub fn from_half_extent(center: V, half_extent: V) -> Self {
        let mut min = center;
        let mut max = center;

        for i in 0..V::D {
            assert!(half_extent[i] >= V::Num::ZERO);

            min[i] = min[i] - half_extent[i];
            max[i] = max[i] + half_extent[i];
        }

        Self { min, max }
    }

    pub fn intersects_sphere(&self, center: &V, radius: V::Num) -> bool {
        let mut nearest = *center;

        for i in 0..V::D {
            nearest[i] = nearest[i].clamp(self.min[i], self.max[i]);
        }

        center.distance_sqr(&nearest) <= radius.sqr()
    }

    /// Creates a new `AabbRect` with the given minimum and maximum vectors
    /// without any checks.
    ///
    /// # Safety
    ///
    /// This function is marked as unsafe because it does not perform any checks
    /// to ensure that the minimum values are less than or equal to the maximum
    /// values in each dimension. It is the responsibility of the caller to
    /// ensure that the input vectors are valid.
    ///
    /// # Arguments
    ///
    /// * `min` - The minimum vector.
    /// * `max` - The maximum vector.
    ///
    /// # Returns
    ///
    /// A new `AabbRect` with the given minimum and maximum vectors.
    pub unsafe fn new_unchecked(min: V, max: V) -> Self {
        debug_assert!((0..V::D).all(|i| min[i] <= max[i]));
        Self { min, max }
    }

    pub fn maximum() -> Self {
        // TODO: this is workaround as we can't use expression [..; V::D]
        let mut min = V::zero();
        let mut max = V::zero();

        for i in 0..V::D {
            min[i] = V::Num::MINIMUM;
            max[i] = V::Num::MAXIMUM;
        }

        Self { min, max }
    }

    pub fn min(&self) -> &V {
        &self.min
    }

    pub fn max(&self) -> &V {
        &self.max
    }

    pub fn extent(&self) -> V {
        self.max.sub(&self.min)
    }

    pub fn area(&self) -> V::Num {
        let mut area = V::Num::ONE;
        for i in 0..V::D {
            area = area * (self.max[i] - self.min[i]);
        }
        area
    }

    pub fn length(&self, axis: AxisIndex) -> V::Num {
        if self.max[axis] == V::Num::MAXIMUM || self.min[axis] == V::Num::MINIMUM {
            V::Num::MAXIMUM
        } else {
            debug_assert!(self.max[axis] - self.min[axis] >= V::Num::ZERO);
            self.max[axis] - self.min[axis]
        }
    }

    pub fn center(&self) -> V {
        let mut center = V::zero();
        for i in 0..V::D {
            center[i] = (self.min[i] + self.max[i]) / (V::Num::ONE + V::Num::ONE);
        }
        center
    }

    pub fn contains(&self, point: &V) -> bool {
        for i in 0..V::D {
            if point[i] < self.min[i] || self.max[i] <= point[i] {
                return false;
            }
        }
        true
    }

    pub fn intersects(&self, other: &Self) -> bool {
        for i in 0..V::D {
            if other.max[i] <= self.min[i] || self.max[i] <= other.min[i] {
                return false;
            }
        }
        true
    }

    pub fn is_empty(&self) -> bool {
        for i in 0..V::D {
            if self.min[i] == self.max[i] {
                return true;
            }
        }
        false
    }

    pub fn extended_by_all(&self, value: V::Num) -> Self {
        let mut min = self.min;
        let mut max = self.max;

        for i in 0..V::D {
            min[i] = min[i] - value;
            max[i] = max[i] + value;

            if min[i] > max[i] {
                std::mem::swap(&mut min[i], &mut max[i]);
            }
        }

        Self { min, max }
    }

    pub fn extended_by(&self, extend: &V) -> Self {
        let mut min = self.min;
        let mut max = self.max;

        for i in 0..V::D {
            min[i] = min[i] - extend[i];
            max[i] = max[i] + extend[i];

            if min[i] > max[i] {
                std::mem::swap(&mut min[i], &mut max[i]);
            }
        }

        Self { min, max }
    }

    pub fn extend_axis(&mut self, axis: AxisIndex, value: V::Num) {
        self.min[axis] = self.min[axis] - value;
        self.max[axis] = self.max[axis] + value;
    }

    pub fn move_by(&self, value: V) -> Self {
        let mut min = self.min;
        let mut max = self.max;

        for i in 0..V::D {
            min[i] = min[i] + value[i];
            max[i] = max[i] + value[i];
        }

        Self { min, max }
    }

    pub fn contains_axis(&self, axis: AxisIndex, value: V::Num) -> bool {
        self.min[axis] <= value && value < self.max[axis]
    }

    pub fn apply_intersection(&mut self, other: &Self) {
        for i in 0..V::D {
            self.min[i] = self.min[i].max_value(other.min[i]);
            self.max[i] = self.max[i].min_value(other.max[i]);
        }
    }

    pub fn intersection(&self, other: &Self) -> Self {
        let mut min = self.min;
        let mut max = self.max;

        for i in 0..V::D {
            min[i] = min[i].max_value(other.min[i]);
            max[i] = max[i].min_value(other.max[i]);
        }

        Self { min, max }
    }

    pub fn apply_split_minus(&mut self, axis: AxisIndex, value: V::Num) {
        self.max[axis] = self.min[axis].max_value(value);
    }

    pub fn apply_split_plus(&mut self, axis: AxisIndex, value: V::Num) {
        self.min[axis] = self.max[axis].min_value(value);
    }

    pub fn split_minus(&self, axis: AxisIndex, value: V::Num) -> Self {
        { *self }.tap_mut(|x| x.apply_split_minus(axis, value))
    }

    pub fn split_plus(&self, axis: AxisIndex, value: V::Num) -> Self {
        { *self }.tap_mut(|x| x.apply_split_plus(axis, value))
    }
}

/* ---------------------------------------------------------------------------------------------- */
/*                                           GEOMETRIES                                           */
/* ---------------------------------------------------------------------------------------------- */

pub struct ZeroNorm;

/* ---------------------------------------- Line Segment ---------------------------------------- */

#[derive(Debug, Clone, Copy)]
pub struct LineSegment<V: Vector> {
    pub p_start: V,
    pub s_norm: V::Num,
    u_d: V,
}

impl<V: Vector> LineSegment<V> {
    pub fn new(p_start: V, p_end: V) -> Self {
        let u_d = p_end.sub(&p_start);
        let s_norm = u_d.norm();

        let u_d = if s_norm.is_zero() {
            V::unit(0) // It's just dummy normal
        } else {
            u_d.div(s_norm)
        };

        Self {
            p_start,
            s_norm,
            u_d,
        }
    }

    pub fn from_capsule(p_start: V, capsule: DirectionSegment<V>) -> Self {
        Self {
            p_start,
            s_norm: capsule.s_len,
            u_d: capsule.u_dir,
        }
    }

    pub fn from_capsule_centered(p_start: V, capsule: DirectionSegment<V>) -> Self {
        Self {
            p_start: p_start.sub(&capsule.u_dir.amp(capsule.s_len).div(V::Num::from_int(2))),
            s_norm: capsule.s_len,
            u_d: capsule.u_dir,
        }
    }

    pub fn invert(&mut self) {
        self.u_d = self.u_d.neg();
    }

    /// # Safety
    ///
    /// This function is marked as unsafe because it does not perform any checks
    /// that given normal is valid normal
    pub unsafe fn new_unchecked(p_offset: V, s_norm: V::Num, u_d: V) -> Self {
        // It MUST be a valid normal with length 1
        debug_assert!((u_d.norm_sqr() - V::Num::ONE).to_f64().abs() < 1e-3);

        Self {
            p_start: p_offset,
            s_norm,
            u_d,
        }
    }

    pub fn dist_point_sqr(&self, p_dst: &V) -> V::Num {
        self.nearest(p_dst).distance_sqr(p_dst)
    }

    pub fn nearest(&self, p_dst: &V) -> V {
        let l = self;

        let v_to_dst = p_dst.sub(&l.p_start);
        let s_perpend_len = l.u_d.dot(&v_to_dst).clamp(V::Num::ZERO, l.s_norm);

        l.p_start.add(&l.u_d.amp(s_perpend_len))
    }

    /// Find closest point pair between lines
    pub fn nearest_pair(&self, other: &Self) -> [V; 2] {
        let ab = self;
        let cd = other;

        // @see https://zalo.github.io/blog/closest-point-between-segments/

        // Make hyperplane from CD, and project AB onto CD. It makes CD into a dot; thus
        // nearest point of AB can be found.

        let plane_cd = Hyperplane::from_line(cd);
        let p_pl_a = plane_cd.project_pos(&ab.p_start);
        let p_pl_b = plane_cd.project_pos(&ab.calc_p_end());
        let t = line_find_t(&p_pl_a, &p_pl_b, &cd.p_start);
        let p_ab_to_cd_draft = ab.by_t(t.clamp_0_1());

        let p_cd_to_ab = cd.nearest(&p_ab_to_cd_draft);
        [ab.nearest(&p_cd_to_ab), p_cd_to_ab]
    }

    pub fn u_d(&self) -> &V {
        &self.u_d
    }

    pub fn by_t(&self, t: V::Num) -> V {
        self.by_len(self.s_norm * t)
    }

    pub fn by_len(&self, len: V::Num) -> V {
        self.p_start.add(&self.u_d.amp(len))
    }

    pub fn set_end(&mut self, p_end: V) {
        self.u_d = p_end.sub(&self.p_start);
        self.s_norm = self.u_d.norm();
        self.u_d = self.u_d.div(self.s_norm);
    }

    pub fn calc_p_end(&self) -> V {
        self.by_len(self.s_norm)
    }
}

/// Calculates the `t` value, which is the alpha value to the foot of the perpendicular
/// from `pos` to the line.
pub fn line_find_t<V: Vector>(start: &V, end: &V, pos: &V) -> V::Num {
    let v_se = end.sub(start);
    let base = v_se.norm_sqr();

    if base.is_zero() {
        V::Num::ZERO
    } else {
        pos.sub(start).dot(&v_se) / base
    }
}

/* ------------------------------------------- Capsule ------------------------------------------ */

#[derive(Debug, Clone, Copy)]
pub struct DirectionSegment<V: Vector> {
    u_dir: V,
    pub s_len: V::Num,
}

impl<V: Vector> Default for DirectionSegment<V> {
    fn default() -> Self {
        Self {
            u_dir: V::unit(0),
            s_len: V::Num::ZERO,
        }
    }
}

impl<V: Vector> From<V> for DirectionSegment<V> {
    fn from(v: V) -> Self {
        Self::new(v)
    }
}

impl<V: Vector> DirectionSegment<V> {
    pub fn new(v_dir: V) -> Self {
        let s_norm = v_dir.norm();
        let u_dir = if s_norm.is_zero() {
            V::unit(0)
        } else {
            v_dir.div(s_norm)
        };

        Self {
            u_dir,
            s_len: s_norm,
        }
    }

    pub fn u_dir(&self) -> &V {
        &self.u_dir
    }

    pub fn calc_v_dir(&self) -> V {
        self.u_dir.amp(self.s_len)
    }
}

/* -------------------------------------- Plane: Positional ------------------------------------- */

/// Represent a plane, centered with given position.
#[derive(Debug, Clone, Copy)]
pub struct PositionalPlane<V: Vector> {
    pub p: V,
    n: V,
}

impl<V: Vector> PositionalPlane<V> {
    /// # Safety
    ///
    /// `n` is valid normal.
    pub unsafe fn new_unchecked(p: V, n: V) -> Self {
        Self { p, n }
    }

    pub fn from_line(l: &LineSegment<V>) -> Self {
        let p = l.p_start;
        let n = l.u_d;
        Self { p, n }
    }

    pub fn to_hyperplane(&self) -> Hyperplane<V> {
        Hyperplane {
            d: self.n.dot(&self.p),
            n: self.n,
        }
    }

    pub fn project_pos(&self, pos: &V) -> V {
        let v = pos.sub(&self.p);
        pos.sub(&self.n.proj(&v))
    }

    pub fn project_dir(&self, dir_vec: &V) -> V {
        dir_vec.sub(&self.n.proj(dir_vec))
    }

    pub fn signed_distance(&self, pos: &V) -> V::Num {
        let v = pos.sub(&self.p);
        v.dot(&self.n)
    }

    pub fn flipped(self) -> Self {
        Self {
            p: self.p,
            n: self.n.neg(),
        }
    }

    /// Line:
    ///
    ///     p_L + t * u_L, where 0 <= t <= s, s is line length
    ///
    /// Hyperplane:
    ///
    ///     p_P := Random point on the plane.
    ///     u_P := Plane normal
    ///     p_H := Contact point between plane and the line
    ///
    /// Following equation holds:
    ///
    ///     u_P dot ( p_H - p_P ) = 0
    ///     u_P dot ( p_L + t_0 * u_L - p_P ) = 0 since p_H is point on the line.
    ///        
    /// Solving this equation for `t_0`:
    ///
    ///     t_0 = u_P dot ( p_P - p_L ) / u_P dot u_L
    ///
    /// Therefore, the contact point is:
    ///
    ///     p_H = p_L + t_0 * u_L, where u_P dot u_L != 0
    pub fn contact_point(&self, line: &LineSegment<V>) -> Option<V> {
        let dot_p_l = self.n.dot(&line.u_d);
        if dot_p_l.is_zero() {
            return None;
        }

        let v_l_p = self.p.sub(&line.p_start);
        let t_0 = self.n.dot(&v_l_p) / dot_p_l;

        Some(line.p_start.add(&line.u_d.amp(t_0)))
    }

    pub fn n(&self) -> &V {
        &self.n
    }
}

impl<T: Vector> From<Hyperplane<T>> for PositionalPlane<T> {
    fn from(h: Hyperplane<T>) -> Self {
        let p = h.calc_p();
        let n = h.n;
        Self { p, n }
    }
}

/* -------------------------------------- Plane: Hyperplane ------------------------------------- */

/// Represent a plane with normal and distance from center.
#[derive(Debug, Clone, Copy)]
pub struct Hyperplane<V: Vector> {
    pub d: V::Num,
    n: V,
}

impl<V: Vector> Hyperplane<V> {
    /// # Safety
    ///
    /// `n` is valid normal.
    pub unsafe fn new_unchecked(n: V, d: V::Num) -> Self {
        debug_assert!((n.norm_sqr() - V::Num::ONE).abs() < Number::from_f64(1e-4));

        Self { d, n }
    }

    pub fn from_line(l: &LineSegment<V>) -> Self {
        let d = l.u_d.dot(&l.p_start);
        Self { d, n: l.u_d }
    }

    pub fn to_positional(&self) -> PositionalPlane<V> {
        PositionalPlane::from(*self)
    }

    pub fn project_pos(&self, pos: &V) -> V {
        self.to_positional().project_pos(pos)
    }

    pub fn signed_distance_sqr(&self, pos: &V) -> V::Num {
        self.to_positional().signed_distance(pos)
    }

    pub fn flipped(self) -> Self {
        Self {
            d: self.d.neg(),
            n: self.n.neg(),
        }
    }

    pub fn contact_point(&self, line: &LineSegment<V>) -> Option<V> {
        self.to_positional().contact_point(line)
    }

    pub fn calc_p(&self) -> V {
        self.n.amp(self.d)
    }

    pub fn n(&self) -> &V {
        &self.n
    }
}

impl<T: Vector> From<PositionalPlane<T>> for Hyperplane<T> {
    fn from(p: PositionalPlane<T>) -> Self {
        p.to_hyperplane()
    }
}
