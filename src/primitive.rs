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
    const MINVALUE: Self;
    const MAXVALUE: Self;
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

    fn clamp_unordered(self, a: Self, b: Self) -> Self {
        let [min, max] = if a < b { [a, b] } else { [b, a] };
        self.min_value(min).max_value(max)
    }

    fn sqr(self) -> Self {
        self * self
    }

    fn is_zero(self) -> bool {
        self == Self::ZERO
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

    fn minimum() -> Self {
        Self::from_fn(|_| Self::Num::MINVALUE)
    }

    fn maximum() -> Self {
        Self::from_fn(|_| Self::Num::MAXVALUE)
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

    fn mul(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i] * other[i])
    }

    fn amp(&self, value: Self::Num) -> Self {
        Self::from_fn(|i| self[i] * value)
    }

    fn div(&self, other: &Self) -> Self {
        Self::from_fn(|i| self[i] / other[i])
    }

    fn norm(&self) -> Self::Num {
        self.norm_sqr().sqrt()
    }

    fn norm_sqr(&self) -> Self::Num {
        self.dot(self)
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
                const MINVALUE: Self = Self::MIN;
                const MAXVALUE: Self = Self::MAX;
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
                const MINVALUE: Self = Self::MIN;
                const MAXVALUE: Self = Self::MAX;
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
    pub fn new(mut p1: V, mut p2: V) -> Self {
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
    pub fn new_circular(center: V, radius: V::Num) -> Self {
        assert!(radius >= V::Num::ZERO);

        let mut min = center;
        let mut max = center;

        for i in 0..V::D {
            min[i] = min[i] - radius;
            max[i] = max[i] + radius;
        }

        Self { min, max }
    }

    pub fn new_extent(center: V, extent: V) -> Self {
        let mut min = center;
        let mut max = center;

        for i in 0..V::D {
            let half = extent[i] / V::Num::from_int(2);
            assert!(half >= V::Num::ZERO);

            min[i] = min[i] - half;
            max[i] = max[i] + half;
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
            min[i] = V::Num::MINVALUE;
            max[i] = V::Num::MAXVALUE;
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
        if self.max[axis] == V::Num::MAXVALUE || self.min[axis] == V::Num::MINVALUE {
            V::Num::MAXVALUE
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
            u_d.amp(s_norm.inv())
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
            p_start: p_start.sub(&capsule.u_dir.amp(capsule.s_len / V::Num::from_int(2))),
            s_norm: capsule.s_len,
            u_d: capsule.u_dir,
        }
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

    pub fn dist_line_sqr(&self, other: &Self) -> V::Num {
        let [a, b] = self.nearest_pair(other);
        a.distance_sqr(&b)
    }

    pub fn nearest(&self, p_dst: &V) -> V {
        let l = self;

        let v_to_dst = p_dst.sub(&l.p_start);
        let s_perpend_len = l.u_d.dot(&v_to_dst).clamp(V::Num::ZERO, l.s_norm);

        l.p_start.add(&l.u_d.amp(s_perpend_len))
    }

    /// Find closest point pair between lines
    pub fn nearest_pair(&self, other: &Self) -> [V; 2] {
        let p_l2e = other.calc_p_end();

        let Self { p_start: p_l1s, .. } = self;
        let Self {
            p_start: p_l2s,
            s_norm: s_l2_len,
            u_d: u_l2,
        } = other;

        // @see https://zalo.github.io/blog/closest-point-between-segments/
        let l1 = self;

        // Project other line's start/end points upon this line's plane
        let p_proj_l2s = l1.proj_as_plane(p_l2s);
        let p_proj_l2e = l1.proj_as_plane(&p_l2e);

        // From projected line, calculate t of l1
        let v_proj = p_proj_l2e.sub(&p_proj_l2s);
        let s_proj_sqr = v_proj.norm_sqr();
        let t_l2 = if s_proj_sqr.is_zero() {
            V::Num::ZERO
        } else {
            (p_l1s.sub(&p_proj_l2s).dot(&v_proj) / s_proj_sqr).clamp(Number::ZERO, *s_l2_len)
        };

        let p_l2_nearest = p_l2s.add(&u_l2.amp(*s_l2_len * t_l2));
        let p_l1_nearest = l1.nearest(&p_l2_nearest);

        [p_l1_nearest, p_l2_nearest]
    }

    /// Interpreting the line segment as plane, project the given point onto the plane.
    pub fn proj_as_plane(&self, p: &V) -> V {
        let v = p.sub(&self.p_start);
        p.sub(&self.u_d.amp(self.u_d.dot(&v)))
    }

    pub fn u_d(&self) -> &V {
        &self.u_d
    }

    pub fn set_end(&mut self, p_end: V) {
        self.u_d = p_end.sub(&self.p_start);
        self.s_norm = self.u_d.norm();
        self.u_d = self.u_d.amp(self.s_norm.inv());
    }

    pub fn calc_p_end(&self) -> V {
        self.p_start.add(&self.u_d.amp(self.s_norm))
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

impl<V: Vector> DirectionSegment<V> {
    pub fn new(v_dir: V) -> Self {
        let s_norm = v_dir.norm();
        let u_dir = if s_norm.is_zero() {
            V::unit(0)
        } else {
            v_dir.amp(s_norm.inv())
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
