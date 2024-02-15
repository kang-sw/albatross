/* ---------------------------------------------------------------------------------------------- */
/*                                             TRAITS                                             */
/* ---------------------------------------------------------------------------------------------- */

use std::ops::{Add, Div, Index, IndexMut, Mul, Sub};

macro_rules! trait_alias {
	($vis:vis trait $name:ident {}, $($args:tt)*) => {
		$vis trait $name: $($args)+ {}
		impl<T> $name for T where T: $($args)+ {}
	};
}
trait_alias!(
    pub trait Number {},
    Copy
        + PartialOrd
        + Add<Output = Self>
        + Mul<Output = Self>
        + Sub<Output = Self>
        + Div<Output = Self>
        + NumberCommon
);

pub trait NumberCommon {
    const MINVALUE: Self;
    const MAXVALUE: Self;

    fn to_f64(&self) -> f64;
    fn from_f64(value: f64) -> Self;

    fn one() -> Self;
    fn zero() -> Self;
}

pub trait Vector:
    Clone + Copy + Sized + Index<usize, Output = Self::Num> + IndexMut<usize, Output = Self::Num>
{
    type Num: Number;
    const D: AxisIndex;

    fn zero() -> Self;
    fn zero_f64() -> impl Vector<Num = f64>;
    fn get(&self, i: AxisIndex) -> Self::Num;
    fn get_mut(&mut self, i: AxisIndex) -> &mut Self::Num;
    fn set(&mut self, i: AxisIndex, value: Self::Num);
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

    fn clamp_value(self, min: Self, max: Self) -> Self {
        self.min_value(max).max_value(min)
    }

    fn sqr(self) -> Self {
        self * self
    }
}

impl<T: Number> NumExt for T {}

pub trait VectorExt: Vector {
    fn minimum() -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, Self::Num::MINVALUE);
        }
        v
    }

    fn maximum() -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, Self::Num::MAXVALUE);
        }
        v
    }

    fn min_values(&self, other: &Self) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, self.get(i).min_value(other.get(i)));
        }
        v
    }

    fn max_values(&self, other: &Self) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, self.get(i).max_value(other.get(i)));
        }
        v
    }

    fn values(&self, value: Self::Num) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, value);
        }
        v
    }

    fn sub(&self, other: &Self) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, self.get(i) - other.get(i));
        }
        v
    }

    fn add(&self, other: &Self) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, self.get(i) + other.get(i));
        }
        v
    }

    fn mul(&self, other: &Self) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, self.get(i) * other.get(i));
        }
        v
    }

    fn div(&self, other: &Self) -> Self {
        let mut v = Self::zero();
        for i in 0..Self::D {
            v.set(i, self.get(i) / other.get(i));
        }
        v
    }

    fn dot(&self, other: &Self) -> Self::Num {
        let mut sum = Self::Num::zero();
        for i in 0..Self::D {
            sum = sum + self.get(i) * other.get(i);
        }
        sum
    }

    fn length_squared(&self) -> Self::Num {
        self.dot(self)
    }

    fn distance_squared(&self, other: &Self) -> Self::Num {
        (self.sub(other)).length_squared()
    }
}

impl<T: Vector> VectorExt for T {}

/* ------------------------------------------ Defaults ------------------------------------------ */

impl<T: Number, const D: usize> Vector for [T; D] {
    type Num = T;
    const D: AxisIndex = D;

    fn zero() -> Self {
        [T::zero(); D]
    }

    fn zero_f64() -> impl Vector<Num = f64> {
        [0.0; D]
    }

    fn get(&self, i: AxisIndex) -> Self::Num {
        self[i]
    }

    fn get_mut(&mut self, i: AxisIndex) -> &mut Self::Num {
        &mut self[i]
    }

    fn set(&mut self, i: AxisIndex, value: Self::Num) {
        self[i] = value;
    }
}

#[doc(hidden)]
mod _impl_fixed {
    use super::NumberCommon;
    use fixed::*;

    macro_rules! define_minmax {
    ($($ty:ty), *) => {
        $(impl NumberCommon for $ty {
            const MINVALUE: Self = Self::MIN;
            const MAXVALUE: Self = Self::MAX;

            fn to_f64(&self) -> f64 {
                *self as f64
            }

            fn from_f64(value: f64) -> Self {
                value as Self
            }

            fn one() -> Self {
                1 as _
            }

            fn zero() -> Self {
                0 as _
            }
        })*
    };
}

    define_minmax!(i8, i16, i32, i64, i128, isize, u8, u16, u32, u64, u128, usize, f32, f64);

    macro_rules! define_minmax_fixed {
        ($ty:ident <$t:ident>, $tr:ident) => {
            impl<$t: fixed::types::extra::$tr> NumberCommon for $ty<$t> {
                const MINVALUE: Self = Self::MIN;
                const MAXVALUE: Self = Self::MAX;

                fn to_f64(&self) -> f64 {
                    (*self).to_num()
                }

                fn from_f64(value: f64) -> Self {
                    Self::from_num(value)
                }

                fn one() -> Self {
                    Self::from_num(1)
                }

                fn zero() -> Self {
                    Self::from_num(0)
                }
            }
        };
    }
    define_minmax_fixed!(FixedI8<T>, LeEqU8);
    define_minmax_fixed!(FixedU8<T>, LeEqU8);
    define_minmax_fixed!(FixedI16<T>, LeEqU16);
    define_minmax_fixed!(FixedU16<T>, LeEqU16);
    define_minmax_fixed!(FixedI32<T>, LeEqU32);
    define_minmax_fixed!(FixedU32<T>, LeEqU32);
    define_minmax_fixed!(FixedI64<T>, LeEqU64);
    define_minmax_fixed!(FixedU64<T>, LeEqU64);
    define_minmax_fixed!(FixedI128<T>, LeEqU128);
    define_minmax_fixed!(FixedU128<T>, LeEqU128);
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
        assert!(radius >= V::Num::zero());

        let mut min = center;
        let mut max = center;

        for i in 0..V::D {
            min[i] = min[i] - radius;
            max[i] = max[i] + radius;
        }

        Self { min, max }
    }

    pub fn new_rectangular(center: V, extent: V) -> Self {
        let mut min = center;
        let mut max = center;

        for i in 0..V::D {
            let half = extent[i] / (V::Num::one() + V::Num::one());
            assert!(half >= V::Num::zero());

            min[i] = min[i] - half;
            max[i] = max[i] + half;
        }

        Self { min, max }
    }

    pub fn intersects_sphere(&self, center: &V, radius: V::Num) -> bool {
        let mut nearest = *center;

        for i in 0..V::D {
            nearest[i] = nearest[i].clamp_value(self.min[i], self.max[i]);
        }

        center.distance_squared(&nearest) <= radius.sqr()
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

    pub fn area(&self) -> V::Num {
        let mut area = V::Num::one();
        for i in 0..V::D {
            area = area * (self.max[i] - self.min[i]);
        }
        area
    }

    pub fn length(&self, axis: AxisIndex) -> V::Num {
        if self.max[axis] == V::Num::MAXVALUE || self.min[axis] == V::Num::MINVALUE {
            V::Num::MAXVALUE
        } else {
            debug_assert!(self.max[axis] - self.min[axis] >= V::Num::zero());
            self.max[axis] - self.min[axis]
        }
    }

    pub fn center(&self) -> V {
        let mut center = V::zero();
        for i in 0..V::D {
            center.set(
                i,
                (self.min[i] + self.max[i]) / (V::Num::one() + V::Num::one()),
            );
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

    pub fn intersect(&mut self, other: &Self) {
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

    pub fn split_minus(&mut self, axis: AxisIndex, value: V::Num) {
        self.max[axis] = self.min[axis].max_value(value);
    }

    pub fn split_plus(&mut self, axis: AxisIndex, value: V::Num) {
        self.min[axis] = self.max[axis].min_value(value);
    }
}
