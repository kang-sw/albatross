use std::ops::*;

use crate::primitive::Number;

#[cfg(feature = "reflect")]
use bevy_reflect::Reflect;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[cfg(feature = "glam")]
pub use glam_impls::*;

#[cfg(feature = "glam")]
mod glam_impls;

#[doc(hidden)]
pub trait BitContainer:
    Copy
    + Shr<usize, Output = Self>
    + Shl<usize, Output = Self>
    + BitAnd<Output = Self>
    + BitOr<Output = Self>
    + SignedInteger
    + Number
{
}

impl<T> BitContainer for T where
    T: Copy
        + Shr<usize, Output = T>
        + Shl<usize, Output = T>
        + BitAnd<T, Output = T>
        + BitOr<T, Output = T>
        + SignedInteger
        + Number
{
}

#[doc(hidden)]
pub trait SignedInteger {
    const SIGNED: bool = false;

    fn leading_zeros(self) -> u32;
    fn bit_flip(self) -> Self;
    fn clamp_bits(self, bits: usize) -> Self;
    fn to_normal(self, bits: usize) -> f32;
    fn from_normal(normal: f32, bits: usize) -> Self;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                             VECTOR                                             */
/* ---------------------------------------------------------------------------------------------- */

#[derive(Default, Clone, Copy, PartialEq, Eq, Hash)]
#[repr(transparent)]
#[cfg_attr(feature = "reflect", derive(Reflect))]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[cfg_attr(feature = "serde", serde(transparent))]
pub struct PackedIVec3<B, const X: usize, const Y: usize, const Z: usize>(B);

impl<B, const X: usize, const Y: usize, const Z: usize> std::fmt::Debug for PackedIVec3<B, X, Y, Z>
where
    B: BitContainer,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        self.to_arr().fmt(f)
    }
}

impl<B, const X: usize, const Y: usize, const Z: usize> PackedIVec3<B, X, Y, Z>
where
    B: BitContainer,
{
    pub const X_BITS: Range<usize> = 0..X;
    pub const Y_BITS: Range<usize> = X..X + Y;
    pub const Z_BITS: Range<usize> = X + Y..X + Y + Z;

    /// `x`, `y`, `z` MUST not exceed the bit length of `X`, `Y`, `Z` respectively.
    pub fn new(x: B, y: B, z: B) -> Self {
        debug_assert!(X + Y + Z <= std::mem::size_of::<B>() * 8);

        let [x, y, z] = Self::clamped([x, y, z]);
        Self(x | (y << X) | (z << (X + Y)))
    }

    fn clamped([x, y, z]: [B; 3]) -> [B; 3] {
        [
            x.clamp_bits(X) & ((B::ONE << X) - B::ONE),
            y.clamp_bits(Y) & ((B::ONE << Y) - B::ONE),
            z.clamp_bits(Z) & ((B::ONE << Z) - B::ONE),
        ]
    }

    pub fn to_arr(self) -> [B; 3] {
        let result = [
            self.0 & ((B::ONE << X) - B::ONE),
            (self.0 >> X) & ((B::ONE << Y) - B::ONE),
            (self.0 >> (X + Y)) & ((B::ONE << Z) - B::ONE),
        ];

        if B::SIGNED {
            let max_bit_pos = std::mem::size_of::<B>() * 8;
            let dim_bits = [X, Y, Z];

            [0, 1, 2].map(|i| {
                let r = result[i];
                let sign_bit = dim_bits[i];
                let msb = max_bit_pos - r.leading_zeros() as usize;

                if msb == sign_bit {
                    r | ((B::ONE << sign_bit) - B::ONE).bit_flip()
                } else {
                    r
                }
            })
        } else {
            result
        }
    }

    pub fn from_arr([x, y, z]: [B; 3]) -> Self {
        Self::new(x, y, z)
    }

    pub fn to_normals(self) -> [f32; 3] {
        let [x, y, z] = self.to_arr();
        [x.to_normal(X), y.to_normal(Y), z.to_normal(Z)]
    }

    pub fn from_normals([x, y, z]: [f32; 3]) -> Self {
        Self::new(
            B::from_normal(x, X),
            B::from_normal(y, Y),
            B::from_normal(z, Z),
        )
    }
}

macro_rules! signed {

    (true, $($t:ty)*) => {
        $(
            impl SignedInteger for $t {
                signed!(base, true);

                fn clamp_bits(self, bits: usize) -> Self {
                    self.clamp(
                        -(1 << (bits - 1)),
                        (1 << (bits - 1)) - 1
                    )
                }

                fn to_normal(self, bits: usize) -> f32 {
                    2.0 * if self >= 0 {
                        self as f32 / ((1 << bits) - 1) as f32
                    } else {
                        self as f32 / (1 << bits) as f32
                    }
                }

                fn from_normal(mut normal: f32, bits: usize) -> Self {
                    normal *= 0.5;

                    if normal >= 0.0 {
                        (normal * ((1 << bits) - 1) as f32)
                    } else {
                        (normal * (1 << bits) as f32)
                    }.round () as Self
                }

            }
        )*
    };

    (false, $($t:ty)*) => {
        $(
            impl SignedInteger for $t {
                signed!(base, false);

                fn clamp_bits(self, bits: usize) -> Self {
                    self.min((1 << bits) - 1)
                }

                fn to_normal(self, bits: usize) -> f32 {
                    self as f32 / ((1 << bits) - 1) as f32
                }

                fn from_normal(normal: f32, bits: usize) -> Self {
                    (normal * ((1 << bits) - 1) as f32).round() as Self
                }
            }
        )*
    };

    (base, $value:literal) => {
        const SIGNED: bool = $value;

        fn leading_zeros(self) -> u32 {
            self.leading_zeros()
        }

        fn bit_flip(self) -> Self {
            !self
        }
    };
}

signed!(true, i8 i16 i32 i64 );
signed!(false, u8 u16 u32 u64 );

/* ----------------------------------------- Conversion ----------------------------------------- */

macro_rules! from {
    ($component:ty, $dst:ty, $($body:tt)*) => {
        impl<B, const X: usize, const Y: usize, const Z: usize> From<$dst>
            for PackedIVec3<B, X, Y, Z>
        where
            B: BitContainer + From<$component>,
        {
            $($body)*
        }
    };
}

macro_rules! into {
    ($component:ty, $dst:ty, $($body:tt)*) => {
        impl<B, const X: usize, const Y: usize, const Z: usize> From<PackedIVec3<B, X, Y, Z>>
            for $dst
        where
            B: BitContainer + Into<$component>,
        {
            $($body)*
        }
    };
}

macro_rules! arr {
    ($ty:ty, $($rest:tt)*) => {
        from!(
            $ty,
            [$ty; 3],
            fn from(x: [$ty; 3]) -> Self {
                Self::from_arr(x.map(|x| x.into()))
            }
        );

        into!(
            $ty,
            [$ty; 3],
            fn from(v: PackedIVec3<B, X, Y, Z>) -> [$ty; 3] {
                v.to_arr().map(|x| x.into())
            }
        );

        arr!($($rest)*);
    };
    () => {};
}

macro_rules! glamvec {
    ($(($comp:ty, $ty:ty)),*) => {
        $(
            from!($comp, $ty, fn from(v: $ty) -> Self {
                Self::new(v.x.into(), v.y.into(), v.z.into())
            });

            into!($comp, $ty, fn from(v: PackedIVec3<B, X, Y, Z>) -> Self {
                let [x,y,z] = v.to_arr().map(|s| s.into());
                Self::new(x,y,z)
            });
        )*
    };

    () => {}
}

arr!(i8, i16, i32, i64, u8, u16, u32, u64,);

#[cfg(feature = "glam")]
mod __impl_glam {
    use super::*;
    use glam::*;

    glamvec!(
        (i16, I16Vec3),
        (u16, U16Vec3),
        (i32, IVec3),
        (u32, UVec3),
        (i64, I64Vec3),
        (u64, U64Vec3)
    );

    impl<B, const X: usize, const Y: usize, const Z: usize> PackedIVec3<B, X, Y, Z> where B: BitContainer
    {}
}

/* ---------------------------------------------------------------------------------------------- */
/*                                              TESTS                                             */
/* ---------------------------------------------------------------------------------------------- */

#[cfg(test)]
mod tests {
    use super::PackedIVec3;

    #[test]
    fn test_packed_ivec3_unsigned() {
        let g = PackedIVec3::<u8, 2, 2, 4>::new(1, 2, 3);
        assert_eq!(g.to_arr(), [1, 2, 3]);

        let k = PackedIVec3::<u16, 4, 4, 8>::new(2, 3, 4);
        assert_eq!(k.to_arr(), [2, 3, 4]);

        let k = PackedIVec3::<u16, 4, 4, 8>::new(15, 15, 15);
        assert_eq!(k.to_arr(), [15, 15, 15]);

        let k = PackedIVec3::<u16, 0, 4, 12>::new(0, 15, 4095);
        assert_eq!(k.to_arr(), [0, 15, 4095]);

        // Test case for u32 with different bit widths
        let m = PackedIVec3::<u32, 10, 10, 12>::new(512, 512, 1024);
        assert_eq!(m.to_arr(), [512, 512, 1024]);

        // Test case with maximum values for given bit widths
        let n = PackedIVec3::<u32, 5, 5, 6>::new(31, 31, 63);
        assert_eq!(n.to_arr(), [31, 31, 63]);

        // Test case with minimum values (assuming signed integers are not allowed)
        let o = PackedIVec3::<u8, 3, 3, 2>::new(0, 0, 0);
        assert_eq!(o.to_arr(), [0, 0, 0]);
    }

    #[test]
    fn test_packed_ivec3_signed() {
        // Test case with mixed positive and negative values
        let a = PackedIVec3::<i16, 5, 6, 5>::new(-16, 31, -1);
        assert_eq!(a.to_arr(), [-16, 31, -1]);

        // Test case with all negative values
        let b = PackedIVec3::<i8, 3, 3, 2>::new(-4, -3, -2);
        assert_eq!(b.to_arr(), [-4, -3, -2]);

        // Test case to verify handling of minimum and maximum values for signed integers
        let c = PackedIVec3::<i32, 16, 1, 15>::new(i16::MIN as i32, 0, (i16::MAX >> 1) as i32);
        assert_eq!(c.to_arr(), [i16::MIN as i32, 0, (i16::MAX >> 1) as i32]);

        // Test case to check behavior with values just beyond positive and negative limits
        let d = PackedIVec3::<i16, 4, 4, 8>::new(-9, 8, 128); // Beyond the range of 4-bit signed (-8 to 7) and 8-bit signed (-128 to 127)
        assert_eq!(d.to_arr(), [-8, 7, 127]); // Assuming clipping or wrapping to the closest valid value

        // Test case with zero values to verify no sign bit issues
        let e = PackedIVec3::<i32, 10, 10, 12>::new(0, 0, 0);
        assert_eq!(e.to_arr(), [0, 0, 0]);
    }

    #[test]
    fn test_normals_conversion() {
        // Round-Trip Conversion Test for Unsigned Type
        let original_unsigned = PackedIVec3::<u16, 5, 5, 6>::new(31, 0, 63);
        let normalized_vec = original_unsigned.to_normals();
        let converted_back = PackedIVec3::from_normals(normalized_vec);
        assert_eq!(original_unsigned, converted_back);

        // Round-Trip Conversion Test for Signed Type
        let original_signed = PackedIVec3::<i16, 5, 5, 6>::new(-16, 0, 31);
        let normalized_vec_signed = original_signed.to_normals();
        let converted_back_signed = PackedIVec3::from_normals(normalized_vec_signed);
        assert_eq!(original_signed, converted_back_signed);
    }

    #[test]
    fn test_normalization_accuracy() {
        // Unsigned normalization
        let max_value_unsigned = PackedIVec3::<u16, 5, 5, 6>::new(31, 31, 63);
        let normalized_unsigned = max_value_unsigned.to_normals();
        assert!((normalized_unsigned[0] - 1.0).abs() < 1.0 / 31.0);
        assert!((normalized_unsigned[1] - 1.0).abs() < 1.0 / 31.0);
        assert!((normalized_unsigned[2] - 1.0).abs() < 1.0 / 63.0);

        // Signed normalization
        let max_value_signed = PackedIVec3::<i16, 5, 5, 6>::new(-16, 0, 31);
        let normalized_signed = max_value_signed.to_normals();
        assert!((normalized_signed[0] + 1.0).abs() < 1.0 / 16.0); // Mapping -16 to -1.0
        assert!(normalized_signed[1].abs() < 1.0 / 16.0); // Mapping 0 to 0.0
        assert!((normalized_signed[2] - 1.0).abs() < 1.0 / 31.0); // Mapping 31 to 1.0
    }

    #[test]
    fn test_denormalization_accuracy() {
        // Test denormalization from normals back to packed integers
        let normal_vec = [1.0, 0.5, -1.0];
        let packed = PackedIVec3::<i16, 5, 5, 6>::from_normals(normal_vec);
        // Assuming B::from_normal correctly maps the floating-point range back to integers
        assert_eq!(packed.to_arr(), [15, 8, -32]); // Example expected values, adjust based on actual mapping logic
    }
}
