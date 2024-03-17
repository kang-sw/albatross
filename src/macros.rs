#[macro_export]
macro_rules! define_custom_vector {
    (
        $(#[$meta:meta])*
        $vis:vis struct $name:ident {
            $(
                $(#[$elem_meta:meta])*
                $elem:ident: $elem_ty:ty,
            )*
        }

        $($rest:tt)*
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq)]
        #[derive(
            $crate::derive_more::Add,
            $crate::derive_more::Sub,
            $crate::derive_more::AddAssign,
            $crate::derive_more::SubAssign,
            $crate::derive_more::MulAssign,
            $crate::derive_more::DivAssign
        )]
        $vis struct $name {
            $(
                $(#[$elem_meta])*
                pub $elem: $elem_ty,
            )*
        }

        #[allow(unused)]
        const _: () = {
            use $crate::num::traits::*;

            impl $name {
                #[inline]
                pub const fn new($($elem: $elem_ty),*) -> Self {
                    Self {
                        $($elem),*
                    }
                }

                #[inline]
                pub const fn to_tuple(self) -> ($($elem_ty,)*) {
                    ($(
                        self.$elem,
                    )*)
                }

                #[inline]
                pub fn to_array<T>(self) -> [
                    T; $crate::define_custom_vector!(@count $($elem )*)
                ] where
                T: Num + Copy + 'static,
                $($elem_ty: AsPrimitive<T>,)*
                {
                    [
                        $(
                            AsPrimitive::as_(self.$elem),
                        )*
                    ]
                }

            }

            impl<T> From<[T; $crate::define_custom_vector!(@count $($elem )*)]> for $name
            where
                $(T: AsPrimitive<$elem_ty>,)*
            {
                #[inline]
                fn from([$( $elem ),*]: [T; $crate::define_custom_vector!(@count $($elem )*)]) -> Self {
                    Self {
                        $($elem: $elem.as_(),)*
                    }
                }
            }

            impl<__R> std::ops::Mul<__R> for $name
                where __R: Copy $(+AsPrimitive<$elem_ty>)*
            {
                type Output = Self;

                #[inline]
                fn mul(self, rhs: __R) -> Self::Output
                {
                    Self {
                        $($elem: self.$elem * AsPrimitive::<$elem_ty>::as_(rhs),)*
                    }
                }
            }

            impl<__R> std::ops::Div<__R> for $name
                where __R: Copy $(+AsPrimitive<$elem_ty>)*
            {
                type Output = Self;

                #[inline]
                fn div(self, rhs: __R) -> Self::Output
                {
                    Self {
                        $($elem: self.$elem / AsPrimitive::<$elem_ty>::as_(rhs),)*
                    }
                }
            }
        };

        $crate::define_custom_vector!( $($rest)* );
    };

    (@count $head:ident) => { 1 };
    (@count $head:ident $($tail:ident)*) => { 1 + $crate::define_custom_vector!(@count $($tail)*) };

    () => {};
}

#[test]
fn foo() {
    define_custom_vector!(
        struct MyVec {
            x: i32,
            y: f32,
            z: f64,
            w: i32,
        }
    );

    let v = MyVec::new(1, 2.0, 3.0, 4);
    assert_eq!(v.to_tuple(), (1, 2.0, 3.0, 4));
    assert_eq!(v.to_array::<i32>(), [1, 2, 3, 4]);
    assert_eq!(MyVec::from([1, 2, 3, 4]), v);
    assert_eq!(v + v, MyVec::new(2, 4.0, 6.0, 8));
    assert_eq!(v - v, MyVec::new(0, 0.0, 0.0, 0));
    assert_eq!(v * 2, MyVec::new(2, 4.0, 6.0, 8));
    assert_eq!(v / 2, MyVec::new(0, 1.0, 1.5, 2));
}
