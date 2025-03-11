use core::ops::{Add, Div, Mul, Neg, Rem, Sub};

use num::{FromPrimitive, Zero};
use vexide::float::Float;

pub enum AngularDirection {
    Clockwise,
    CounterClockwise,
    Auto,
}

pub fn gaussian_pdf<
    T: FromPrimitive
        + Copy
        + Float
        + Mul<Output = T>
        + Div<Output = T>
        + Neg<Output = T>
        + Default
        + Sub<Output = T>,
>(
    x: T,
    mu: T,
    sigma: T,
) -> T {
    let exponent = -(x - mu) * (x - mu) / (T::from_u8(2).unwrap_or_default() * sigma * sigma);
    (T::from_f32(1.0).unwrap_or_default()
        / (sigma
            * (T::from_u8(2).unwrap_or_default()
                * T::from_f64(core::f64::consts::PI).unwrap_or_default())
            .sqrt()))
        * exponent.exp()
}

#[macro_export]
macro_rules! signed_mod {
    ($dividend:expr, $divisor:expr) => {
        (($dividend % $divisor) + $divisor) % $divisor
    };
}
#[macro_export]
macro_rules! lerp {
    ($value1:expr, $value2:expr, $t:expr) => {
        $value1 + ($value2 - $value1) * $t
    };
}

#[macro_export]
macro_rules! ilerp {
    ($value1:expr, $value2:expr, $inter:expr) => {
        ($inter - $value1) / ($value2 - $value1)
    };
}

#[macro_export]
macro_rules! avg_valid {
    ($vec:expr) => {{
        let (sum, count) = $vec
            .iter()
            .filter_map(|&x| x)
            .fold((0.0, 0), |(sum, count), x| (sum + x, count + 1));

        if count > 0 {
            Some(sum / count as f64)
        } else {
            None
        }
    }};
}

pub use avg_valid;
pub use ilerp;
pub use lerp;
pub use signed_mod;

pub fn delta_clamp<
    T: Neg<Output = T> + Zero + Copy + PartialEq + num_traits::Float + Sub<Output = T>,
>(
    value: T,
    prev: T,
    max_delta: T,
) -> T {
    if max_delta == T::zero() {
        return value;
    }
    prev + (value - prev).clamp(-max_delta.abs(), max_delta.abs())
}

pub fn sanitize_angle<T: FromPrimitive + Rem<Output = T> + Add<Output = T> + Default>(
    angle: T,
    radians: bool,
) -> T {
    if radians {
        signed_mod!(
            angle,
            T::from_f64(core::f64::consts::TAU).unwrap_or_default()
        )
    } else {
        signed_mod!(angle, T::from_u16(360).unwrap_or_default())
    }
}

pub fn angle_error<
    T: Copy
        + FromPrimitive
        + Default
        + Sub<Output = T>
        + Add<Output = T>
        + Zero
        + PartialOrd
        + Rem<Output = T>,
>(
    mut target: T,
    current: T,
    radians: bool,
    direction: AngularDirection,
) -> T {
    target = sanitize_angle(target, radians);
    target = sanitize_angle(target, radians);
    let max: T = if radians {
        T::from_f64(core::f64::consts::TAU).unwrap_or_default()
    } else {
        T::from_f64(360.0).unwrap_or_default()
    };
    let raw_error: T = target - current;
    match direction {
        AngularDirection::Clockwise => {
            if raw_error < T::zero() {
                raw_error + max
            } else {
                raw_error
            }
        }
        AngularDirection::CounterClockwise => {
            if raw_error > T::zero() {
                raw_error - max
            } else {
                raw_error
            }
        }
        _ => {
            signed_mod!(raw_error, max)
        }
    }
}
