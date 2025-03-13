use core::{
    ops::{Add, Div, Mul, Neg, Rem, Sub},
    time::Duration,
};

use num::{FromPrimitive, Zero};
use vexide::float::Float;

pub enum AngularDirection {
    Clockwise,
    Counterclockwise,
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

fn remainder<T: Div<Output = T> + Sub<Output = T> + Float + Copy + Mul<Output = T>>(
    a: T,
    b: T,
) -> T {
    let n = (a / b).round();
    a - b * n
}

pub fn delta_clamp<
    T: Neg<Output = T>
        + Zero
        + Copy
        + PartialEq
        + num_traits::Float
        + Sub<Output = T>
        + FromPrimitive
        + Default,
>(
    target: T,
    current: T,
    max_delta: T,
    delta_time: Option<Duration>,
) -> T {
    if max_delta == T::zero() {
        return target;
    }
    let delta_time = match delta_time {
        Some(duration) => T::from_f64(duration.as_secs_f64()).unwrap_or(T::from_u8(1).unwrap()),
        None => T::from_u8(1).unwrap(),
    };
    current + (target - current).clamp(-max_delta.abs() * delta_time, max_delta.abs() * delta_time)
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

/// Calculates the error between two angles. Useful when calculating the difference between two headings.
///
/// # Parameters
/// - `target`: The target angle.
/// - `current`: The current angle.
/// - `radians`: Whether the angles are in radians.
/// - `direction`: A [`AngularDirection`] of the direction to calculate the error in.
///
/// # Returns
/// The signed angle difference, wrapped within the valid range.
///
/// # Examples
/// ```
/// assert_eq!(angle_error(10.0, 350.0, false, AngularDirection::Auto), 20.0);
/// assert_eq!(angle_error(350.0, 10.0, false, AngularDirection::Auto), -20.0);
/// ```
pub fn angle_error<
    T: Copy
        + Float
        + FromPrimitive
        + Zero
        + PartialOrd
        + Default
        + Sub<Output = T>
        + Add<Output = T>
        + Rem<Output = T>
        + Div<Output = T>
        + Mul<Output = T>,
>(
    mut target: T,
    mut current: T,
    radians: bool,
    direction: Option<AngularDirection>,
) -> T {
    target = sanitize_angle(target, radians);
    current = sanitize_angle(current, radians);
    let max: T = if radians {
        T::from_f64(core::f64::consts::TAU).unwrap_or_default()
    } else {
        T::from_u16(360).unwrap_or_default()
    };
    let raw_error: T = target - current;
    if let Some(direction) = direction {
        match direction {
            AngularDirection::Clockwise => {
                if raw_error > T::zero() {
                    raw_error - max
                } else {
                    raw_error
                }
            }
            AngularDirection::Counterclockwise => {
                if raw_error < T::zero() {
                    raw_error + max
                } else {
                    raw_error
                }
            }
        }
    } else {
        remainder(raw_error, max)
    }
}

pub fn arcade_desaturate<
    T: Copy
        + Sub<Output = T>
        + Add<Output = T>
        + Float
        + PartialOrd<T>
        + Div<Output = T>
        + FromPrimitive,
>(
    lateral: T,
    angular: T,
) -> (T, T) {
    let left: T = lateral - angular;
    let right: T = lateral + angular;
    let sum = {
        let raw_sum = lateral.abs() + angular.abs();
        let one = T::from_u8(1).unwrap();
        if raw_sum < one {
            one
        } else {
            raw_sum
        }
    };
    (left / sum, right / sum)
}
