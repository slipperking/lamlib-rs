use core::{
    ops::{Add, Div, Mul, Neg, Rem, Sub},
    time::Duration,
};

use num_traits::{FromPrimitive, Zero};
use vexide::float::Float;

/// Represents the direction for angular calculations.
#[derive(Clone, Copy, PartialEq)]
pub enum AngularDirection {
    /// Clockwise rotation direction.
    Clockwise,
    /// Counterclockwise rotation direction.
    Counterclockwise,
}
/// Computes the remainder of `a` divided by `b`.
///
/// This function is similar to the modulo operation but ensures the result is closer to 0
/// than the value on the other side of 0.
///
/// # Examples
/// ```
/// use crate::math::remainder;
/// assert_eq!(remainder(7.0, 4.0), -1.0);
/// assert_eq!(remainder(-7.0, 4.0), -3.0);
/// ```
pub fn remainder<T: Div<Output = T> + Sub<Output = T> + Float + Copy + Mul<Output = T>>(
    a: T,
    b: T,
) -> T {
    let n = (a / b).round();
    a - b * n
}

/// Computes the probability density function (PDF) of a Gaussian distribution at `x`.
///
/// # Arguments
/// * `x` - The point to evaluate the PDF at.
/// * `mu` - The mean (μ) of the distribution.
/// * `sigma` - The standard deviation (σ) of the distribution.
///
/// # Examples
/// ```
/// use crate::math::gaussian_pdf;
/// let value = gaussian_pdf(0.0, 0.0, 1.0);
/// assert!((value - 0.3989422804).abs() < 1e-9);
/// ```
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
    T::from_f32(1.0).unwrap_or_default()
        / (sigma
            * (T::from_u8(2).unwrap_or_default()
                * T::from_f64(core::f64::consts::PI).unwrap_or_default())
            .sqrt())
        * exponent.exp()
}

/// Computes the unsigned modulus, ensuring the result is positive.
///
/// # Examples
/// ```
/// # use crate::unsigned_mod;
/// assert_eq!(unsigned_mod!(-1, 3), 2);
/// assert_eq!(unsigned_mod!(5, 3), 2);
/// ```
#[macro_export]
macro_rules! unsigned_mod {
    ($dividend:expr, $divisor:expr) => {
        (($dividend % $divisor) + $divisor) % $divisor
    };
}

/// Linearly interpolates between two values.
///
/// When `t` is 0.0, returns `value1`; when `t` is 1.0, returns `value2`.
///
/// # Examples
/// ```
/// # use crate::lerp;
/// assert_eq!(lerp!(1.0, 3.0, 0.5), 2.0);
/// ```
#[macro_export]
macro_rules! lerp {
    ($value1:expr, $value2:expr, $t:expr) => {
        $value1 + ($value2 - $value1) * $t
    };
}

/// Computes the inverse linear interpolation parameter `t` between two values.
///
/// # Examples
/// ```
/// # use crate::ilerp;
/// assert_eq!(ilerp!(1.0, 3.0, 2.0), 0.5);
/// ```
#[macro_export]
macro_rules! ilerp {
    ($value1:expr, $value2:expr, $inter:expr) => {
        ($inter - $value1) / ($value2 - $value1)
    };
}

/// Computes the average of non-`None` values in a vector.
///
/// Returns `None` if there are no valid values.
///
/// # Examples
/// ```
/// # use crate::avg_valid;
/// let values = vec![Some(2.0), None, Some(4.0)];
/// assert_eq!(avg_valid!(values), Some(3.0));
/// ```
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
pub use unsigned_mod;

/// Clamps the change from `current` towards `target` by `max_delta` scaled by `delta_time`.
///
/// # Arguments
/// * `target` - The value to approach.
/// * `current` - The current value.
/// * `max_delta` - The maximum allowed change per unit time.
/// * `delta_time` - Optional time elapsed since the last update.
///
/// # Examples
/// ```
/// use core::time::Duration;
/// use crate::math::delta_clamp;
/// let result = delta_clamp(10.0, 5.0, 2.0, Some(Duration::from_secs(1)));
/// assert_eq!(result, 7.0);
/// ```
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

/// Normalizes an angle to [0, 2π) radians or [0, 360) degrees.
///
/// # Arguments
/// * `radians` - If `true`, uses radians; degrees otherwise.
///
/// # Examples
/// ```
/// use crate::math::sanitize_angle;
/// assert_eq!(sanitize_angle(450.0, false), 90.0);
/// ```
pub fn sanitize_angle<T: FromPrimitive + Rem<Output = T> + Add<Output = T> + Default>(
    angle: T,
    radians: bool,
) -> T {
    if radians {
        unsigned_mod!(
            angle,
            T::from_f64(core::f64::consts::TAU).unwrap_or_default()
        )
    } else {
        unsigned_mod!(angle, T::from_u16(360).unwrap_or_default())
    }
}

/// Calculates the shortest signed angular difference between two angles.
///
/// # Arguments
/// * `direction` - Optional [`AngularDirection`] to force error direction.
///
/// # Examples
/// ```
/// use crate::math::{angle_error, AngularDirection};
/// assert_eq!(angle_error(10.0, 350.0, false, None), 20.0);
/// assert_eq!(angle_error(350.0, 10.0, false, None), -20.0);
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

/// Adjusts lateral and angular inputs to prevent motor saturation.
///
/// Returns normalized (left, right) motor values.
///
/// # Examples
/// ```
/// use crate::math::arcade_desaturate;
/// let (left, right) = arcade_desaturate(0.8, 0.6);
/// assert_eq!(left, (0.8 - 0.6) / 1.4);
/// ```
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

/// Default is heading, `standard` defaults to false.
#[macro_export]
macro_rules! angle {
    (
        $(degrees: $degrees:expr,)?
        $(radians: $radians:expr,)?
        $(standard: $standard:expr,)?
    ) => {
        #[allow(unused_mut, unused_assignments)]
        {
            let mut degrees: Option<f64> = None;
            let mut radians: Option<f64> = None;
            let mut standard = false;
            $(degrees = Some($degrees as f64);)?
            $(radians = Some($radians as f64);)?
            $(standard = $standard;)?

            if let Some(degrees) = degrees {
                if standard {
                    degrees.to_radians()
                }
                else {
                    (90.0 - degrees).to_radians()
                }
            }
            else if let Some(radians) = radians {
                if standard {
                    radians
                }
                else {
                    core::f64::consts::FRAC_PI_2 - radians
                }
            }
            else {
                0.0
            }
        }
    };
}

pub trait AngleExt: num_traits::Float {
    fn std_rad(self) -> Self;
    fn std_deg(self) -> Self;
    fn hdg_rad(self) -> Self;
    fn hdg_deg(self) -> Self;
    fn rad(self) -> Self;
    fn deg(self) -> Self;
}

impl<T: num_traits::Float> AngleExt for T {
    fn std_rad(self) -> Self {
        self
    }

    fn std_deg(self) -> Self {
        self.to_radians()
    }

    fn hdg_rad(self) -> Self {
        T::from(90.0).unwrap().to_radians() - self
    }

    fn hdg_deg(self) -> Self {
        T::from(90.0).unwrap() - self
    }

    fn rad(self) -> Self {
        self
    }

    fn deg(self) -> Self {
        self.to_radians()
    }
}
