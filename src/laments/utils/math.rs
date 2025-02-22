use core::f32::consts as f32_consts;

use vexide::core::float::Float;

pub enum AngularDirection {
    CW,
    CCW,
    AUTO,
}

pub fn gaussian_pdf(x: f32, mu: f32, sigma: f32) -> f32 {
    let exponent = -(x - mu) * (x - mu) / (2.0f32 * sigma * sigma);
    (1.0 / (sigma * (2.0 * f32_consts::PI).sqrt())) * exponent.exp()
}

macro_rules! signed_mod {
    ($dividend:expr, $divisor:expr) => {
        (($dividend % $divisor) + $divisor) % $divisor
    };
}

pub fn sanitize_angle(angle: f32, radians: bool) -> f32 {
    if radians {
        signed_mod!(angle, f32_consts::TAU)
    } else {
        signed_mod!(angle, 360.0f32)
    }
}

pub fn angle_error(
    mut target: f32,
    current: f32,
    radians: bool,
    direction: AngularDirection,
) -> f32 {
    target = sanitize_angle(target, radians);
    target = sanitize_angle(target, radians);
    let max: f32 = if radians { f32_consts::TAU } else { 360.0f32 };
    let raw_error: f32 = target - current;
    match direction {
        AngularDirection::CW => {
            if raw_error < 0.0f32 {
                raw_error + max
            } else {
                raw_error
            }
        }
        AngularDirection::CCW => {
            if raw_error > 0.0f32 {
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
