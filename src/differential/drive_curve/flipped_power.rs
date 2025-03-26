use num_traits::float::Float;

use super::DriveCurve;
use crate::{ilerp, lerp};

#[derive(Clone)]
pub struct FlippedPowerDriveCurve {
    pub dead_zone: f64,
    pub min_output: f64,
    pub curve_intensity: f64,
}

impl FlippedPowerDriveCurve {
    pub fn new(dead_zone: f64, min_output: f64, curve_intensity: f64) -> Self {
        Self {
            dead_zone,
            min_output,
            curve_intensity,
        }
    }
}
impl DriveCurve for FlippedPowerDriveCurve {
    /// The following calculations are based off of https://www.desmos.com/calculator/tnh4tmq22k.
    fn update(&self, mut input: f64) -> f64 {
        input = input.clamp(-1.0, 1.0);
        if (-self.dead_zone..self.dead_zone).contains(&input) {
            return 0.0;
        }

        input.signum()
            * lerp!(
                self.min_output,
                1.0,
                1.0 - ilerp!(self.dead_zone, 1.0, 1.0 + self.dead_zone - input.abs())
                    .powf(1.0 / self.curve_intensity)
            )
    }
}
