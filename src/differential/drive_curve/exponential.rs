use num_traits::float::Float;

use super::DriveCurve;

#[derive(Clone)]
pub struct ExponentialDriveCurve {
    pub dead_zone: f64,
    pub min_output: f64,

    /// Recommended range is from 1.0 to 1.5.
    pub curve_intensity: f64,
}

impl ExponentialDriveCurve {
    pub fn new(dead_zone: f64, min_output: f64, curve_intensity: f64) -> Self {
        Self {
            dead_zone,
            min_output,
            curve_intensity,
        }
    }
}
impl DriveCurve for ExponentialDriveCurve {
    /// The following calculations are based off of https://www.desmos.com/calculator/e70h0a936n.
    fn update(&self, mut input: f64) -> f64 {
        input = input.clamp(-1.0, 1.0);
        if (-self.dead_zone..self.dead_zone).contains(&input) {
            return 0.0;
        }

        let g: f64 = input.abs() - self.dead_zone;
        let g_max: f64 = 1.0 - self.dead_zone;
        let i: f64 = self.curve_intensity.powf(g - 1.0) * g * (input).signum();
        let i_max: f64 = self.curve_intensity.powf(g_max - 1.0) * g_max;
        (1.0 - self.min_output) * i / i_max + self.min_output * input.signum()
    }
}
