use num_traits::float::Float;
pub struct ExponentialDriveCurve {
    /// Minimum input.
    pub deadzone: f64,

    /// Minimum output.
    pub min_output: f64,

    /// How curved it is.
    pub curve_intensity: f64,
}
impl ExponentialDriveCurve {
    pub fn new(deadzone: f64, min_output: f64, curve_intensity: f64) -> Self {
        Self {
            deadzone,
            min_output,
            curve_intensity,
        }
    }
    /// The following calculations are based off of https://www.desmos.com/calculator/e70h0a936n.
    pub fn update(&self, input: f64, maximum: f64) -> f64 {
        if input.abs() <= self.deadzone {
            return 0.0;
        }

        let g: f64 = input.abs() - self.deadzone;
        let g_max: f64 = maximum - self.deadzone;
        let i: f64 = self.curve_intensity.powf(g - maximum) * g * (input).signum();
        let i_max: f64 = self.curve_intensity.powf(g_max - maximum) * g_max;
        (maximum - self.min_output) / maximum * i * maximum / i_max
            + self.min_output * input.signum()
    }
}
