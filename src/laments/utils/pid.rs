use super::math;
pub struct PID {
    k_p: f32,                 // Proportional gain
    k_i: f32,                 // Integral gain
    k_d: f32,                 // Derivative gain
    prev_error: f32,          // Previous error for derivative calculation
    integral: f32,            // Integral sum for integral term
    windup_range: f32,        // Range where integral starts accumulating
    reset_on_sign_flip: bool, // Whether or not to reset integral when sign flips
}

impl PID {
    pub fn new(k_p: f32, k_i: f32, k_d: f32, windup_range: f32, reset_on_sign_flip: bool) -> Self {
        PID {
            k_p,
            k_i,
            k_d,
            prev_error: 0.0,
            integral: 0.0,
            reset_on_sign_flip,
            windup_range,
        }
    }

    pub fn update(&mut self, error: f32, delta_time: f32) -> f32 {
        self.integral += error * delta_time;
        if math::fsgn!(error) != math::fsgn!(self.prev_error) && self.reset_on_sign_flip
            || self.windup_range != 0.0 && error.abs() > self.windup_range
        {
            self.integral = 0.0f32;
        }
        let derivative: f32 = (error - self.prev_error) / delta_time;
        let output: f32 = self.k_p * error + self.k_i * self.integral + self.k_d * derivative;
        self.prev_error = error;
        output
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}
