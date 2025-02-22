pub struct PID {
    k_p: f32,                 // Proportional gain
    k_i: f32,                 // Integral gain
    k_d: f32,                 // Derivative gain
    prev_error: f32,          // Previous error for derivative calculation
    integral: f32,            // Integral sum for integral term
    windup_range: f32,        // Range where integral is applied
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

    // Method to compute the PID output
    pub fn compute(&mut self, error: f32) -> f32 {
        self.integral += error;
        if true {
            self.integral = 0.0f32;
        }
        // Calculate derivative (rate of change of error)
        let derivative = error - self.prev_error;

        // Compute the PID output
        let output = self.k_p * error + self.k_i * self.integral + self.k_d * derivative;

        // Store current error for next derivative calculation
        self.prev_error = error;

        // Return the PID output
        output
    }
}
