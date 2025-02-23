use super::math;
pub struct PID {
    gains: PIDGains,
    prev_error: f32, // Previous error for derivative calculation
    prev_time: Option<crate::Instant>,
    integral: f32,            // Integral sum for integral term
    windup_range: f32,        // Range where integral starts accumulating
    reset_on_sign_flip: bool, // Whether or not to reset integral when sign flips
}
pub struct PIDGains {
    kp: f32, // Proportional gain
    ki: f32, // Integral gain
    kd: f32, // Derivative gain
}

impl PID {
    pub fn new(kp: f32, ki: f32, kd: f32, windup_range: f32, reset_on_sign_flip: bool) -> Self {
        PID {
            gains: PIDGains { kp, ki, kd },
            prev_error: 0.0,
            integral: 0.0,
            prev_time: None,
            reset_on_sign_flip,
            windup_range,
        }
    }

    pub fn update(&mut self, error: f32) -> f32 {
        let delta_time = match self.prev_time {
            Some(instant) => instant.elapsed(),
            None => core::time::Duration::ZERO,
        }
        .as_millis() as f32;
        self.integral += error * delta_time;
        if math::fsgn!(error) != math::fsgn!(self.prev_error) && self.reset_on_sign_flip
            || self.windup_range != 0.0 && error.abs() > self.windup_range
        {
            self.integral = 0.0f32;
        }
        let derivative: f32 = (error - self.prev_error) / delta_time;
        let output: f32 =
            self.gains.kp * error + self.gains.ki * self.integral + self.gains.kd * derivative;
        self.prev_error = error;
        output
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}
