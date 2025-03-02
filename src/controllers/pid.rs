use core::ops::AddAssign;

use num::FromPrimitive;
use num_traits::Float;
use vexide::core::time::Instant;

use super::ControllerMethod;
pub struct PID<T: Float + FromPrimitive + AddAssign> {
    gains: PIDGains<T>,
    prev_error: T, // Previous error for derivative calculation
    prev_time: Option<crate::Instant>,
    integral: T,              // Integral sum for integral term
    windup_range: T,          // Range where integral starts accumulating
    reset_on_sign_flip: bool, // Whether or not to reset integral when sign flips
}
pub struct PIDGains<T: Float + FromPrimitive + AddAssign> {
    kp: T, // Proportional gain
    ki: T, // Integral gain
    kd: T, // Derivative gain
}

impl<T: Float + FromPrimitive + AddAssign> PID<T> {
    pub fn new(kp: T, ki: T, kd: T, windup_range: T, reset_on_sign_flip: bool) -> Self {
        PID {
            gains: PIDGains { kp, ki, kd },
            prev_error: T::zero(),
            integral: T::zero(),
            prev_time: None,
            reset_on_sign_flip,
            windup_range,
        }
    }
}
impl<T: Float + FromPrimitive + AddAssign> ControllerMethod<T>
    for PID<T>
{
    fn update(&mut self, error: T) -> T {
        let current_time = Instant::now();
        let delta_time = T::from_u128(match self.prev_time {
            Some(instant) => current_time.duration_since(instant),
            None => core::time::Duration::ZERO,
        }
        .as_millis()).unwrap();
        self.prev_time = Some(current_time);
        self.integral += error * delta_time;
        if error.signum() != self.prev_error.signum() && self.reset_on_sign_flip
            || self.windup_range != T::zero() && error.abs() > self.windup_range
        {
            self.integral = T::zero();
        }
        let derivative: T = (error - self.prev_error) / delta_time;
        let output: T =
            self.gains.kp * error + self.gains.ki * self.integral + self.gains.kd * derivative;
        self.prev_error = error;
        output
    }

    fn reset(&mut self) {
        self.integral = T::zero();
        self.prev_error = T::zero();
    }
}
