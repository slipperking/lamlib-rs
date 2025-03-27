use core::ops::AddAssign;

use num_traits::{FromPrimitive, Zero};
use vexide::{float::Float, time::Instant};

use super::FeedbackController;
pub struct PID<T> {
    /// Struct [`PIDGains`] containing the gains.
    gains: PIDGains<T>,

    /// Previous error for derivative calculation.
    prev_error: T,

    /// Previous update time for integral and derivative calculations.         
    prev_time: Option<Instant>,

    /// Accumulated integral sum.
    integral: T,

    /// Range that integral accumulates in.
    /// The integral value resets when the error is outside of this range.     
    windup_range: T,

    /// Whether or not to reset the accumulated integral when sign flips
    reset_on_sign_flip: bool,
}

#[derive(Clone)]
pub struct PIDGains<T> {
    kp: T, // Proportional gain
    ki: T, // Integral gain
    kd: T, // Derivative gain
}

impl<T: Float + FromPrimitive + AddAssign + Zero> PID<T> {
    pub fn new(kp: T, ki: T, kd: T, windup_range: T, reset_on_sign_flip: bool) -> Self {
        Self {
            gains: PIDGains { kp, ki, kd },
            prev_error: T::zero(),
            integral: T::zero(),
            prev_time: None,
            reset_on_sign_flip,
            windup_range,
        }
    }
    pub fn from_pid_gains(gains: PIDGains<T>, windup_range: T, reset_on_sign_flip: bool) -> Self {
        Self {
            gains,
            prev_error: T::zero(),
            integral: T::zero(),
            prev_time: None,
            reset_on_sign_flip,
            windup_range,
        }
    }
}
impl<T: Float + Zero + FromPrimitive + AddAssign + num_traits::Float> FeedbackController<T>
    for PID<T>
{
    fn update(&mut self, error: T) -> T {
        let current_time = Instant::now();
        let delta_time = match self.prev_time {
            Some(instant) => {
                T::from_f64(current_time.duration_since(instant).as_secs_f64() * 1000.0)
            }
            None => None,
        }
        .unwrap_or(T::zero());
        self.prev_time = Some(current_time);
        self.integral += error * delta_time;
        if Float::signum(error) != Float::signum(self.prev_error) && self.reset_on_sign_flip
            || self.windup_range != T::zero() && Float::abs(error) > self.windup_range
        {
            self.integral = T::zero();
        }
        let derivative: T = (error - self.prev_error)
            / if delta_time.is_zero() {
                T::infinity()
            } else {
                delta_time
            };
        let output: T =
            self.gains.kp * error + self.gains.ki * self.integral + self.gains.kd * derivative;
        self.prev_error = error;
        output
    }

    fn reset(&mut self) {
        self.integral = T::zero();
        self.prev_error = T::zero();
        self.prev_time = None;
    }
}

impl<T: num_traits::Float> Clone for PID<T> {
    fn clone(&self) -> Self {
        Self {
            gains: self.gains.clone(),
            prev_error: T::zero(),
            prev_time: None,
            integral: T::zero(),
            windup_range: self.windup_range,
            reset_on_sign_flip: self.reset_on_sign_flip,
        }
    }
}
