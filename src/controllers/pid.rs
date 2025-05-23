use core::ops::AddAssign;

use num_traits::{Float, FromPrimitive, Zero};

use super::FeedbackController;
use crate::utils::differential_tracker::DifferentialTracker;
pub struct PID<T: Float> {
    /// Struct [`PIDGains`] containing the gains.
    gains: PIDGains<T>,

    /// Range that integral accumulates in.
    /// The integral value resets when the error is outside of this range.     
    windup_range: T,

    /// Whether or not to reset the accumulated integral when sign flips
    reset_on_sign_flip: bool,

    differential_tracker: DifferentialTracker<T>,
}

#[derive(Clone)]
pub struct PIDGains<T> {
    kp: T, // Proportional gain
    ki: T, // Integral gain
    kd: T, // Derivative gain
}

impl<T: Float + Zero> PID<T> {
    pub fn new(
        kp: T,
        ki: T,
        kd: T,
        windup_range: T,
        reset_on_sign_flip: bool,
        differential_tracker_len: usize,
    ) -> Self {
        assert!(
            differential_tracker_len >= 2,
            "Differential tracker length must be at least 2 to support a derivative."
        );
        Self {
            gains: PIDGains { kp, ki, kd },
            reset_on_sign_flip,
            windup_range,
            differential_tracker: DifferentialTracker::new(differential_tracker_len),
        }
    }
    pub fn from_pid_gains(
        gains: PIDGains<T>,
        windup_range: T,
        reset_on_sign_flip: bool,
        differential_tracker_len: usize,
    ) -> Self {
        assert!(
            differential_tracker_len >= 2,
            "Differential tracker length must be at least 2 to support a derivative."
        );
        Self {
            gains,
            reset_on_sign_flip,
            windup_range,
            differential_tracker: DifferentialTracker::new(differential_tracker_len),
        }
    }
}
impl<T: Float + Zero + FromPrimitive + AddAssign + num_traits::Float> FeedbackController<T>
    for PID<T>
{
    fn update(&mut self, set_point: T, process_variable: T) -> T {
        let error = set_point - process_variable;
        self.differential_tracker.update(error);

        if Float::signum(error)
            != Float::signum({
                if let Some(prev_error) = self
                    .differential_tracker
                    .at_index(self.differential_tracker.space_size() - 2)
                {
                    prev_error
                } else {
                    error
                }
            })
            && self.reset_on_sign_flip
            || self.windup_range != T::zero() && Float::abs(error) > self.windup_range
        {
            self.differential_tracker.reset_integral();
        }
        let derivative: T = self.differential_tracker.derivative(1).unwrap_or(T::zero());
        self.gains.kp * error
            + self.gains.ki * self.differential_tracker.integral()
            + self.gains.kd * derivative
    }

    fn reset(&mut self) {
        self.differential_tracker.reset();
    }
}

impl<T: num_traits::Float> Clone for PID<T> {
    fn clone(&self) -> Self {
        Self {
            gains: self.gains.clone(),
            windup_range: self.windup_range,
            reset_on_sign_flip: self.reset_on_sign_flip,
            differential_tracker: DifferentialTracker::new(self.differential_tracker.space_size()),
        }
    }
}
