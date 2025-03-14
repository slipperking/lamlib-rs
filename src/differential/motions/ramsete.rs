use alloc::{boxed::Box, rc::Rc};

use super::ExitConditionGroup;
use crate::{controllers::ControllerMethod, differential::chassis::Chassis, tracking::Tracking};

/// Uses a hybrid between a RAMSETE controller and a closed loop feedback controller
/// such as PID.
pub struct RAMSETEHybridSettings {
    lateral_controller: Box<dyn ControllerMethod<f64>>,
    angular_controller: Box<dyn ControllerMethod<f64>>,
    k_lateral: f64,

    lateral_exit_conditions: ExitConditionGroup<f64>,
    angular_exit_conditions: ExitConditionGroup<f64>,
}

pub struct RAMSETEHybridParameters {
    /// The `k_lateral` here, if set to `Some(_)`, overrides that of [`RAMSETEHybridSettings`].
    pub k_lateral: Option<f64>,
    pub forwards: bool,
    pub min_lateral_speed: f64,
    pub max_lateral_speed: f64,
    pub max_angular_speed: f64,
    pub early_exit_range: f64,
    pub lateral_slew: Option<f64>,
    pub angular_slew: Option<f64>,
}

#[macro_export]
macro_rules! params_ramsete_h {
    (
        $(k_lateral: $k_lateral:expr,)?
        $(forwards: $forwards:expr,)?
        $(min_lateral_speed: $min_lateral_speed:expr,)?
        $(max_lateral_speed: $max_lateral_speed:expr,)?
        $(max_angular_speed: $max_angular_speed:expr,)?
        $(early_exit_range: $early_exit_range:expr,)?
        $(lateral_slew: $lateral_slew:expr,)?
        $(angular_slew: $angular_slew:expr,)?
    ) => {
        #[allow(unused_mut, unused_assignments)]
        {
            let mut k_lateral = None;
            let mut forwards = true;
            let mut min_lateral_speed = 0.0;
            let mut max_lateral_speed = 1.0;
            let mut max_angular_speed = 1.0;
            let mut early_exit_range = 0.0;
            let mut lateral_slew = None;
            let mut angular_slew = None;

            $(k_lateral = Some($k_lateral);)?
            $(forwards = $forwards;)?
            $(min_lateral_speed = $min_lateral_speed;)?
            $(max_lateral_speed = $max_lateral_speed;)?
            $(early_exit_range = $early_exit_range;)?
            $(lateral_slew = Some($lateral_slew);)?
            $(angular_slew = Some($angular_slew);)?

            $crate::differential::motions::ramsete::RAMSETEHybridParameters {
                k_lateral,
                forwards,
                min_lateral_speed,
                max_lateral_speed,
                max_angular_speed,
                early_exit_range,
                lateral_slew,
                angular_slew,
            }
        }
    }
}
pub use params_ramsete_h;

// Control scheme:
// First calculate local error: rotation matrix of (-current heading) * (target - current).
// Then calculate omega as PID(e_θ) + k_lateral * v_d * e_y * sinc(e_θ).
// Calculate v as |cos(e_θ)| * v_d.
impl<T: Tracking + 'static> Chassis<T> {
    pub fn ramsete_hybrid(self: Rc<Self>) {}
}
