use alloc::{boxed::Box, rc::Rc};

use crate::{controllers::ControllerMethod, differential::chassis::Chassis, tracking::Tracking};

/// Uses a hybrid between a RAMSETE controller and a closed loop feedback controller
/// such as PID.
pub struct RAMSETEHybridSettings {
    lateral_controller: Box<dyn ControllerMethod<f64>>,
    angular_controller: Box<dyn ControllerMethod<f64>>,
    k_lateral: f64,
}

// Control scheme:
// First calculate local error: rotation matrix of (-current heading) * (target - current).
// Then calculate omega as PID(e_θ) + k_lat * v_d * e_y * sinc(e_θ).
// Calculate v as |cos(e_θ)| * v_d.
impl<T: Tracking + 'static> Chassis<T> {
    pub fn ramsete_hybrid(self: Rc<Self>) {}
}
