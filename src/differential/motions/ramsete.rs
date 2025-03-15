use alloc::{boxed::Box, rc::Rc};
use core::time::Duration;

use nalgebra::Vector2;

use super::{angular::TurnToParameters, ExitConditionGroup};
use crate::{controllers::ControllerMethod, differential::chassis::Chassis, tracking::Tracking};

#[derive(Clone, Copy, PartialEq)]
pub struct RAMSETEHybridParameters {
    /// The `b` here, if set to `Some(_)`, overrides that of [`RAMSETEHybridSettings`].
    pub b: Option<f64>,
    pub forwards: bool,
    pub min_lateral_speed: f64,
    pub max_lateral_speed: f64,
    pub max_angular_speed: f64,
    pub early_exit_range: f64,
    pub lateral_slew: Option<f64>,
    pub angular_slew: Option<f64>,
}

#[derive(Clone)]
/// Uses a hybrid between a RAMSETE controller and a closed loop feedback controller
/// such as PID.
pub struct RAMSETEHybridSettings {
    lateral_controller: Box<dyn ControllerMethod<f64>>,
    angular_controller: Box<dyn ControllerMethod<f64>>,

    lateral_exit_conditions: ExitConditionGroup<f64>,
    angular_exit_conditions: ExitConditionGroup<f64>,

    b: f64,
}

impl RAMSETEHybridSettings {
    pub fn new(
        lateral_controller: Box<dyn ControllerMethod<f64>>,
        angular_controller: Box<dyn ControllerMethod<f64>>,
        lateral_exit_conditions: ExitConditionGroup<f64>,
        angular_exit_conditions: ExitConditionGroup<f64>,
        b: f64,
    ) -> Self {
        Self {
            lateral_controller,
            angular_controller,
            lateral_exit_conditions,
            angular_exit_conditions,
            b,
        }
    }

    pub fn reset(&mut self) {
        self.angular_controller.reset();
        self.angular_exit_conditions.reset();
    }
}

#[macro_export]
macro_rules! params_ramsete_h {
    (
        $(b: $b:expr,)?
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
            let mut b = None;
            let mut forwards = true;
            let mut min_lateral_speed = 0.0;
            let mut max_lateral_speed = 1.0;
            let mut max_angular_speed = 1.0;
            let mut early_exit_range = 0.0;
            let mut lateral_slew = None;
            let mut angular_slew = None;

            $(b = Some($b);)?
            $(forwards = $forwards;)?
            $(min_lateral_speed = $min_lateral_speed;)?
            $(max_lateral_speed = $max_lateral_speed;)?
            $(max_angular_speed = $max_angular_speed;)?
            $(early_exit_range = $early_exit_range;)?
            $(lateral_slew = Some($lateral_slew);)?
            $(angular_slew = Some($angular_slew);)?

            $crate::differential::motions::ramsete::RAMSETEHybridParameters {
                b,
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
// Then calculate omega as PID(e_θ) + b * v_d * e_y * sinc(e_θ).
// Calculate v as |cos(e_θ)| * v_d.
impl<T: Tracking + 'static> Chassis<T> {
    pub async fn ramsete_hybrid(
        self: Rc<Self>,
        target: Vector2<f64>,
        timeout: Option<Duration>,
        params: Option<RAMSETEHybridParameters>,
        mut settings: Option<RAMSETEHybridSettings>,
        run_async: bool,
    ) {
        let mut unwrapped_params = params.unwrap_or(params_ramsete_h!());
        self.motion_handler.wait_for_motions_end().await;
        if self.motion_handler.in_motion() {
            return;
        }
        if run_async {
            // Spawn vexide task
            vexide::task::spawn({
                let self_clone = self.clone();
                async move {
                    self_clone
                        .ramsete_hybrid(target, timeout, params, settings.clone(), false)
                        .await
                }
            })
            .detach();
            self.motion_handler.end_motion().await;
            vexide::time::sleep(Duration::from_millis(10)).await;
            return;
        }
        if let Some(settings) = &mut settings {
            settings.reset();
        } else {
            self.motion_settings
                .ramsete_hybrid_settings
                .borrow_mut()
                .reset();
        }
        {
            *self.distance_traveled.borrow_mut() = Some(0.0);
        }
    }
}
