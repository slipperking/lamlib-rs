use alloc::{boxed::Box, rc::Rc};
use core::{f64::consts::PI, time::Duration};

use bon::bon;
use nalgebra::{Vector2, Vector3};
use vexide::{float::Float, prelude::Motor};

use super::ExitConditionGroup;
use crate::{
    controllers::FeedbackController,
    differential::{chassis::Chassis, pose::Pose},
    tracking::Tracking,
    utils::{
        math::{angle_error, arcade_desaturate, delta_clamp},
        timer::Timer,
    },
};

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
    lateral_controller: Box<dyn FeedbackController<f64>>,
    angular_controller: Box<dyn FeedbackController<f64>>,

    lateral_exit_conditions: ExitConditionGroup<f64>,
    angular_exit_conditions: ExitConditionGroup<f64>,

    b: f64,
}

impl RAMSETEHybridSettings {
    pub fn new(
        lateral_controller: Box<dyn FeedbackController<f64>>,
        angular_controller: Box<dyn FeedbackController<f64>>,
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
        self.lateral_controller.reset();
        self.angular_controller.reset();
        self.lateral_exit_conditions.reset();
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
// Calculate v_d as PID(|e| * sign(cos(e_θ))).
// Then calculate angular_output as PID(e_θ) + b * v_d * e_y * sinc(e_θ).
// Calculate v as |cos(e_θ)| * v_d.

pub enum RAMSETETarget {
    Pose(Pose),
    Point(Vector2<f64>),
}

impl RAMSETETarget {
    pub fn pose(x: f64, y: f64, orientation: f64) -> Self {
        Self::Pose(Pose::new(x, y, orientation))
    }
    pub fn point(x: f64, y: f64) -> Self {
        Self::Point(Vector2::new(x, y))
    }
}

#[bon]
impl<T: Tracking + 'static> Chassis<T> {
    #[builder]
    pub async fn ramsete_hybrid(
        self: Rc<Self>,
        target: RAMSETETarget,
        timeout: Option<Duration>,
        params: Option<RAMSETEHybridParameters>,
        mut settings: Option<RAMSETEHybridSettings>,
        run_async: Option<bool>,
    ) {
        self.motion_handler.wait_for_motions_end().await;
        if self.motion_handler.in_motion() {
            return;
        }
        if run_async.unwrap_or(true) {
            // Spawn vexide task
            vexide::task::spawn({
                let self_clone = self.clone();
                async move {
                    self_clone
                        .ramsete_hybrid()
                        .target(target)
                        .maybe_timeout(timeout)
                        .maybe_params(params)
                        .maybe_settings(settings.clone())
                        .run_async(false)
                        .call()
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
        let mut unwrapped_params = params.unwrap_or(params_ramsete_h!());
        unwrapped_params.min_lateral_speed = unwrapped_params.min_lateral_speed.abs();
        unwrapped_params.max_lateral_speed = unwrapped_params.max_lateral_speed.abs();
        unwrapped_params.max_angular_speed = unwrapped_params.max_angular_speed.abs();
        if unwrapped_params.max_angular_speed < unwrapped_params.min_lateral_speed {
            panic!("Minimum speed may not exceed the maximum.")
        }
        let mut previous_pose = self.pose().await;
        let mut is_near = false; // Possibly use settling logic.
        let mut previous_lateral_output: f64 = 0.0;
        let mut previous_angular_output: f64 = 0.0;
        let mut timer = Timer::new(timeout.unwrap_or(Duration::MAX));
        while !timer.is_done() && self.motion_handler.in_motion() {
            let pose: Pose = self.pose().await;
            if let Some(distance_traveled) = self.distance_traveled.borrow_mut().as_mut() {
                *distance_traveled += pose.distance_to(&previous_pose);
            }
            previous_pose = pose;
            let mut local_error = Pose::from(
                // Similar to ∇×v, by the right hand rule, a negative direction
                // value is a clockwise rotation (negative orientation).
                nalgebra::Rotation3::new(Vector3::<f64>::new(0.0, 0.0, -pose.orientation))
                    * <Pose as Into<Vector3<f64>>>::into(
                        match target {
                            RAMSETETarget::Point(point) => Pose::new(point.x, point.y, {
                                let error = point - pose.position;
                                error.y.atan2(error.x)
                            }),
                            RAMSETETarget::Pose(pose) => pose,
                        } - pose,
                    ),
            );
            if !is_near && local_error.position.norm() < 5.0 {
                is_near = true;
                unwrapped_params.max_lateral_speed = previous_lateral_output.max(0.5);
            }
            local_error.orientation = angle_error(
                local_error.orientation,
                if unwrapped_params.forwards { 0.0 } else { PI },
                true,
                None,
            );

            if if let Some(settings) = &mut settings {
                let lateral_done = settings
                    .lateral_exit_conditions
                    .update_all(local_error.position.norm());
                let angular_done = settings
                    .angular_exit_conditions
                    .update_all(local_error.orientation);
                lateral_done && angular_done
            } else {
                let mut motion_settings = self.motion_settings.boomerang_settings.borrow_mut();
                let lateral_done = motion_settings
                    .lateral_exit_conditions
                    .update_all(local_error.position.norm());
                let angular_done = motion_settings
                    .angular_exit_conditions
                    .update_all(local_error.orientation);
                lateral_done && angular_done
            } && is_near
            {
                break;
            }

            let v_d = {
                let controller_input =
                    local_error.position.norm() * local_error.orientation.cos().signum();
                if let Some(settings) = &mut settings {
                    settings.lateral_controller.update(controller_input)
                } else {
                    self.motion_settings
                        .ramsete_hybrid_settings
                        .borrow_mut()
                        .lateral_controller
                        .update(controller_input)
                }
            };
            let lateral_output = (v_d * local_error.orientation.cos().abs()).clamp(
                -unwrapped_params.max_lateral_speed,
                unwrapped_params.max_lateral_speed,
            );
            let lateral_output = {
                let check_1_result = if (-unwrapped_params.min_lateral_speed
                    ..unwrapped_params.min_lateral_speed)
                    .contains(&lateral_output)
                {
                    unwrapped_params.min_lateral_speed
                } else {
                    lateral_output
                };
                let check_2_result = if check_1_result < 0.0 && !is_near {
                    0.0
                } else {
                    check_1_result
                };
                delta_clamp(
                    check_2_result,
                    previous_lateral_output,
                    unwrapped_params.lateral_slew.unwrap_or(0.0),
                    None,
                )
            };
            let angular_output = ({
                if let Some(settings) = &mut settings {
                    settings.angular_controller.update(local_error.orientation)
                } else {
                    self.motion_settings
                        .ramsete_hybrid_settings
                        .borrow_mut()
                        .angular_controller
                        .update(local_error.orientation)
                }
            } + v_d
                * local_error.position.y
                * {
                    use nalgebra::ComplexField;
                    local_error.orientation.sinc()
                }
                * {
                    unwrapped_params
                        .b
                        .unwrap_or(if let Some(settings) = &settings {
                            settings.b
                        } else {
                            self.motion_settings.ramsete_hybrid_settings.borrow().b
                        })
                })
            .clamp(
                -unwrapped_params.max_angular_speed,
                unwrapped_params.max_angular_speed,
            );
            let angular_output = delta_clamp(
                angular_output,
                previous_angular_output,
                unwrapped_params.angular_slew.unwrap_or(0.0),
                None,
            );
            previous_lateral_output = lateral_output;
            previous_angular_output = angular_output;

            let (left, right) = arcade_desaturate(
                if unwrapped_params.forwards {
                    lateral_output
                } else {
                    -lateral_output
                },
                angular_output,
            );
            {
                self.drivetrain
                    .left_motors
                    .borrow_mut()
                    .set_velocity_percentage_all(left);
                self.drivetrain
                    .right_motors
                    .borrow_mut()
                    .set_velocity_percentage_all(right);
            }

            vexide::time::sleep(Motor::WRITE_INTERVAL).await;
        }
        self.motion_handler.end_motion().await;
    }
}
