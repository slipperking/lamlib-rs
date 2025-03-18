use alloc::{boxed::Box, rc::Rc};
use core::{f64::consts::PI, time::Duration};

use vexide::prelude::{BrakeMode, Float};

use super::ExitConditionGroup;
use crate::{
    controllers::ControllerMethod,
    differential::{chassis::Chassis, pose::Pose},
    tracking::Tracking,
    utils::{
        math::{angle_error, arcade_desaturate, delta_clamp},
        timer::Timer,
    },
};

#[derive(Clone, Copy, PartialEq)]
pub struct BoomerangParameters {
    pub forwards: bool,
    pub lead: f64,
    pub min_lateral_speed: f64,
    pub max_lateral_speed: f64,
    pub max_angular_speed: f64,
    pub early_exit_range: f64,
    pub lateral_slew: Option<f64>,
    pub angular_slew: Option<f64>,
    pub horizontal_drift_compensation: Option<f64>,
}
#[derive(Clone)]
pub struct BoomerangSettings {
    pub lateral_controller: Box<dyn ControllerMethod<f64>>,
    pub angular_controller: Box<dyn ControllerMethod<f64>>,

    pub lateral_exit_conditions: ExitConditionGroup<f64>,
    pub angular_exit_conditions: ExitConditionGroup<f64>,
}
impl BoomerangSettings {
    pub fn new(
        lateral_controller: Box<dyn ControllerMethod<f64>>,
        angular_controller: Box<dyn ControllerMethod<f64>>,
        lateral_exit_conditions: ExitConditionGroup<f64>,
        angular_exit_conditions: ExitConditionGroup<f64>,
    ) -> Self {
        Self {
            lateral_controller,
            angular_controller,
            lateral_exit_conditions,
            angular_exit_conditions,
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
macro_rules! params_boomerang {
    (
        $(forwards: $forwards:expr,)?
        $(lead: $lead:expr,)?
        $(min_lateral_speed: $min_lateral_speed:expr,)?
        $(max_lateral_speed: $max_lateral_speed:expr,)?
        $(max_angular_speed: $max_angular_speed:expr,)?
        $(early_exit_range: $early_exit_range:expr,)?
        $(lateral_slew: $lateral_slew:expr,)?
        $(angular_slew: $angular_slew:expr,)?
        $(horizontal_drift_compensation: $horizontal_drift_compensation:expr,)?
    ) => {
        #[allow(unused_mut, unused_assignments)]
        {
            let mut forwards = true;
            let mut lead = 0.6;
            let mut min_lateral_speed = 0.0;
            let mut max_lateral_speed = 1.0;
            let mut max_angular_speed = 1.0;
            let mut early_exit_range = 0.0;
            let mut lateral_slew = None;
            let mut angular_slew = None;
            let mut horizontal_drift_compensation = None;

            $(forwards = $forwards;)?
            $(lead = $lead;)?
            $(min_lateral_speed = $min_lateral_speed;)?
            $(max_lateral_speed = $max_lateral_speed;)?
            $(early_exit_range = $early_exit_range;)?
            $(lateral_slew = Some($lateral_slew);)?
            $(angular_slew = Some($angular_slew);)?
            $(horizontal_drift_compensation = $horizontal_drift_compensation;)?

            $crate::differential::motions::boomerang::BoomerangParameters {
                forwards,
                lead,
                min_lateral_speed,
                max_lateral_speed,
                max_angular_speed,
                early_exit_range,
                lateral_slew,
                angular_slew,
                horizontal_drift_compensation

            }
        }
    }
}
pub use params_boomerang;

impl<T: Tracking + 'static> Chassis<T> {
    pub async fn boomerang(
        self: Rc<Self>,
        boomerang_target: Pose,
        timeout: Option<Duration>,
        params: Option<BoomerangParameters>,
        mut settings: Option<BoomerangSettings>,
        run_async: bool,
    ) {
        let mut unwrapped_params = params.unwrap_or(params_boomerang!());
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
                        .boomerang(boomerang_target, timeout, params, settings.clone(), false)
                        .await
                }
            })
            .detach();
            self.motion_handler.end_motion().await;
            vexide::time::sleep(Duration::from_millis(10)).await;
            return;
        }
        if let Some(ref mut settings) = settings {
            settings.reset();
        } else {
            self.motion_settings.boomerang_settings.borrow_mut().reset();
        }
        let mut previous_pose = self.pose().await;
        {
            *self.distance_traveled.borrow_mut() = Some(0.0);
        }
        let mut is_near = false;
        let mut previous_was_same_side = false;
        let mut previous_lateral_output: f64 = 0.0;
        let mut previous_angular_output: f64 = 0.0;
        let mut timer = Timer::new(timeout.unwrap_or(Duration::MAX));
        while !timer.is_done() && self.motion_handler.in_motion() {
            let pose = self.pose().await;
            {
                if let Some(distance) = self.distance_traveled.borrow_mut().as_mut() {
                    *distance += pose.distance_to(&previous_pose);
                }
            }
            let distance_to_target = pose.distance_to(&boomerang_target);
            previous_pose = pose;
            if distance_to_target < 7.5 && !is_near {
                is_near = true;
                unwrapped_params.max_lateral_speed = previous_lateral_output.max(0.5);
            }
            let robot_vectors_normalized = nalgebra::Vector2::<f64>::new(
                boomerang_target.orientation.cos(),
                boomerang_target.orientation.sin(),
            );
            let carrot_point = if is_near {
                boomerang_target.position
            } else {
                boomerang_target.position
                    - robot_vectors_normalized * unwrapped_params.lead * distance_to_target
            };
            let carrot_pose_angle =
                (carrot_point.y - pose.position.y).atan2(carrot_point.x - pose.position.x);
            let lateral_error = {
                // Find the cosine of the signed angle between.
                let cos_angle_difference =
                    angle_error(pose.orientation, carrot_pose_angle, true, None).cos();
                if is_near {
                    distance_to_target * cos_angle_difference
                } else {
                    distance_to_target * cos_angle_difference.signum()
                }
            };
            let angular_error = {
                let adjusted_orientation = if unwrapped_params.forwards {
                    pose.orientation
                } else {
                    pose.orientation + PI
                };
                // Counterclockwise is positive.
                if is_near {
                    angle_error(
                        adjusted_orientation,
                        boomerang_target.orientation,
                        true,
                        None,
                    )
                } else {
                    angle_error(adjusted_orientation, carrot_pose_angle, true, None)
                }
            };

            if if let Some(settings) = &mut settings {
                let lateral_done = settings.lateral_exit_conditions.update_all(lateral_error);
                let angular_done = settings.angular_exit_conditions.update_all(angular_error);
                lateral_done && angular_done
            } else {
                let mut motion_settings = self.motion_settings.boomerang_settings.borrow_mut();
                let lateral_done = motion_settings
                    .lateral_exit_conditions
                    .update_all(lateral_error);
                let angular_done = motion_settings
                    .angular_exit_conditions
                    .update_all(angular_error);
                lateral_done && angular_done
            } && is_near
            {
                break;
            }
            if false {
                let is_robot_side: bool = (pose.position.y - boomerang_target.position.y)
                    * -boomerang_target.orientation.sin()
                    <= (pose.position.x - boomerang_target.position.x)
                        * boomerang_target.orientation.cos()
                        + unwrapped_params.early_exit_range;
                let is_carrot_side: bool = (carrot_point.y - boomerang_target.position.y)
                    * -boomerang_target.orientation.sin()
                    <= (carrot_point.x - boomerang_target.position.x)
                        * boomerang_target.orientation.cos()
                        + unwrapped_params.early_exit_range;
                let is_same_side: bool = is_robot_side == is_carrot_side;

                if !is_same_side
                    && previous_was_same_side
                    && is_near
                    && unwrapped_params.min_lateral_speed != 0.0
                {
                    break;
                }
                previous_was_same_side = is_same_side;
            }
            let angular_output = {
                let mut motion_settings = self.motion_settings.boomerang_settings.borrow_mut();
                let mut raw_output = if let Some(settings) = &mut settings {
                    settings
                } else {
                    &mut *motion_settings
                }
                .angular_controller
                .update(angular_error)
                .clamp(
                    -unwrapped_params.max_angular_speed,
                    unwrapped_params.max_angular_speed,
                );
                raw_output = delta_clamp(
                    raw_output,
                    previous_angular_output,
                    unwrapped_params.angular_slew.unwrap_or(0.0),
                    None,
                );
                previous_angular_output = raw_output;
                raw_output
            };
            let lateral_output = {
                let mut motion_settings = self.motion_settings.boomerang_settings.borrow_mut();
                let mut raw_output = if let Some(settings) = &mut settings {
                    settings
                } else {
                    &mut motion_settings
                }
                .lateral_controller
                .update(lateral_error)
                .clamp(
                    -unwrapped_params.max_lateral_speed,
                    unwrapped_params.max_lateral_speed,
                );
                if !is_near {
                    raw_output = delta_clamp(
                        raw_output,
                        previous_lateral_output,
                        unwrapped_params.lateral_slew.unwrap_or(0.0),
                        None,
                    )
                }
                // Get radius. Calculate local error to carrot.
                let carrot_error: nalgebra::Vector2<f64> =
                    nalgebra::Rotation2::new(-pose.orientation) * (carrot_point - pose.position);
                let half_arc = carrot_error.y / carrot_error.x.atan().abs();
                let radius = carrot_error.norm() / (2.0 * half_arc.sin());
                if let Some(horizontal_drift_compensation) =
                    unwrapped_params.horizontal_drift_compensation
                {
                    let anti_drift_max_speed = (horizontal_drift_compensation * radius).sqrt();
                    raw_output = raw_output.clamp(-anti_drift_max_speed, anti_drift_max_speed);
                }

                let overturn: f64 =
                    angular_output.abs() + raw_output.abs() - unwrapped_params.max_lateral_speed;
                if overturn > 0.0 {
                    raw_output -= if raw_output > 0.0 {
                        overturn
                    } else {
                        -overturn
                    };
                }
                // Prevent moving in the wrong direction.
                if !unwrapped_params.forwards && !is_near {
                    raw_output = raw_output.min(0.0);
                } else if unwrapped_params.forwards && !is_near {
                    raw_output = raw_output.max(0.0);
                }

                if !unwrapped_params.forwards
                    && -raw_output < unwrapped_params.min_lateral_speed.abs()
                    && raw_output < 0.0
                {
                    raw_output = -unwrapped_params.min_lateral_speed.abs();
                } else if unwrapped_params.forwards
                    && raw_output < unwrapped_params.min_lateral_speed.abs()
                    && raw_output > 0.0
                {
                    raw_output = unwrapped_params.min_lateral_speed.abs();
                }
                previous_lateral_output = raw_output;
                raw_output
            };
            let (left, right) = arcade_desaturate(lateral_output, angular_output);
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
            vexide::time::sleep(vexide::prelude::Motor::WRITE_INTERVAL).await;
        }
        {
            self.drivetrain
                .left_motors
                .borrow_mut()
                .set_target_all(vexide::prelude::MotorControl::Brake(BrakeMode::Coast));
            self.drivetrain
                .right_motors
                .borrow_mut()
                .set_target_all(vexide::prelude::MotorControl::Brake(BrakeMode::Coast));
            *self.distance_traveled.borrow_mut() = None;
        }
        self.motion_handler.end_motion().await;
    }
}
