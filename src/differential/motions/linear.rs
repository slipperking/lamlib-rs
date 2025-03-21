use alloc::{boxed::Box, rc::Rc};
use core::{f64::consts::PI, time::Duration};

use bon::{bon, Builder};
use nalgebra::Vector2;
use vexide::prelude::{Float, Motor};

use super::ExitConditionGroup;
use crate::{
    controllers::FeedbackController,
    differential::chassis::Chassis,
    tracking::Tracking,
    utils::{
        math::{arcade_desaturate, delta_clamp},
        timer::Timer,
    },
};

#[derive(Clone, Copy, PartialEq, Builder)]
pub struct MoveToPointParameters {
    #[builder(default = true)]
    pub forwards: bool,

    #[builder(default = 0.0)]
    pub min_linear_speed: f64,

    #[builder(default = 1.0)]
    pub max_linear_speed: f64,

    #[builder(default = 1.0)]
    pub max_angular_speed: f64,

    #[builder(default = 0.0)]
    pub early_exit_range: f64,

    pub linear_slew: Option<f64>,
    pub angular_slew: Option<f64>,
}

#[derive(Clone)]
pub struct MoveToPointSettings {
    linear_controller: Box<dyn FeedbackController<f64>>,
    angular_controller: Box<dyn FeedbackController<f64>>,

    linear_exit_conditions: ExitConditionGroup<f64>,
    angular_exit_conditions: ExitConditionGroup<f64>,
}

impl MoveToPointSettings {
    pub fn new(
        linear_controller: Box<dyn FeedbackController<f64>>,
        angular_controller: Box<dyn FeedbackController<f64>>,
        linear_exit_conditions: ExitConditionGroup<f64>,
        angular_exit_conditions: ExitConditionGroup<f64>,
    ) -> Self {
        Self {
            linear_controller,
            angular_controller,
            linear_exit_conditions,
            angular_exit_conditions,
        }
    }

    pub fn reset(&mut self) {
        self.linear_controller.reset();
        self.angular_controller.reset();
        self.linear_exit_conditions.reset();
        self.angular_exit_conditions.reset();
    }
}

#[macro_export]
macro_rules! params_move_to_point {
    (
        $($key:ident : $value:expr),* $(,)?
    ) => {
        $crate::differential::motions::linear::MoveToPointParameters::builder()
            $(.$key($value))*
            .build()
    };
}
pub use params_move_to_point;
#[derive(Clone, Copy, PartialEq, Builder)]
pub struct MoveRelativeParameters {
    #[builder(default = 0.0)]
    pub min_linear_speed: f64,

    #[builder(default = 1.0)]
    pub max_linear_speed: f64,

    #[builder(default = 0.0)]
    pub early_exit_range: f64,
    pub linear_slew: Option<f64>,
}

type MoveRelativeSettings = MoveToPointSettings;

#[bon]
impl<T: Tracking + 'static> Chassis<T> {
    #[builder]
    pub async fn move_to_point(
        self: Rc<Self>,
        target: Vector2<f64>,
        timeout: Option<Duration>,
        params: Option<MoveToPointParameters>,
        mut settings: Option<MoveToPointSettings>,
        run_async: Option<bool>,
    ) {
        self.motion_handler.wait_for_motions_end().await;
        if self.motion_handler.is_in_motion() {
            return;
        }
        if run_async.unwrap_or(true) {
            // Spawn vexide task
            vexide::task::spawn({
                let self_clone = self.clone();
                async move {
                    self_clone
                        .move_to_point()
                        .target(target)
                        .maybe_timeout(timeout)
                        .maybe_params(params)
                        .maybe_settings(settings)
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
                .move_to_point_settings
                .borrow_mut()
                .reset();
        }
        {
            *self.distance_traveled.borrow_mut() = Some(0.0);
        }
        let mut unwrapped_params = params.unwrap_or(params_move_to_point!());
        let mut previous_pose = self.pose().await;
        let mut is_near: bool = false;
        let mut previous_linear_output: f64 = 0.0;
        let mut previous_angular_output: f64 = 0.0;

        let mut timer = Timer::new(timeout.unwrap_or(Duration::MAX));
        while !timer.is_done() && self.motion_handler.is_in_motion() {
            let pose = self.pose().await;
            if let Some(distance) = self.distance_traveled.borrow_mut().as_mut() {
                *distance += pose.distance_to(&previous_pose);
            }
            previous_pose = pose;
            let local_error = nalgebra::Rotation2::new(
                -pose.orientation + if unwrapped_params.forwards { 0.0 } else { PI },
            ) * (target - pose.position);
            let linear_error = local_error.norm();
            let cosine_linear_error = local_error.x;
            let angular_error = local_error.y.atan2(local_error.x);
            if local_error.norm() < 5.0 && !is_near {
                unwrapped_params.max_linear_speed = 0.6;
                is_near = true;
            }

            if linear_error.abs() < unwrapped_params.early_exit_range {
                break;
            }

            if if let Some(settings) = &mut settings {
                let linear_done = settings.linear_exit_conditions.update_all(linear_error);
                let angular_done = settings.angular_exit_conditions.update_all(angular_error);
                linear_done && angular_done
            } else {
                let mut motion_settings = self.motion_settings.boomerang_settings.borrow_mut();
                let linear_done = motion_settings
                    .linear_exit_conditions
                    .update_all(linear_error);
                let angular_done = motion_settings
                    .angular_exit_conditions
                    .update_all(angular_error);
                linear_done && angular_done
            } && is_near
            {
                break;
            }
            let linear_output = {
                if let Some(settings) = &mut settings {
                    settings.linear_controller.update(cosine_linear_error)
                } else {
                    self.motion_settings
                        .move_to_point_settings
                        .borrow_mut()
                        .linear_controller
                        .update(cosine_linear_error)
                }
            }
            .clamp(
                -unwrapped_params.max_linear_speed,
                unwrapped_params.max_linear_speed,
            );

            let linear_output = {
                let check_1_result = if (-unwrapped_params.min_linear_speed
                    ..unwrapped_params.min_linear_speed)
                    .contains(&linear_output)
                {
                    unwrapped_params.min_linear_speed
                } else {
                    linear_output
                };
                let check_2_result = if check_1_result < 0.0 && !is_near {
                    0.0
                } else {
                    check_1_result
                };
                delta_clamp(
                    check_2_result,
                    previous_linear_output,
                    unwrapped_params.linear_slew.unwrap_or(0.0),
                    None,
                )
            };

            let angular_output = {
                if let Some(settings) = &mut settings {
                    settings.linear_controller.update(angular_error)
                } else {
                    self.motion_settings
                        .move_to_point_settings
                        .borrow_mut()
                        .linear_controller
                        .update(angular_error)
                }
            }
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

            previous_linear_output = linear_output;
            previous_angular_output = angular_output;

            let (left, right) = arcade_desaturate(
                if unwrapped_params.forwards {
                    linear_output
                } else {
                    -linear_output
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

    #[builder]
    pub async fn move_relative(
        self: Rc<Self>,
        distance: f64,
        timeout: Option<Duration>,
        params: Option<MoveRelativeParameters>,
        settings: Option<MoveRelativeSettings>,
        run_async: Option<bool>,
    ) {
        // Wait until the motion is done. before calculating angles.
        if self.motion_handler.is_in_motion() {
            self.wait_until_complete().await;
        }
        let pose = self.pose().await;
        let displacement_vector =
            nalgebra::Rotation2::new(pose.orientation) * Vector2::new(distance, 0.0);
        let move_to_point_params = match params {
            Some(params) => MoveToPointParameters {
                forwards: distance > 0.0,
                min_linear_speed: params.min_linear_speed,
                max_linear_speed: params.max_linear_speed,
                max_angular_speed: 1.0,
                linear_slew: params.linear_slew,
                angular_slew: None,
                early_exit_range: params.early_exit_range,
            },
            None => {
                params_move_to_point!(forwards: distance > 0.0,)
            }
        };
        self.move_to_point()
            .target(pose.position + displacement_vector)
            .maybe_timeout(timeout)
            .params(move_to_point_params)
            .maybe_settings(settings)
            .maybe_run_async(run_async)
            .call()
            .await;
    }
}
