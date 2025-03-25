use alloc::{boxed::Box, rc::Rc};
use core::{f64::consts::PI, time::Duration};

use bon::{bon, Builder};
use num_traits::AsPrimitive;
use vexide::prelude::{BrakeMode, Float};

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

#[derive(Clone, Copy, PartialEq, Builder)]
pub struct BoomerangParameters {
    #[builder(default = true)]
    pub forwards: bool,

    #[builder(default = 0.6)]
    pub lead: f64,

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
    pub horizontal_drift_compensation: Option<f64>,
}
#[derive(Clone)]
pub struct BoomerangSettings {
    pub linear_controller: Box<dyn FeedbackController<f64>>,
    pub angular_controller: Box<dyn FeedbackController<f64>>,

    pub linear_exit_conditions: ExitConditionGroup<f64>,
    pub angular_exit_conditions: ExitConditionGroup<f64>,
}
impl BoomerangSettings {
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
macro_rules! params_boomerang {
    (
        $($key:ident : $value:expr),* $(,)?
    ) => {
        $crate::differential::motions::boomerang::BoomerangParameters::builder()
            $(.$key($value))*
            .build()
    };
}
pub use params_boomerang;

pub struct MoveToPoseTarget(pub Pose);

impl From<Pose> for MoveToPoseTarget {
    fn from(pose: Pose) -> Self {
        Self(pose)
    }
}

impl From<MoveToPoseTarget> for Pose {
    fn from(target: MoveToPoseTarget) -> Self {
        target.0
    }
}

impl<T: AsPrimitive<f64>, U: AsPrimitive<f64>, V: AsPrimitive<f64>> From<(T, U, V)>
    for MoveToPoseTarget
{
    fn from((x, y, orientation): (T, U, V)) -> Self {
        Self(Pose::new(x.as_(), y.as_(), orientation.as_()))
    }
}

#[bon]
impl<T: Tracking + 'static> Chassis<T> {
    #[builder]
    pub async fn boomerang(
        self: Rc<Self>,
        target: impl Into<MoveToPoseTarget> + 'static,
        timeout: Option<Duration>,
        params: Option<BoomerangParameters>,
        mut settings: Option<BoomerangSettings>,
        run_async: Option<bool>,
    ) {
        let mut unwrapped_params = params.unwrap_or(params_boomerang!());
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
                        .boomerang()
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
        if let Some(ref mut settings) = settings {
            settings.reset();
        } else {
            self.motion_settings.boomerang_settings.borrow_mut().reset();
        }
        let mut previous_pose = self.pose().await;
        {
            *self.distance_traveled.borrow_mut() = Some(0.0);
        }
        let target = target.into().0;
        let mut is_near = false;
        let mut previous_was_same_side = false;
        let mut previous_linear_output: f64 = 0.0;
        let mut previous_angular_output: f64 = 0.0;
        let mut timer = Timer::new(timeout.unwrap_or(Duration::MAX));
        while !timer.is_done() && self.motion_handler.is_in_motion() {
            let pose = self.pose().await;
            if let Some(distance) = self.distance_traveled.borrow_mut().as_mut() {
                *distance += pose.distance_to(&previous_pose);
            }
            let distance_to_target = pose.distance_to(&target);
            previous_pose = pose;
            if distance_to_target < 7.5 && !is_near {
                is_near = true;
                unwrapped_params.max_linear_speed = previous_linear_output.max(0.5);
            }
            let robot_vectors_normalized =
                nalgebra::Vector2::<f64>::new(target.orientation.cos(), target.orientation.sin());
            let carrot_point = if is_near {
                target.position
            } else {
                target.position
                    - robot_vectors_normalized * unwrapped_params.lead * distance_to_target
            };
            let carrot_pose_angle =
                (carrot_point.y - pose.position.y).atan2(carrot_point.x - pose.position.x);
            let linear_error = {
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
                    angle_error(adjusted_orientation, target.orientation, true, None)
                } else {
                    angle_error(adjusted_orientation, carrot_pose_angle, true, None)
                }
            };

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
            {
                let is_robot_side: bool = (pose.position.y - target.position.y)
                    * -target.orientation.sin()
                    <= (pose.position.x - target.position.x) * target.orientation.cos()
                        + unwrapped_params.early_exit_range;
                let is_carrot_side: bool = (carrot_point.y - target.position.y)
                    * -target.orientation.sin()
                    <= (carrot_point.x - target.position.x) * target.orientation.cos()
                        + unwrapped_params.early_exit_range;
                let is_same_side: bool = is_robot_side == is_carrot_side;

                if !is_same_side
                    && previous_was_same_side
                    && is_near
                    && unwrapped_params.min_linear_speed != 0.0
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
            let linear_output = {
                let mut motion_settings = self.motion_settings.boomerang_settings.borrow_mut();
                let mut raw_output = if let Some(settings) = &mut settings {
                    settings
                } else {
                    &mut motion_settings
                }
                .linear_controller
                .update(linear_error)
                .clamp(
                    -unwrapped_params.max_linear_speed,
                    unwrapped_params.max_linear_speed,
                );
                if !is_near {
                    raw_output = delta_clamp(
                        raw_output,
                        previous_linear_output,
                        unwrapped_params.linear_slew.unwrap_or(0.0),
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

                // Prevent moving in the wrong direction.
                if !unwrapped_params.forwards && !is_near {
                    raw_output = raw_output.min(0.0);
                } else if unwrapped_params.forwards && !is_near {
                    raw_output = raw_output.max(0.0);
                }

                if !unwrapped_params.forwards
                    && -raw_output < unwrapped_params.min_linear_speed.abs()
                    && raw_output < 0.0
                {
                    raw_output = -unwrapped_params.min_linear_speed.abs();
                } else if unwrapped_params.forwards
                    && raw_output < unwrapped_params.min_linear_speed.abs()
                    && raw_output > 0.0
                {
                    raw_output = unwrapped_params.min_linear_speed.abs();
                }
                previous_linear_output = raw_output;
                raw_output
            };
            let (left, right) = arcade_desaturate(linear_output, angular_output);
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
