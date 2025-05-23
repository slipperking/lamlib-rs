use alloc::{boxed::Box, rc::Rc};
use core::{
    f64::consts::{FRAC_PI_2, PI},
    time::Duration,
};

use bon::{bon, Builder};
use nalgebra::Vector2;
use num_traits::AsPrimitive;
use vexide::prelude::{BrakeMode, Float, Motor, MotorControl};

use super::ToleranceGroup;
use crate::{
    controllers::FeedbackController,
    differential::{chassis::Chassis, pose::Pose},
    tracking::Tracking,
    utils::{
        math::{angle_error, arcade_desaturate, delta_clamp, AngularDirection},
        timer::Timer,
    },
};

#[derive(Clone, Copy, PartialEq)]
pub enum DifferentialDriveSide {
    Left,
    Right,
}

#[derive(Clone, Copy, PartialEq, Builder)]
pub struct TurnToParameters {
    #[builder(default = true)]
    pub forwards: bool,

    /// Speed constraints here restrict both drive sides.
    #[builder(default = 0.0)]
    pub min_speed: f64,

    #[builder(default = 1.0)]
    pub max_speed: f64,

    #[builder(default = 0.0)]
    pub early_exit_range: f64,
    pub angular_slew: Option<f64>,
    pub direction: Option<AngularDirection>,

    /// The locked side for a swing turn.
    pub locked_side: Option<DifferentialDriveSide>,
}

#[derive(Clone)]
pub struct TurnToSettings {
    pub angular_controller: Box<dyn FeedbackController<f64>>,
    pub swing_controller: Box<dyn FeedbackController<f64>>,

    pub angular_tolerances: ToleranceGroup<f64>,
}

impl TurnToSettings {
    pub fn new(
        angular_controller: Box<dyn FeedbackController<f64>>,
        swing_controller: Box<dyn FeedbackController<f64>>,
        angular_tolerances: ToleranceGroup<f64>,
    ) -> Self {
        Self {
            angular_controller,
            swing_controller,
            angular_tolerances,
        }
    }

    pub fn reset(&mut self) {
        self.angular_controller.reset();
        self.swing_controller.reset();
        self.angular_tolerances.reset();
    }
}

#[macro_export]
macro_rules! params_turn_to {
    (
        $($key:ident : $value:expr),* $(,)?
    ) => {
        $crate::differential::motions::angular::TurnToParameters::builder()
            $(.$key($value))*
            .build()
    };
}
pub use params_turn_to;

#[derive(Clone, PartialEq, Debug)]
pub enum TurnToTarget {
    /// A point the robot can turn to face.
    /// This is in inches.
    Point(Vector2<f64>),

    /// An angle the robot can turn to.
    ///
    /// # Example
    /// ```
    /// TurnToTarget::Angle(angle!(degrees: 90.0, standard: false,))
    /// ```
    Angle(f64),
}

impl TurnToTarget {
    pub fn point<T: AsPrimitive<f64>, U: AsPrimitive<f64>>(x: T, y: U) -> Self {
        Self::Point(Vector2::new(x.as_(), y.as_()))
    }

    /// The angle error between a pose and a target.
    pub fn error(&self, pose: Pose, direction: Option<AngularDirection>) -> f64 {
        match *self {
            Self::Point(point) => {
                let delta_position = point - pose.position;
                crate::utils::math::angle_error(
                    f64::atan2(delta_position.y, delta_position.x),
                    pose.orientation,
                    true,
                    direction,
                )
            }
            Self::Angle(angle) => {
                crate::utils::math::angle_error(angle, pose.orientation, true, direction)
            }
        }
    }
}

macro_rules! scalar_turn_to_impl {
    ($($t:ty),*) => {
        $(
            impl From<$t> for TurnToTarget {
                fn from(angle: $t) -> Self {
                    Self::Angle(angle as f64)
                }
            }
        )*
    };
}

scalar_turn_to_impl!(f64, f32, i32);

impl<T: AsPrimitive<f64>, U: AsPrimitive<f64>> From<(T, U)> for TurnToTarget {
    fn from((x, y): (T, U)) -> Self {
        Self::point(x.as_(), y.as_())
    }
}

#[bon]
impl<T: Tracking + 'static> Chassis<T> {
    #[builder]
    pub async fn turn_to(
        self: Rc<Self>,
        target: impl Into<TurnToTarget> + 'static,
        timeout: Option<Duration>,
        params: Option<TurnToParameters>,
        mut settings: Option<TurnToSettings>,
        run_async: Option<bool>,
    ) {
        let unwrapped_params = params.unwrap_or(params_turn_to!());
        self.motion_handler.wait_for_motions_end().await;
        if !self.motion_handler.is_in_motion() {
            return;
        }
        if run_async.unwrap_or(true) {
            // Spawn vexide task
            vexide::task::spawn({
                let self_clone = self.clone();
                async move {
                    self_clone
                        .turn_to()
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
            self.motion_settings.turn_to_settings.borrow_mut().reset();
        }
        *self.distance_traveled.borrow_mut() = Some(0.0);
        match unwrapped_params.locked_side {
            Some(DifferentialDriveSide::Left) => {
                self.drivetrain
                    .left_motors
                    .borrow_mut()
                    .set_target_all(MotorControl::Brake(BrakeMode::Brake));
            }
            Some(DifferentialDriveSide::Right) => {
                self.drivetrain
                    .right_motors
                    .borrow_mut()
                    .set_target_all(MotorControl::Brake(BrakeMode::Brake));
            }
            None => {}
        };
        let target: TurnToTarget = target.into();
        let mut previous_pose = self.pose().await;
        let mut previous_raw_error: Option<f64> = None;
        let mut oscillations_begin = false;
        let mut previous_output: f64 = 0.0;
        let mut timer = Timer::new(timeout.unwrap_or(Duration::MAX));

        while !timer.is_done() && self.motion_handler.is_in_motion() {
            let pose = self.pose().await;
            if let Some(distance) = self.distance_traveled.borrow_mut().as_mut() {
                *distance +=
                    (angle_error(pose.orientation, previous_pose.orientation, true, None)).abs();
            }
            previous_pose = pose;

            let raw_error = target.error(
                pose + Pose::new(0.0, 0.0, if unwrapped_params.forwards { 0.0 } else { PI }),
                None,
            );

            // If it crosses error being 0, then drop the `direction` parameter.
            if !oscillations_begin
                && previous_raw_error.unwrap().signum() != raw_error.signum()
                // Make sure the crossing is actually on low error:
                && (0.0..FRAC_PI_2).contains(&previous_raw_error.unwrap_or(raw_error).abs())
                && (0.0..FRAC_PI_2).contains(&raw_error.abs())
            {
                oscillations_begin = true;
            }

            previous_raw_error = Some(raw_error);

            if unwrapped_params.min_speed != 0.0 && oscillations_begin
                || raw_error.abs() < unwrapped_params.early_exit_range
            {
                break;
            }
            let error = if oscillations_begin {
                raw_error
            } else {
                target.error(
                    pose + Pose::new(0.0, 0.0, if unwrapped_params.forwards { 0.0 } else { PI }),
                    unwrapped_params.direction,
                )
            };
            if if let Some(settings) = &mut settings {
                settings.angular_tolerances.update_all(error)
            } else {
                self.motion_settings
                    .boomerang_settings
                    .borrow_mut()
                    .angular_tolerances
                    .update_all(error)
            } {
                break;
            }

            // Apply controllers.
            let mut raw_output = if unwrapped_params.locked_side.is_some() {
                if let Some(settings) = &mut settings {
                    settings.swing_controller.update(error, 0.0)
                } else {
                    self.motion_settings
                        .turn_to_settings
                        .borrow_mut()
                        .swing_controller
                        .update(error, 0.0)
                }
            } else if let Some(settings) = &mut settings {
                settings.angular_controller.update(error, 0.0)
            } else {
                self.motion_settings
                    .turn_to_settings
                    .borrow_mut()
                    .angular_controller
                    .update(error, 0.0)
            }
            .clamp(-unwrapped_params.max_speed, unwrapped_params.max_speed);
            raw_output = delta_clamp(
                raw_output,
                previous_output,
                unwrapped_params.angular_slew.unwrap_or(0.0),
                None,
            );
            if (-unwrapped_params.min_speed..0.0).contains(&raw_output) {
                raw_output = -unwrapped_params.min_speed;
            } else if (0.0..unwrapped_params.min_speed).contains(&raw_output) {
                raw_output = unwrapped_params.min_speed;
            }
            previous_output = raw_output;
            let (left, right) = arcade_desaturate(
                if let Some(locked_side) = unwrapped_params.locked_side {
                    match locked_side {
                        DifferentialDriveSide::Left => raw_output,
                        DifferentialDriveSide::Right => -raw_output,
                    }
                } else {
                    0.0
                },
                raw_output,
            );
            self.drivetrain
                .left_motors
                .borrow_mut()
                .set_velocity_percentage_all(left);
            self.drivetrain
                .right_motors
                .borrow_mut()
                .set_velocity_percentage_all(right);

            vexide::time::sleep(Motor::WRITE_INTERVAL).await;
        }
        self.drivetrain
            .left_motors
            .borrow_mut()
            .set_target_all(vexide::prelude::MotorControl::Brake(BrakeMode::Coast));
        self.drivetrain
            .right_motors
            .borrow_mut()
            .set_target_all(vexide::prelude::MotorControl::Brake(BrakeMode::Coast));
        *self.distance_traveled.borrow_mut() = None;
        self.motion_handler.end_motion().await;
    }
}
