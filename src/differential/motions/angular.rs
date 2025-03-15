use alloc::{boxed::Box, rc::Rc};
use core::{
    f64::consts::{FRAC_PI_2, PI},
    time::Duration,
};

use nalgebra::Vector2;
use vexide::prelude::{BrakeMode, Float, Motor, MotorControl};

use super::ExitConditionGroup;
use crate::{
    controllers::ControllerMethod,
    differential::{chassis::Chassis, pose::Pose},
    tracking::Tracking,
    utils::{
        math::{angle_error, arcade_desaturate, delta_clamp, AngularDirection},
        timer::Timer,
    },
};

#[derive(Clone, Copy, PartialEq)]
pub enum DriveSide {
    Left,
    Right,
}

#[derive(Clone, Copy, PartialEq)]
pub struct TurnToParameters {
    pub forwards: bool,

    /// Speed constraints here restrict both drive sides.
    pub min_speed: f64,
    pub max_speed: f64,
    pub early_exit_range: f64,
    pub angular_slew: Option<f64>,
    pub direction: Option<AngularDirection>,

    /// The locked side for a swing turn.
    pub locked_side: Option<DriveSide>,
}

#[derive(Clone)]
pub struct TurnToSettings {
    pub angular_controller: Box<dyn ControllerMethod<f64>>,
    pub swing_controller: Box<dyn ControllerMethod<f64>>,

    pub angular_exit_conditions: ExitConditionGroup<f64>,
}
impl TurnToSettings {
    pub fn new(
        angular_controller: Box<dyn ControllerMethod<f64>>,
        swing_controller: Box<dyn ControllerMethod<f64>>,
        angular_exit_conditions: ExitConditionGroup<f64>,
    ) -> Self {
        Self {
            angular_controller,
            swing_controller,
            angular_exit_conditions,
        }
    }

    pub fn reset(&mut self) {
        self.angular_controller.reset();
        self.angular_exit_conditions.reset();
    }
}

#[macro_export]
macro_rules! params_swing {
    (
        locked_side: $locked_side:expr,
        $(forwards: $forwards:expr,)?
        $(min_speed: $min_speed:expr,)?
        $(max_speed: $max_speed:expr,)?
        $(direction: $direction:expr,)?
        $(early_exit_range: $early_exit_range:expr,)?
        $(slew: $angular_slew:expr,)?
    ) => {
        #[allow(unused_mut, unused_assignments)]
        {
            let mut locked_side = Some($locked_side);
            let mut forwards = true;
            let mut min_speed = 0.0;
            let mut max_speed = 1.0;
            let mut early_exit_range = 0.0;
            let mut direction = None;
            let mut angular_slew = None;

            $(forwards = $forwards;)?
            $(min_speed = $min_speed;)?
            $(max_speed = $max_speed;)?
            $(direction = Some($direction);)?
            $(early_exit_range = $early_exit_range;)?
            $(angular_slew = Some($angular_slew);)?

            $crate::differential::motions::angular::TurnToParameters {
                forwards,
                min_speed,
                max_speed,
                early_exit_range,
                angular_slew,
                locked_side,
                direction,
            }
        }
    }
}
pub use params_swing;

#[macro_export]
macro_rules! params_turn_to {
    (
        $(forwards: $forwards:expr,)?
        $(min_speed: $min_speed:expr,)?
        $(max_speed: $max_speed:expr,)?
        $(direction: $direction:expr,)?
        $(early_exit_range: $early_exit_range:expr,)?
        $(slew: $angular_slew:expr,)?
        $(locked_side: $locked_side:expr,)?
    ) => {
        #[allow(unused_mut, unused_assignments)]
        {
            let mut locked_side = None;
            let mut forwards = true;
            let mut min_speed = 0.0;
            let mut max_speed = 1.0;
            let mut early_exit_range = 0.0;
            let mut direction = None;
            let mut angular_slew = None;

            $(forwards = $forwards;)?
            $(min_speed = $min_speed;)?
            $(max_speed = $max_speed;)?
            $(direction = Some($direction);)?
            $(early_exit_range = $early_exit_range;)?
            $(angular_slew = $angular_slew;)?
            $(locked_side = $locked_side;)?

            $crate::differential::motions::angular::TurnToParameters {
                forwards,
                min_speed,
                max_speed,
                early_exit_range,
                angular_slew,
                locked_side,
                direction,
            }
        }
    }
}
pub use params_turn_to;

pub enum TurnTarget {
    /// A point the robot can turn to face.
    /// This is in inches.
    Point(Vector2<f64>),

    /// An angle the robot can turn to.
    ///
    /// # Example
    /// ```
    /// TurnTarget::Angle(angle!(degrees: 90.0, standard: false,))
    /// ```
    Angle(f64),
}

impl TurnTarget {
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
impl<T: Tracking + 'static> Chassis<T> {
    async fn turn_to(
        self: Rc<Self>,
        turn_target: TurnTarget,
        timeout: Option<Duration>,
        params: Option<TurnToParameters>,
        mut settings: Option<TurnToSettings>,
        run_async: bool,
    ) {
        let unwrapped_params = params.unwrap_or(params_turn_to!());
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
                        .turn_to(turn_target, timeout, params, settings.clone(), false)
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
        {
            *self.distance_traveled.borrow_mut() = Some(0.0);
        }
        match unwrapped_params.locked_side {
            Some(DriveSide::Left) => {
                self.drivetrain
                    .left_motors
                    .borrow_mut()
                    .set_target_all(MotorControl::Brake(BrakeMode::Brake));
            }
            Some(DriveSide::Right) => {
                self.drivetrain
                    .right_motors
                    .borrow_mut()
                    .set_target_all(MotorControl::Brake(BrakeMode::Brake));
            }
            None => {}
        };
        let mut previous_pose = self.pose().await;
        let mut previous_raw_error: Option<f64> = None;
        let mut oscillations_begin = false;
        let mut previous_output: f64 = 0.0;
        let mut timer = Timer::new(timeout.unwrap_or(Duration::MAX));

        while !timer.is_done() {
            let pose = self.pose().await;
            if let Some(distance) = self.distance_traveled.borrow_mut().as_mut() {
                *distance +=
                    (angle_error(pose.orientation, previous_pose.orientation, true, None)).abs();
            }
            previous_pose = pose;

            let raw_error = turn_target.error(
                pose + Pose::new(0.0, 0.0, if unwrapped_params.forwards { 0.0 } else { PI }),
                None,
            );
            if previous_raw_error.is_none() {
                previous_raw_error = Some(raw_error);
            }

            // If it crosses error being 0, then drop the `direction` parameter.
            if !oscillations_begin
                && previous_raw_error.unwrap().signum() != raw_error.signum()
                // Make sure the crossing is actually on low error:
                && (0.0..FRAC_PI_2).contains(&previous_raw_error.unwrap().abs())
                && (0.0..FRAC_PI_2).contains(&raw_error.abs())
            {
                oscillations_begin = true;
            }

            if unwrapped_params.min_speed != 0.0 && oscillations_begin
                || raw_error.abs() < unwrapped_params.early_exit_range
            {
                break;
            }
            let error = if oscillations_begin {
                raw_error
            } else {
                turn_target.error(
                    pose + Pose::new(0.0, 0.0, if unwrapped_params.forwards { 0.0 } else { PI }),
                    unwrapped_params.direction,
                )
            };
            if if let Some(settings) = &mut settings {
                settings.angular_exit_conditions.update_all(error)
            } else {
                self.motion_settings
                    .boomerang_settings
                    .borrow_mut()
                    .angular_exit_conditions
                    .update_all(error)
            } {
                break;
            }

            // Apply controllers.
            let mut raw_output = if unwrapped_params.locked_side.is_some() {
                if let Some(settings) = &mut settings {
                    settings.swing_controller.update(error)
                } else {
                    self.motion_settings
                        .turn_to_settings
                        .borrow_mut()
                        .swing_controller
                        .update(error)
                }
            } else if let Some(settings) = &mut settings {
                settings.angular_controller.update(error)
            } else {
                self.motion_settings
                    .turn_to_settings
                    .borrow_mut()
                    .angular_controller
                    .update(error)
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
                        DriveSide::Left => raw_output,
                        DriveSide::Right => -raw_output,
                    }
                } else {
                    0.0
                },
                raw_output,
            );
            {
                self.drivetrain
                    .left_motors
                    .borrow_mut()
                    .set_voltage_all_for_types(
                        left * Motor::V5_MAX_VOLTAGE,
                        left * Motor::EXP_MAX_VOLTAGE,
                    );
                self.drivetrain
                    .right_motors
                    .borrow_mut()
                    .set_voltage_all_for_types(
                        right * Motor::V5_MAX_VOLTAGE,
                        right * Motor::EXP_MAX_VOLTAGE,
                    );
            }

            vexide::time::sleep(Motor::WRITE_INTERVAL).await;
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
