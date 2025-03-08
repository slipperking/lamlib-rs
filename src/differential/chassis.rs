use alloc::{boxed::Box, rc::Rc};
use core::{cell::RefCell, time::Duration};

use nalgebra::Vector3;
use vexide::{prelude::Motor, sync::Mutex};

use super::{drive_curve::ExponentialDriveCurve, motions::MotionHandler};
use crate::{controllers::ControllerMethod, devices::motor_group::MotorGroup, tracking::*};

pub struct Drivetrain {
    left_motors: Rc<RefCell<MotorGroup>>,
    right_motors: Rc<RefCell<MotorGroup>>,
}

impl Drivetrain {
    pub fn new(
        left_motors: Rc<RefCell<MotorGroup>>,
        right_motors: Rc<RefCell<MotorGroup>>,
    ) -> Self {
        Self {
            left_motors,
            right_motors,
        }
    }
}

pub struct MotionSettings {
    pub lateral_pid: Box<dyn ControllerMethod<f64>>,
    pub angular_pid: Box<dyn ControllerMethod<f64>>,
}

impl MotionSettings {
    pub fn new(
        lateral_pid: Box<dyn ControllerMethod<f64>>,
        angular_pid: Box<dyn ControllerMethod<f64>>,
    ) -> Self {
        Self {
            lateral_pid,
            angular_pid,
        }
    }
}

pub struct Chassis<T: Tracking> {
    pub(super) drivetrain: Rc<Drivetrain>,
    pub(super) tracking: Rc<Mutex<T>>,
    pub(super) throttle_curve: ExponentialDriveCurve,
    pub(super) steer_curve: ExponentialDriveCurve,
    pub(super) motion_handler: MotionHandler,
    pub(super) motion_settings: RefCell<MotionSettings>,
    pub(super) distance_traveled: Option<f64>,
}

impl<T: Tracking> Chassis<T> {
    pub fn new(
        drivetrain: Rc<Drivetrain>,
        tracking: Rc<Mutex<T>>,
        throttle_curve: ExponentialDriveCurve,
        steer_curve: ExponentialDriveCurve,
        motion_settings: MotionSettings,
    ) -> Self {
        Self {
            drivetrain,
            tracking,
            throttle_curve,
            steer_curve,
            motion_handler: MotionHandler::new(),
            distance_traveled: None,
            motion_settings: RefCell::new(motion_settings),
        }
    }
    pub async fn calibrate(&self) {
        self.tracking.lock().await.init(self.tracking.clone()).await;
    }

    pub async fn wait_until(&self, distance: f64) {
        loop {
            vexide::time::sleep(Duration::from_millis(10)).await;
            if match self.distance_traveled {
                Some(distance_traveled) => distance_traveled >= distance,
                None => true,
            } {
                break;
            }
        }
    }
    pub async fn wait_until_complete(&self) {
        loop {
            vexide::time::sleep(Duration::from_millis(10)).await;
            if self.distance_traveled.is_none() {
                break;
            }
        }
    }

    pub async fn set_pose(&self, mut position: Vector3<f64>, radians: bool) {
        position.z = if radians {
            position.z
        } else {
            position.z.to_radians()
        };
        self.tracking.lock().await.set_position(&position).await;
    }
    pub async fn pose(&self, radians: bool) -> Vector3<f64> {
        let mut tracking_lock = self.tracking.lock().await;
        let mut pose = tracking_lock.position();
        if !radians {
            pose.z = pose.z.to_degrees()
        };
        pose
    }

    pub fn arcade(
        &self,
        mut throttle: f64,
        mut steer: f64,
        use_drive_curve: bool,
        throttle_over_steer_prioritization: f64,
    ) {
        throttle *= Motor::V5_MAX_VOLTAGE;
        steer *= Motor::V5_MAX_VOLTAGE;
        if !use_drive_curve {
            throttle = self.throttle_curve.update(throttle, Motor::V5_MAX_VOLTAGE);
            steer = self.steer_curve.update(steer, Motor::V5_MAX_VOLTAGE);
        }
        if throttle.abs() + steer.abs() > Motor::V5_MAX_VOLTAGE {
            let original_throttle = throttle;
            let original_steer = steer;
            throttle *= 1.0
                - throttle_over_steer_prioritization
                    * (original_steer / Motor::V5_MAX_VOLTAGE).abs();
            steer *= 1.0
                - (1.0 - throttle_over_steer_prioritization)
                    * (original_throttle / Motor::V5_MAX_VOLTAGE).abs();

            if steer.abs() + throttle.abs() == Motor::V5_MAX_VOLTAGE {
                if throttle_over_steer_prioritization < 0.5 {
                    throttle += throttle.signum();
                } else {
                    steer += steer.signum();
                }
            }
        }
        self.drivetrain
            .left_motors
            .borrow_mut()
            .set_voltage_all_for_types(
                throttle + steer,
                (throttle + steer) * Motor::EXP_MAX_VOLTAGE / Motor::V5_MAX_VOLTAGE,
            );
        self.drivetrain
            .right_motors
            .borrow_mut()
            .set_voltage_all_for_types(
                throttle - steer,
                (throttle - steer) * Motor::EXP_MAX_VOLTAGE / Motor::V5_MAX_VOLTAGE,
            );
    }

    pub fn tank(&self, mut left: f64, mut right: f64, use_drive_curve: bool) {
        left = if use_drive_curve {
            self.throttle_curve.update(left, Motor::V5_MAX_VOLTAGE)
        } else {
            left
        };
        right = if use_drive_curve {
            self.throttle_curve.update(right, Motor::V5_MAX_VOLTAGE)
        } else {
            right
        };
        self.drivetrain
            .left_motors
            .borrow_mut()
            .set_voltage_all_for_types(left, left * Motor::EXP_MAX_VOLTAGE / Motor::V5_MAX_VOLTAGE);
        self.drivetrain
            .right_motors
            .borrow_mut()
            .set_voltage_all_for_types(
                right,
                right * Motor::EXP_MAX_VOLTAGE / Motor::V5_MAX_VOLTAGE,
            );
    }
}
