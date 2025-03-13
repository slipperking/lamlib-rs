use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use nalgebra::Vector3;
use vexide::{prelude::Motor, sync::Mutex};

use super::{
    drive_curve::ExponentialDriveCurve,
    motions::{boomerang::BoomerangSettings, MotionHandler},
    pose::Pose,
};
use crate::{devices::motor_group::MotorGroup, tracking::*};

pub struct Drivetrain {
    pub(super) left_motors: Rc<RefCell<MotorGroup>>,
    pub(super) right_motors: Rc<RefCell<MotorGroup>>,
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
    pub boomerang_settings: RefCell<BoomerangSettings>,
}

impl MotionSettings {
    pub fn new(boomerang_settings: RefCell<BoomerangSettings>) -> Self {
        Self { boomerang_settings }
    }
}

pub struct Chassis<T: Tracking> {
    pub(crate) drivetrain: Rc<Drivetrain>,
    pub(super) tracking: Rc<Mutex<T>>,
    pub(super) throttle_curve: ExponentialDriveCurve,
    pub(super) steer_curve: ExponentialDriveCurve,
    pub(super) motion_handler: MotionHandler,
    pub(super) motion_settings: MotionSettings,
    pub(super) distance_traveled: RefCell<Option<f64>>,
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
            distance_traveled: RefCell::new(None),
            motion_settings,
        }
    }
    pub async fn calibrate(&self) {
        self.tracking.lock().await.init(self.tracking.clone()).await;
    }

    pub async fn wait_until(&self, distance: f64) {
        loop {
            vexide::time::sleep(Duration::from_millis(10)).await;
            if match *self.distance_traveled.borrow() {
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
            if self.distance_traveled.borrow().is_none() {
                break;
            }
        }
    }

    pub async fn set_pose(&self, pose: Pose) {
        self.tracking
            .lock()
            .await
            .set_position(&Vector3::from(pose))
            .await;
    }
    pub async fn pose(&self) -> Pose {
        let mut tracking_lock = self.tracking.lock().await;
        Pose::from(tracking_lock.position())
    }
    pub fn arcade(&self, mut throttle: f64, mut steer: f64, use_drive_curve: bool) {
        if use_drive_curve {
            throttle = self.throttle_curve.update(throttle, 1.0);
            steer = self.steer_curve.update(steer, 1.0);
        }
        self.drivetrain
            .left_motors
            .borrow_mut()
            .set_voltage_all_for_types(
                (throttle + steer) * Motor::V5_MAX_VOLTAGE,
                (throttle + steer) * Motor::EXP_MAX_VOLTAGE,
            );
        self.drivetrain
            .right_motors
            .borrow_mut()
            .set_voltage_all_for_types(
                (throttle - steer) * Motor::V5_MAX_VOLTAGE,
                (throttle - steer) * Motor::EXP_MAX_VOLTAGE,
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
