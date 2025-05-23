use alloc::{boxed::Box, rc::Rc};
use core::{cell::RefCell, time::Duration};

use nalgebra::Vector3;
use vexide::{prelude::Motor, sync::Mutex};

use super::{
    drive_curve::DriveCurve,
    motions::{
        angular::TurnToSettings, boomerang::BoomerangSettings, linear::MoveToPointSettings,
        ramsete::RAMSETEHybridSettings, MotionHandler,
    },
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
    pub move_to_point_settings: RefCell<MoveToPointSettings>,
    pub turn_to_settings: RefCell<TurnToSettings>,
    pub boomerang_settings: RefCell<BoomerangSettings>,
    pub ramsete_hybrid_settings: RefCell<RAMSETEHybridSettings>,
}

impl MotionSettings {
    pub fn new(
        move_to_point_settings: RefCell<MoveToPointSettings>,
        turn_to_settings: RefCell<TurnToSettings>,
        boomerang_settings: RefCell<BoomerangSettings>,
        ramsete_hybrid_settings: RefCell<RAMSETEHybridSettings>,
    ) -> Self {
        Self {
            move_to_point_settings,
            turn_to_settings,
            boomerang_settings,
            ramsete_hybrid_settings,
        }
    }
}

pub struct Chassis<T: Tracking> {
    pub(crate) drivetrain: Rc<Drivetrain>,
    pub(super) tracking: Rc<Mutex<T>>,
    pub(super) throttle_curve: Box<dyn DriveCurve>,
    pub(super) steer_curve: Box<dyn DriveCurve>,
    pub(super) motion_handler: MotionHandler,
    pub(super) motion_settings: MotionSettings,
    pub(super) distance_traveled: RefCell<Option<f64>>,
}

impl<T: Tracking> Chassis<T> {
    pub fn new(
        drivetrain: Rc<Drivetrain>,
        tracking: Rc<Mutex<T>>,
        throttle_curve: Box<dyn DriveCurve>,
        steer_curve: Box<dyn DriveCurve>,
        motion_settings: MotionSettings,
    ) -> Rc<Self> {
        Rc::new(Self {
            drivetrain,
            tracking,
            throttle_curve,
            steer_curve,
            motion_handler: MotionHandler::new(),
            distance_traveled: RefCell::new(None),
            motion_settings,
        })
    }

    pub async fn set_filter_state(&self, state: bool) {
        self.tracking.lock().await.set_filter_state(state).await;
    }

    pub async fn filter_state(&self) -> Option<bool> {
        self.tracking.lock().await.filter_state().await
    }

    pub async fn calibrate(&self) {
        self.tracking.lock().await.init(self.tracking.clone()).await;
    }

    pub async fn wait_until(&self, distance: f64) {
        loop {
            vexide::time::sleep(Duration::from_millis(5)).await;
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
            vexide::time::sleep(Duration::from_millis(5)).await;
            if self.distance_traveled.borrow().is_none() {
                break;
            }
        }
    }

    pub async fn set_pose(&self, pose: impl Into<Pose>) {
        self.tracking
            .lock()
            .await
            .set_position(&Vector3::from(pose.into()))
            .await;
    }
    pub async fn pose(&self) -> Pose {
        let mut tracking_lock = self.tracking.lock().await;
        Pose::from(tracking_lock.position())
    }
    pub fn arcade(&self, mut throttle: f64, mut steer: f64, use_drive_curve: bool) {
        if use_drive_curve {
            throttle = self.throttle_curve.update(throttle);
            steer = self.steer_curve.update(steer);
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
            self.throttle_curve.update(left)
        } else {
            left
        };
        right = if use_drive_curve {
            self.throttle_curve.update(right)
        } else {
            right
        };
        self.drivetrain
            .left_motors
            .borrow_mut()
            .set_voltage_all_for_types(left * Motor::V5_MAX_VOLTAGE, left * Motor::EXP_MAX_VOLTAGE);
        self.drivetrain
            .right_motors
            .borrow_mut()
            .set_voltage_all_for_types(
                right * Motor::V5_MAX_VOLTAGE,
                right * Motor::EXP_MAX_VOLTAGE,
            );
    }
}
