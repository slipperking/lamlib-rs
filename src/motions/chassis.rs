use alloc::rc::Rc;
use core::cell::RefCell;

use nalgebra::{Vector2, Vector3};
use vexide::{core::sync::Mutex, prelude::Motor};

use super::drive_curve::ExponentialDriveCurve;
use crate::{devices::motor_group::MotorGroup, tracking::abstract_tracking::Tracking};

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

pub enum TurnTarget {
    Point(Vector2<f64>),
    Heading(f64),
}

pub enum MoveTarget {
    Point(Vector2<f64>),
    Pose(Vector3<f64>),
}

pub struct Chassis<T: Tracking> {
    drivetrain: Rc<Drivetrain>,
    tracking: Rc<Mutex<T>>,
    throttle_curve: ExponentialDriveCurve,
    steer_curve: ExponentialDriveCurve,
}
impl<T: Tracking> Chassis<T> {
    pub fn new(
        drivetrain: Rc<Drivetrain>,
        tracking: Rc<Mutex<T>>,
        throttle_curve: ExponentialDriveCurve,
        steer_curve: ExponentialDriveCurve,
    ) -> Self {
        Self {
            drivetrain,
            tracking,
            throttle_curve,
            steer_curve,
        }
    }
    pub async fn calibrate(&self) {
        self.tracking.lock().await.init(self.tracking.clone()).await;
    }

    pub async fn set_pose(&mut self, mut position: Vector3<f64>, radians: bool) {
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
        let _ = tracking_lock;
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
        if throttle.abs() + steer.abs() > 12.0 {
            let original_throttle = throttle;
            let original_steer = steer;
            throttle *= 1.0
                - throttle_over_steer_prioritization
                    * (original_steer / Motor::V5_MAX_VOLTAGE).abs();
            steer *=
                1.0 - (1.0 - throttle_over_steer_prioritization) * (original_throttle / 12.0).abs();

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
