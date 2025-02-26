use alloc::rc::Rc;
use core::cell::RefCell;

use vexide::core::sync::Mutex;

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

pub struct Chassis<T: Tracking + 'static> {
    drivetrain: Rc<Drivetrain>,
    tracking: Rc<Mutex<T>>,
    throttle_curve: ExponentialDriveCurve,
    steer_curve: ExponentialDriveCurve,
}

impl<T: Tracking + 'static> Chassis<T> {
    pub async fn calibrate(&self) {
        self.tracking.lock().await.init(self.tracking.clone()).await;
    }
}
