use alloc::rc::Rc;
use core::cell::RefCell;

use crate::devices::motor_group::MotorGroup;

pub struct Drivetrain {
    left_motors: Rc<RefCell<MotorGroup>>,
    right_motors: Rc<RefCell<MotorGroup>>,
    track_width: f32,
    wheel_diameter: f32,
    rpm: f32,
}

pub struct Chassis {
    drivetrain: Drivetrain,
}
