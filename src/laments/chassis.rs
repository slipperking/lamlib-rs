use alloc::{rc::Rc, sync::Arc, vec::Vec};
use core::cell::RefCell;

use vexide::{
    core::sync::Mutex,
    prelude::{AdiEncoder, InertialSensor, RotationSensor},
};

use super::motor_group::MotorGroup;

pub struct OdomWheel {
    rotation: Rc<RefCell<Vec<RotationSensor>>>,
    encoder: Rc<RefCell<Vec<AdiEncoder>>>,
    motors: Rc<RefCell<MotorGroup>>,

    /// The ratio of wheel / sensor.
    gear_ratio: f32,
    wheel_diameter: f32,
    drive_wheel_rpm: f32,
}
pub struct OdomSensors {
    imu: Rc<RefCell<InertialSensor>>,
}

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
