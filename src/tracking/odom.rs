use super::odom_wheels::{self, OdomWheel};
use alloc::{rc::Rc, sync::Arc, vec::Vec};
use core::{cell::RefCell, f32::consts::PI};

use vexide::{
    prelude::{AdiEncoder, InertialSensor, RotationSensor}
};

pub struct OdomSensors {
    imu: Rc<RefCell<InertialSensor>>,
    horizontal1: *const OdomWheel,
    horizontal2: *const OdomWheel,
    vertical1: *const OdomWheel,
    vertical2: *const OdomWheel,
}
