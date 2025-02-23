use alloc::{rc::Rc, vec::Vec};
use nalgebra::Point3;
use core::{cell::RefCell, f32::consts::PI};

use vexide::prelude::InertialSensor;

use super::odom_wheels::OdomWheel;
use crate::tracking::abstract_tracking::Tracking;

pub struct OdomSensors {
    imu: Rc<RefCell<InertialSensor>>,
    horizontal1: Rc<Option<OdomWheel>>,
    horizontal2: Rc<Option<OdomWheel>>,
    vertical1: Rc<Option<OdomWheel>>,
    vertical2: Rc<Option<OdomWheel>>
}
pub struct OdomTracking {
    tracked_pose: Point3<f32>,
    prev_imu: f32,
    prev_horizontal1: f32,
    prev_horizontal2: f32,
    prev_vertical1: f32,
    prev_vertical2: f32,
}
impl OdomTracking {
    fn update(&mut self) {

    }
}
impl Tracking for OdomTracking {
    fn position(&mut self) -> Point3<f32> {
        return self.tracked_pose.clone();
    }
}
