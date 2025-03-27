use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;

use vexide::{
    devices::smart::motor::MotorError,
    prelude::{AdiEncoder, Gearset, Position, RotationSensor},
};

use crate::devices::motor_group::MotorGroup;

// TODO: Use a trait such as "RotarySensor" to generalize the sensor types.

pub struct OdomWheel {
    rotation: Option<Rc<RefCell<RotationSensor>>>,
    encoder: Option<Rc<RefCell<AdiEncoder>>>,
    motors: Option<Rc<RefCell<MotorGroup>>>,

    /// Diameter in inches.
    wheel_diameter: f64,

    /// The ratio of wheel / sensor.
    gear_ratio: f64,
    drive_wheel_rpm: f64,

    /// Tracking offset.
    offset: f64,
}
impl OdomWheel {
    pub fn from_rotation(
        rotation: Rc<RefCell<RotationSensor>>,
        wheel_diameter: f64,
        gear_ratio: f64,
        offset: f64,
    ) -> Self {
        Self {
            rotation: Some(rotation),
            encoder: None,
            motors: None,

            wheel_diameter,
            gear_ratio,
            drive_wheel_rpm: 0.0,
            offset,
        }
    }
    pub fn from_adi_encoder(
        encoder: Rc<RefCell<AdiEncoder>>,
        wheel_diameter: f64,
        gear_ratio: f64,
        offset: f64,
    ) -> Self {
        Self {
            rotation: None,
            encoder: Some(encoder),
            motors: None,

            wheel_diameter,
            gear_ratio,
            drive_wheel_rpm: 0.0,
            offset,
        }
    }
    pub fn from_motors(
        motors: Rc<RefCell<MotorGroup>>,
        wheel_diameter: f64,
        gear_ratio: f64,
        drive_wheel_rpm: f64,
        offset: f64,
    ) -> Self {
        Self {
            rotation: None,
            encoder: None,
            motors: Some(motors),

            wheel_diameter,
            gear_ratio,
            drive_wheel_rpm,
            offset,
        }
    }
    pub fn distance_traveled(&self) -> Option<f64> {
        if let Some(rotation) = &self.rotation {
            return rotation.borrow().position().ok().map(|position| {
                position.as_revolutions()
                    * core::f64::consts::PI
                    * (self.wheel_diameter / self.gear_ratio)
            });
        }
        if let Some(encoder) = &self.encoder {
            return encoder.borrow().position().ok().map(|position| {
                position.as_revolutions()
                    * core::f64::consts::PI
                    * (self.wheel_diameter / self.gear_ratio)
            });
        }
        if let Some(motors) = &self.motors {
            let motors = motors.borrow();
            let gearsets: Vec<Result<Gearset, MotorError>> = motors.gearset_all();
            let positions: Vec<Result<Position, MotorError>> = motors.position_all();
            let motor_count = motors.size();
            let mut distances: Vec<f64> = Vec::new();
            (0..motor_count).for_each(|i| {
                if let (Ok(position), Ok(gearset)) = (&positions[i], &gearsets[i]) {
                    let gearset_rpm: f64 = gearset.max_rpm();
                    distances.push(
                        position.as_revolutions()
                            * core::f64::consts::PI
                            * (self.wheel_diameter * self.drive_wheel_rpm / gearset_rpm),
                    );
                }
            });
            if distances.is_empty() {
                return None;
            }
            return Some(distances.iter().sum::<f64>() / distances.len() as f64);
        }
        None
    }
    pub fn offset(&self) -> f64 {
        self.offset
    }
    pub fn init(&self) {
        if let Some(rotation) = &self.rotation {
            let _ = rotation.borrow_mut().reset_position();
        }
        if let Some(encoder) = &self.encoder {
            let _ = encoder.borrow_mut().reset_position();
        }
        if let Some(motors) = &self.motors {
            motors
                .borrow_mut()
                .set_position_all(Position::from_radians(0.0));
        }
    }
}
