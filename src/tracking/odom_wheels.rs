use alloc::{
    rc::Rc,
    vec::Vec,
};
use core::cell::RefCell;

use vexide::{
    devices::smart::motor::MotorError,
    prelude::{AdiEncoder, Gearset, Position, RotationSensor},
};

use crate::laments::motor_group::MotorGroup;

pub struct OdomWheel {
    rotation: Option<Rc<RefCell<RotationSensor>>>,
    encoder: Option<Rc<RefCell<AdiEncoder>>>,
    motors: Option<Rc<RefCell<MotorGroup>>>,

    /// Diameter in inches.
    wheel_diameter: f32,

    /// The ratio of wheel / sensor.
    gear_ratio: f32,
    drive_wheel_rpm: f32,
}
impl OdomWheel {
    pub fn from_rotation(
        rotation: Rc<RefCell<RotationSensor>>,
        wheel_diameter: f32,
        gear_ratio: f32,
    ) -> Self {
        Self {
            rotation: Some(rotation),
            encoder: None,
            motors: None,

            wheel_diameter,
            gear_ratio,
            drive_wheel_rpm: 0.0,
        }
    }
    pub fn from_adi_encoder(
        encoder: Rc<RefCell<AdiEncoder>>,
        wheel_diameter: f32,
        gear_ratio: f32,
    ) -> Self {
        Self {
            rotation: None,
            encoder: Some(encoder),
            motors: None,

            wheel_diameter,
            gear_ratio,
            drive_wheel_rpm: 0.0,
        }
    }
    pub fn from_motors(
        motors: Rc<RefCell<MotorGroup>>,
        wheel_diameter: f32,
        gear_ratio: f32,
        drive_wheel_rpm: f32,
    ) -> Self {
        Self {
            rotation: None,
            encoder: None,
            motors: Some(motors),

            wheel_diameter,
            gear_ratio,
            drive_wheel_rpm,
        }
    }
    pub fn distance_traveled(&self) -> Option<f32> {
        if let Some(rotation) = &self.rotation {
            if let Ok(position) = rotation.borrow().position() {
                return Some(
                    (position.as_revolutions()
                        * core::f64::consts::PI
                        * (self.wheel_diameter / self.gear_ratio) as f64)
                        as f32,
                );
            }
        }
        if let Some(encoder) = &self.encoder {
            if let Ok(position) = encoder.borrow().position() {
                return Some(
                    (position.as_revolutions()
                        * core::f64::consts::PI
                        * (self.wheel_diameter / self.gear_ratio) as f64)
                        as f32,
                );
            }
        }
        if let Some(motors) = &self.motors {
            let motors = motors.borrow();
            let gearsets: Vec<Result<Gearset, MotorError>> = motors.gearset_all();
            let positions: Vec<Result<Position, MotorError>> = motors.position_all();
            let motor_count = motors.size();
            let mut distances: Vec<Option<f32>> = Vec::new();
            (0..motor_count).for_each(|i| {
                if let Ok(position) = positions[i] {
                    if let Ok(gearset) = gearsets[i] {
                        let gearset_rpm = match &gearset {
                            Gearset::Green => 200.0,
                            Gearset::Blue => 600.0,
                            Gearset::Red => 100.0,
                        };
                        distances.push(Some(
                            (position.as_revolutions()
                                * core::f64::consts::PI
                                * (self.wheel_diameter * self.drive_wheel_rpm / gearset_rpm) as f64)
                                as f32,
                        ));
                        return;
                    }
                }
                distances.push(None);
            });
            let valid_distances: Vec<f32> = distances.iter().filter_map(|&d| d).collect();
            if valid_distances.is_empty() {
                return None;
            }
            return Some(valid_distances.iter().sum::<f32>() / valid_distances.len() as f32);
        }
        None
    }
}
