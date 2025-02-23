use alloc::vec::Vec;

use vexide::{
    devices::smart::motor::MotorError,
    prelude::{Motor, Position},
};

pub struct MotorGroup {
    motors: Vec<Motor>,
}

impl MotorGroup {
    pub fn new(motors: Vec<Motor>) -> Self {
        Self { motors }
    }

    pub fn set_voltage(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_voltage(voltage);
        }
    }

    pub fn set_velocity(&mut self, rpm: i32) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_velocity(rpm);
        }
    }
    pub fn position_all(&self) -> Vec<Result<Position, MotorError>> {
        self.motors.iter().map(|motor| motor.position()).collect()
    }
}
