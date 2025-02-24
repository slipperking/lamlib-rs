use alloc::vec::Vec;

use vexide::{
    devices::smart::motor::MotorError,
    prelude::{Gearset, Motor, Position},
};

pub struct MotorGroup {
    motors: Vec<Motor>,
}

impl MotorGroup {
    pub fn size(&self) -> usize {
        self.motors.len()
    }
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
    pub fn set_position_all(&mut self, position: Position) -> Vec<Result<(), MotorError>> {
        self.motors
            .iter_mut()
            .map(|motor| motor.set_position(position))
            .collect()
    }
    pub fn position_all(&self) -> Vec<Result<Position, MotorError>> {
        self.motors.iter().map(|motor| motor.position()).collect()
    }
    pub fn gearset_all(&self) -> Vec<Result<Gearset, MotorError>> {
        self.motors.iter().map(|motor| motor.gearset()).collect()
    }
}
