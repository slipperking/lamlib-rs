use alloc::vec::Vec;

use vexide::{
    devices::smart::motor::{MotorError, MotorType},
    prelude::{Gearset, Motor, MotorControl, Position},
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
    pub fn set_voltage_all(&mut self, voltage: f64) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_voltage(voltage);
        }
    }
    pub fn set_voltage_all_for_types(&mut self, voltage_v5: f64, voltage_exp: f64) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_voltage(match motor.motor_type() {
                MotorType::V5 => voltage_v5,
                MotorType::Exp => voltage_exp,
            });
        }
    }
    pub fn set_velocity_all(&mut self, rpm: i32) {
        for motor in self.motors.iter_mut() {
            let _ = motor.set_velocity(rpm);
        }
    }
    // Use this for braking.
    pub fn set_target_all(&mut self, target: MotorControl) -> Vec<Result<(), MotorError>>{
        self.motors
            .iter_mut()
            .map(|motor| motor.set_target(target))
            .collect()
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
    pub fn velocity_all(&self) -> Vec<Result<f64, MotorError>> {
        self.motors.iter().map(|motor| motor.velocity()).collect()
    }
    pub fn gearset_all(&self) -> Vec<Result<Gearset, MotorError>> {
        self.motors.iter().map(|motor| motor.gearset()).collect()
    }
}
