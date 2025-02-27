#![no_main]
#![no_std]

extern crate alloc;
extern crate approx;
extern crate nalgebra;
pub mod controllers;
pub mod devices;
pub mod motions;
pub mod particle_flter;
pub mod tracking;
pub mod utils;

use alloc::{rc::Rc, vec, vec::Vec};
use core::cell::RefCell;

use devices::motor_group::MotorGroup;
use motions::{
    chassis::{Chassis, Drivetrain},
    drive_curve::ExponentialDriveCurve,
};
use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use particle_flter::{
    sensors::{distance::LiDAR, ParticleFilterSensor},
    ParticleFilter,
};
use tracking::odom::{odom_tracking::*, odom_wheels::*};
use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::smart::*,
    prelude::*,
};
struct Robot {
    controller: Controller,
    chassis: Chassis<OdomTracking>,
}
impl Robot {
    async fn new(mut peripherals: Peripherals) -> Self {
        let rotation_vertical_odom_wheel: Rc<RefCell<RotationSensor>> = Rc::new(RefCell::new(
            RotationSensor::new(peripherals.port_7, Direction::Forward),
        ));
        let vertical_odom_wheel: OdomWheel =
            OdomWheel::from_rotation(rotation_vertical_odom_wheel.clone(), 2.0, 1.0, 1.0);
        let imu = OdomInertial::new(
            Rc::new(Mutex::new(InertialSensor::new(peripherals.port_16))),
            1.006,
            1.0,
        );
        let sensors = OdomSensors::new(
            vec![Rc::new(imu)],
            vec![],
            vec![Rc::new(vertical_odom_wheel)],
            1.0,
            0.0,
            0.0,
        );
        let localization = Rc::new(ParticleFilter::new(
            300,
            Rc::new(Matrix3::from_diagonal(&Vector3::<f32>::new(
                0.08, 0.08, 0.003,
            ))),
        ));

        let sensor_position_noise =
            Rc::new(Matrix2::from_diagonal(&Vector2::<f32>::new(0.15, 0.15)));
        let mcl_lidar_0 = Rc::new(RefCell::new(DistanceSensor::new(peripherals.port_6)));
        let particle_filter_sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>> = Rc::new(
            vec![Rc::new(RefCell::new(LiDAR::new(
                Vector3::<f32>::new(1.0, 1.0, 0.0),
                sensor_position_noise,
                3.0,
                7.0,
                mcl_lidar_0,
            ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>],
        );

        let tracking = Rc::new(Mutex::new(OdomTracking::new(
            Rc::new(sensors),
            localization.clone(),
            particle_filter_sensors.clone(),
        )));
        let left_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
            Motor::new(peripherals.port_1, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_18, Gearset::Blue, Direction::Reverse),
            Motor::new(peripherals.port_4, Gearset::Blue, Direction::Reverse),
        ])));
        let right_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
            Motor::new(peripherals.port_10, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_9, Gearset::Blue, Direction::Forward),
            Motor::new(peripherals.port_3, Gearset::Blue, Direction::Forward),
        ])));
        let drivetrain = Rc::new(Drivetrain::new(left_motors, right_motors));
        Self {
            controller: peripherals.primary_controller,
            chassis: Chassis::new(
                drivetrain.clone(),
                tracking,
                ExponentialDriveCurve::new(0.5, 1.0, 1.01),
                ExponentialDriveCurve::new(0.5, 1.0, 1.01),
            ),
        }
    }
}
impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }
    async fn driver(&mut self) {
        println!("Driver!");
    }
    async fn disabled(&mut self) {}
    async fn disconnected(&mut self) {}
    async fn connected(&mut self) {}
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;
    robot.compete().await;
}
