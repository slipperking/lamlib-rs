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
use core::{cell::RefCell, panic::PanicInfo, time::Duration};

use tracking::odom::{odom_tracking::*, odom_wheels::*};
use vexide::{
    core::{
        competition::{CompetitionMode, CompetitionSystem},
        sync::Mutex,
        time::Instant,
    },
    devices::{controller::ControllerId, peripherals, smart::*},
    prelude::*,
};
struct Robot {
    controller: Controller,
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
        let tracker = Rc::new(Mutex::new(OdomTracking::new(Rc::new(sensors))));
        Self {
            controller: peripherals.primary_controller,
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
