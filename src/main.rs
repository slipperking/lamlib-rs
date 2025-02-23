#![no_main]
#![no_std]
extern crate alloc;
extern crate approx;
extern crate nalgebra;

use alloc::{rc::Rc, vec::Vec};
use core::{cell::RefCell, panic::PanicInfo, time::Duration};

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
pub mod laments;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let robot = Robot::new(peripherals).await;
    robot.compete().await;
}
