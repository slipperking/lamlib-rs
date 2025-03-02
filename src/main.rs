#![no_main]
#![no_std]

extern crate alloc;
extern crate approx;
extern crate nalgebra;
pub mod controllers;
pub mod devices;
pub mod motions;
pub mod particle_flter;
pub mod subsystems;
pub mod tracking;
pub mod utils;

use alloc::{rc::Rc, vec, vec::Vec};
use core::{
    cell::RefCell,
    f32::consts::{FRAC_PI_2, PI},
    time::Duration,
};

use controllers::pid::PID;
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
use subsystems::arm_state::{ArmState, ArmStateMachine};
use tracking::odom::{odom_tracking::*, odom_wheels::*};
use vexide::{
    async_runtime::time,
    core::{sync::Mutex, time::Instant},
    devices::smart::*,
    prelude::*,
};
struct Robot {
    controller: Controller,
    chassis: Chassis<OdomTracking>,
    intake_arm: Rc<Mutex<ArmStateMachine>>,
}
impl Robot {
    async fn new(peripherals: Peripherals) -> Self {
        let rotation_vertical_odom_wheel: Rc<RefCell<RotationSensor>> = Rc::new(RefCell::new(
            RotationSensor::new(peripherals.port_7, Direction::Forward),
        ));
        let vertical_odom_wheel: OdomWheel =
            OdomWheel::from_rotation(rotation_vertical_odom_wheel.clone(), 2.0, 1.0, 1.0);
        let imu = OdomInertial::new(
            Rc::new(Mutex::new(InertialSensor::new(peripherals.port_13))),
            1.004,
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
            Matrix3::from_diagonal(&Vector3::<f32>::new(0.08, 0.08, 0.003)),
        ));

        let sensor_position_noise = Matrix2::from_diagonal(&Vector2::<f32>::new(0.15, 0.15));
        let mcl_lidar_0 = Rc::new(RefCell::new(DistanceSensor::new(peripherals.port_6)));
        let mcl_lidar_pi_2 = Rc::new(RefCell::new(DistanceSensor::new(peripherals.port_14)));
        let mcl_lidar_pi = Rc::new(RefCell::new(DistanceSensor::new(peripherals.port_12)));
        let mcl_lidar_3_pi_2 = Rc::new(RefCell::new(DistanceSensor::new(peripherals.port_8)));
        let particle_filter_sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>> =
            Rc::new(vec![
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, 0.0),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_0,
                ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, FRAC_PI_2),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_pi_2,
                ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, PI),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_pi,
                ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, 3.0 * FRAC_PI_2),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_3_pi_2,
                ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
            ]);

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
        let chassis = Chassis::new(
            drivetrain.clone(),
            tracking,
            ExponentialDriveCurve::new(0.5, 1.0, 1.01),
            ExponentialDriveCurve::new(0.5, 1.0, 1.01),
        );

        let pid_arm_controller = Rc::new(RefCell::new(PID::new(240.0, 1.0, 2000.0, 4.0, true)));
        let rotation_arm_state_machine = Rc::new(RefCell::new(RotationSensor::new(
            peripherals.port_17,
            Direction::Forward,
        )));

        let arm_state_machine_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
            Motor::new_exp(peripherals.port_19, Direction::Reverse),
            Motor::new_exp(peripherals.port_20, Direction::Forward),
        ])));
        let intake_arm = Rc::new(Mutex::new(ArmStateMachine::new(
            arm_state_machine_motors,
            rotation_arm_state_machine.clone(),
            1.0,
            pid_arm_controller,
        )));

        chassis.calibrate().await;
        intake_arm.lock().await.init(intake_arm.clone()).await;
        Self {
            controller: peripherals.primary_controller,
            chassis,
            intake_arm,
        }
    }
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        println!("Autonomous!");
    }
    async fn driver(&mut self) {
        println!("Driver!");
        loop {
            {
                let state = self.controller.state().unwrap_or_default();
                self.chassis
                    .arcade(state.left_stick.y(), state.right_stick.x(), true, 0.5);
                {
                    self.intake_arm.lock().await.opcontrol(&state);
                }
            }
            time::sleep(Duration::from_millis(20)).await;
        }
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
