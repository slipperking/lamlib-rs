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
    sensors::{
        distance::{LiDAR, LiDARPrecomputedData},
        ParticleFilterSensor,
    },
    ParticleFilter,
};
use subsystems::{arm_state_machine::ArmStateMachine, intake::Intake};
use tracking::odom::{odom_tracking::*, odom_wheels::*};
use utils::AllianceColor;
use vexide::{
    async_runtime::time,
    core::{sync::Mutex, time::Instant},
    devices::smart::*,
    prelude::*,
};
struct Robot {
    alliance_color: Rc<RefCell<AllianceColor>>,
    controller: Controller,
    chassis: Chassis<OdomTracking>,
    intake_arm: Rc<Mutex<ArmStateMachine>>,
    intake: Rc<Mutex<Intake>>,
}
impl Robot {
    async fn new(peripherals: Peripherals) -> Self {
        // TODO: impleemnt color.
        let alliance_color = Rc::new(RefCell::new(AllianceColor::Red));

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
        let mcl_lidar_0 = Rc::new(DistanceSensor::new(peripherals.port_6));
        let mcl_lidar_pi_2 = Rc::new(DistanceSensor::new(peripherals.port_11));
        let mcl_lidar_pi = Rc::new(DistanceSensor::new(peripherals.port_12));
        let mcl_lidar_3_pi_2 = Rc::new(DistanceSensor::new(peripherals.port_8));
        let lidar_group_precompute_data = Rc::new(RefCell::new(LiDARPrecomputedData::new()));
        let particle_filter_sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>> =
            Rc::new(vec![
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, 0.0),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_0,
                    lidar_group_precompute_data.clone(),
                ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, FRAC_PI_2),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_pi_2,
                    lidar_group_precompute_data.clone(),
                ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, PI),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_pi,
                    lidar_group_precompute_data.clone(),
                ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
                Rc::new(RefCell::new(LiDAR::new(
                    Vector3::<f32>::new(1.0, 1.0, 3.0 * FRAC_PI_2),
                    sensor_position_noise,
                    3.0,
                    7.0,
                    mcl_lidar_3_pi_2,
                    lidar_group_precompute_data.clone(),
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

        let pid_arm_controller = Rc::new(RefCell::new(PID::new(2.0, 0.01, 10.0, 2.0, true)));
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
        let intake_motors = Rc::new(RefCell::new(MotorGroup::new(vec![Motor::new(
            peripherals.port_2,
            Gearset::Blue,
            Direction::Reverse,
        )])));
        let optical_sorter = Rc::new(RefCell::new(OpticalSensor::new(peripherals.port_14)));
        let distance_sorter = Rc::new(DistanceSensor::new(peripherals.port_21));
        let intake = Rc::new(Mutex::new(Intake::new(
            intake_motors,
            Some(optical_sorter),
            Some(distance_sorter),
            alliance_color.clone(),
            Some(alloc::boxed::Box::new(|| true)),
            Some(Duration::from_millis(20)),
            Some(Duration::from_millis(20)),
        )));

        chassis.calibrate().await;
        {
            intake_arm.lock().await.init(intake_arm.clone()).await;
        }
        {
            intake.lock().await.init(intake.clone()).await
        }
        Self {
            alliance_color,
            controller: peripherals.primary_controller,
            chassis,
            intake_arm,
            intake,
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
                // Put in scopes to free mutexes.
                {
                    self.intake_arm.lock().await.driver(&state);
                }
                {
                    self.intake.lock().await.driver(&state);
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
