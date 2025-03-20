#![no_main]
#![no_std]
extern crate alloc;

pub mod controllers;
pub mod devices;

#[macro_use]
pub mod differential;
pub mod particle_filter;
pub mod subsystems;
pub mod tracking;
pub mod utils;

pub mod auton_routines;
use alloc::{boxed::Box, rc::Rc, vec, vec::Vec};
use core::{
    cell::RefCell,
    f32::consts::{FRAC_PI_2, PI},
    time::Duration,
};

use autons::{prelude::*, simple::SimpleSelect};
use controllers::pid::PID;
use devices::motor_group::MotorGroup;
use differential::{
    chassis::{Chassis, Drivetrain, MotionSettings},
    drive_curve::ExponentialDriveCurve,
    motions::{
        angular::TurnToSettings, boomerang::BoomerangSettings, lateral::MoveToPointSettings,
        ramsete::RAMSETEHybridSettings, ExitCondition, ExitConditionGroup,
    },
};
use nalgebra::{Matrix2, Matrix3, Vector2, Vector3};
use particle_filter::{
    sensors::{
        distance::{LiDAR, LiDARPrecomputedData},
        ParticleFilterSensor,
    },
    ParticleFilter,
};
use subsystems::{intake::Intake, ladybrown::LadyBrown, pneumatics::PneumaticWrapper};
use tracking::odom::{odom_tracking::*, odom_wheels::*};
use utils::AllianceColor;
use vexide::{devices::adi::digital::LogicLevel, prelude::*, sync::Mutex, time};
pub struct Robot {
    #[allow(dead_code)]
    pub alliance_color: Rc<RefCell<AllianceColor>>,
    #[warn(dead_code)]
    pub controller: Controller,
    pub chassis: Rc<Chassis<OdomTracking>>,

    pub ladybrown_arm: Rc<RefCell<LadyBrown>>,

    /// A Mutex containing the intake wrapped within a shared pointer.
    ///
    /// Mutex for async closures. Anything async should use Mutex so that
    /// borrows are not held across await points.
    pub intake: Rc<Mutex<Intake>>,
    pub doinker_left: PneumaticWrapper,
    pub doinker_right: PneumaticWrapper,
    pub clamp_main: PneumaticWrapper,
}

impl SelectCompete for Robot {
    async fn driver(&mut self) {
        println!("Driver!");
        loop {
            {
                let state = self.controller.state().unwrap_or_default();
                self.chassis
                    .arcade(state.left_stick.y(), state.right_stick.x(), true);
                // Put in scopes to free mutexes.
                {
                    self.ladybrown_arm.borrow_mut().driver(&state);
                    self.doinker_left.driver_toggle(state.button_b.is_pressed());
                    self.doinker_right
                        .driver_toggle(state.button_up.is_pressed());
                    self.clamp_main.driver_explicit(
                        state.button_l1.is_pressed(),
                        state.button_l2.is_pressed(),
                        LogicLevel::High,
                        LogicLevel::Low,
                    );
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
    // TODO: implement color.
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
    let particle_filter_sensors: Rc<Vec<Rc<RefCell<dyn ParticleFilterSensor<3>>>>> = Rc::new(vec![
        Rc::new(RefCell::new(LiDAR::new(
            Vector3::<f32>::new(1.0, 1.0, 0.0),
            sensor_position_noise,
            3.0,
            7.0,
            mcl_lidar_0,
            lidar_group_precompute_data.clone(),
            None,
        ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
        Rc::new(RefCell::new(LiDAR::new(
            Vector3::<f32>::new(1.0, 1.0, FRAC_PI_2),
            sensor_position_noise,
            3.0,
            7.0,
            mcl_lidar_pi_2,
            lidar_group_precompute_data.clone(),
            None,
        ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
        Rc::new(RefCell::new(LiDAR::new(
            Vector3::<f32>::new(1.0, 1.0, PI),
            sensor_position_noise,
            3.0,
            7.0,
            mcl_lidar_pi,
            lidar_group_precompute_data.clone(),
            None,
        ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
        Rc::new(RefCell::new(LiDAR::new(
            Vector3::<f32>::new(1.0, 1.0, 3.0 * FRAC_PI_2),
            sensor_position_noise,
            3.0,
            7.0,
            mcl_lidar_3_pi_2,
            lidar_group_precompute_data.clone(),
            None,
        ))) as Rc<RefCell<dyn ParticleFilterSensor<3>>>,
    ]);

    let tracking: Rc<Mutex<OdomTracking>> = Rc::new(Mutex::new(OdomTracking::new(
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
    let lateral_exit_conditions = ExitConditionGroup::new(vec![
        ExitCondition::new(1.0, Duration::from_millis(150)),
        ExitCondition::new(3.0, Duration::from_millis(500)),
    ]);
    let angular_exit_conditions = ExitConditionGroup::new(vec![
        ExitCondition::new(angle!(degrees: 1.0,), Duration::from_millis(150)),
        ExitCondition::new(angle!(degrees: 3.0,), Duration::from_millis(500)),
    ]);
    let lateral_controller = Box::new(PID::new(0.18, 0.0, 0.0, 2.0, true));
    let angular_controller = Box::new(PID::new(0.18, 0.0, 0.0, 2.0, true));

    let motion_settings = MotionSettings::new(
        RefCell::new(MoveToPointSettings::new(
            lateral_controller.clone(),
            angular_controller.clone(),
            lateral_exit_conditions.clone(),
            angular_exit_conditions.clone(),
        )),
        RefCell::new(TurnToSettings::new(
            angular_controller.clone(),
            Box::new(PID::new(0.18, 0.0, 0.0, 2.0, true)), // Swing constants.
            angular_exit_conditions.clone(),
        )),
        RefCell::new(BoomerangSettings::new(
            lateral_controller.clone(),
            angular_controller.clone(),
            lateral_exit_conditions.clone(),
            angular_exit_conditions.clone(),
        )),
        RefCell::new(RAMSETEHybridSettings::new(
            lateral_controller.clone(),
            angular_controller.clone(),
            lateral_exit_conditions.clone(),
            angular_exit_conditions.clone(),
            1.0,
        )),
    );
    let chassis = Chassis::new(
        drivetrain.clone(),
        tracking,
        ExponentialDriveCurve::new(0.1, 0.1, 1.01),
        ExponentialDriveCurve::new(0.1, 0.1, 1.01),
        motion_settings,
    );

    let pid_arm_controller = Rc::new(RefCell::new(PID::new(0.18, 0.0, 0.0, 2.0, true)));
    let rotation_arm_state_machine = Rc::new(RefCell::new(RotationSensor::new(
        peripherals.port_17,
        Direction::Forward,
    )));

    let arm_state_machine_motors = Rc::new(RefCell::new(MotorGroup::new(vec![
        Motor::new_exp(peripherals.port_19, Direction::Reverse),
        Motor::new_exp(peripherals.port_20, Direction::Forward),
    ])));
    let ladybrown_arm = Rc::new(RefCell::new(LadyBrown::new(
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
    let optical_sorter = Rc::new(RefCell::new(OpticalSensor::new(peripherals.port_15)));
    let distance_sorter = Rc::new(DistanceSensor::new(peripherals.port_21));
    let ladybrown_arm_clone = ladybrown_arm.clone();
    let intake = Rc::new(Mutex::new(Intake::new(
        intake_motors,
        Some(optical_sorter),
        Some(distance_sorter),
        alliance_color.clone(),
        Some(alloc::boxed::Box::new(move || {
            let ladybrown_arm = ladybrown_arm_clone.clone();
            alloc::boxed::Box::pin(async move {
                ladybrown_arm.borrow_mut().state() != subsystems::ladybrown::LadyBrownState::Load
            })
        })),
        Some(Duration::from_millis(20)),
        Some(Duration::from_millis(20)),
    )));

    chassis.calibrate().await;
    {
        ladybrown_arm.borrow_mut().init(ladybrown_arm.clone());
    }
    {
        intake.lock().await.init(intake.clone()).await
    }
    chassis
        .set_pose(differential::pose::Pose::new(
            0.0,
            0.0,
            angle!(degrees: 0.0,),
        ))
        .await;
    let mut controller = peripherals.primary_controller;
    let _ = controller.rumble("._.").await;

    let robot = Robot {
        alliance_color,
        controller,
        chassis,
        ladybrown_arm,
        intake,
        doinker_left: PneumaticWrapper::new(Rc::new(RefCell::new(AdiDigitalOut::new(
            peripherals.adi_f,
        )))),
        doinker_right: PneumaticWrapper::new(Rc::new(RefCell::new(AdiDigitalOut::new(
            peripherals.adi_g,
        )))),
        clamp_main: PneumaticWrapper::new(Rc::new(RefCell::new(AdiDigitalOut::new(
            peripherals.adi_h,
        )))),
    };
    robot
        .compete(SimpleSelect::new(
            peripherals.display,
            // TODO: Add on-switch controller screen updates.
            create_routine_array!(),
        ))
        .await;
}
