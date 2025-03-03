use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;

use vexide::{core::time::Instant, prelude::Task};

use crate::{devices::motor_group::MotorGroup, utils::AllianceColor};

pub struct Intake {
    /// Intake motors.
    motor_group: Rc<RefCell<MotorGroup>>,

    /// If available, for color sorting.
    optical: Option<Rc<RefCell<vexide::prelude::OpticalSensor>>>,
    last_optical_color: AllianceColor,
    color_history: Vec<AllianceColor>,

    /// If available, distance is used for color sorting.
    /// Optical sensor stores last color, while distance is for ring detection.
    distance: Option<Rc<RefCell<vexide::prelude::DistanceSensor>>>,

    velocity: f64,

    jam_start_time: Option<Instant>,
    jam_detected: bool,

    task: Option<Task<()>>,
}

impl Intake {
    pub fn new(
        motor_group: Rc<RefCell<MotorGroup>>,
        optical: Option<Rc<RefCell<vexide::prelude::OpticalSensor>>>,
        distance: Option<Rc<RefCell<vexide::prelude::DistanceSensor>>>,
    ) -> Self {
        Self {
            motor_group,
            optical,
            distance,
            velocity: 0.0,
            jam_start_time: None,
            jam_detected: false,
            color_history: Vec::new(),
            last_optical_color: AllianceColor::None,
            task: None,
        }
    }
}
