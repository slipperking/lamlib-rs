use alloc::{boxed::Box, collections::VecDeque, rc::Rc, vec::Vec};
use core::{cell::RefCell, future::Future, pin::Pin, time::Duration};

use vexide::{
    prelude::{BrakeMode, Motor, MotorControl, SmartDevice, Task},
    sync::Mutex,
    time::Instant,
};

use crate::{avg_valid, devices::motor_group::MotorGroup, utils::AllianceColor};

#[derive(PartialEq)]
enum IntakeButtonState {
    None,
    Spin,
    Reverse,
}

// An empty optical callback.
#[macro_export]
macro_rules! default_optical_callback {
    () => {
        RefCell::new(Box::new(|_color: AllianceColor| false))
    };
}
pub use default_optical_callback;

#[allow(clippy::type_complexity)]
pub struct Intake {
    /// Intake motors.
    motor_group: Rc<RefCell<MotorGroup>>,

    alliance_color: Rc<RefCell<AllianceColor>>,

    /// If available, for color sorting.
    optical_sensor: Option<Rc<RefCell<vexide::prelude::OpticalSensor>>>,
    optical_color_history: VecDeque<(Option<AllianceColor>, Instant)>,
    optical_sort_delay: Option<Duration>,

    /// Optical callback ran on every optical sampling. Use for timed intake stops, etc.
    optical_callback: RefCell<Box<dyn Fn(AllianceColor) -> bool>>,

    /// If available, distance is used for color sorting.
    /// Optical sensor stores last color, while distance is for ring detection.
    distance_sensor: Option<Rc<vexide::prelude::DistanceSensor>>,
    distance_stored_color: AllianceColor,
    distance_color_history: VecDeque<(Option<AllianceColor>, Instant)>,
    distance_sort_delay: Option<Duration>,

    last_sort_time: Option<Instant>,
    was_sort_on_previous_epoch: bool,
    is_sort_enabled: bool,

    voltage: f64,
    previous_intake_voltage: f64,

    previous_intake_button_state: IntakeButtonState,
    previous_intake_sort_button_state: bool,

    jam_start_time: Option<Instant>,
    jam_detected: bool,
    additional_anti_jam_criterion: Option<Box<dyn Fn() -> Pin<Box<dyn Future<Output = bool>>>>>,

    task: Option<Task<()>>,
}

#[allow(clippy::type_complexity)]
impl Intake {
    pub fn new(
        motor_group: Rc<RefCell<MotorGroup>>,
        optical_sensor: Option<Rc<RefCell<vexide::prelude::OpticalSensor>>>,
        distance_sensor: Option<Rc<vexide::prelude::DistanceSensor>>,
        alliance_color: Rc<RefCell<AllianceColor>>,
        additional_anti_jam_criterion: Option<Box<dyn Fn() -> Pin<Box<dyn Future<Output = bool>>>>>,

        // This value is not used if distance is available.
        optical_sort_delay: Option<Duration>,
        distance_sort_delay: Option<Duration>,
    ) -> Self {
        Self {
            motor_group,
            optical_sensor,
            distance_sensor,
            voltage: 0.0,
            previous_intake_voltage: 0.0,
            last_sort_time: None,
            jam_start_time: None,
            jam_detected: false,
            optical_color_history: VecDeque::new(),
            distance_color_history: VecDeque::new(),
            distance_stored_color: AllianceColor::None,
            task: None,
            was_sort_on_previous_epoch: false,
            is_sort_enabled: true,
            additional_anti_jam_criterion,
            previous_intake_button_state: IntakeButtonState::None,
            previous_intake_sort_button_state: false,
            alliance_color,
            optical_sort_delay,
            distance_sort_delay,
            optical_callback: default_optical_callback!(),
        }
    }
    pub fn set_voltage(&mut self, voltage: f64) {
        self.voltage = voltage;
    }
    pub fn spin(&mut self) {
        self.set_voltage(1.0);
    }
    pub fn spin_reverse(&mut self) {
        self.set_voltage(-1.0)
    }
    pub fn stop(&mut self) {
        self.voltage = 0.0;
    }
    pub fn set_sort_state(&mut self, state: bool) {
        self.is_sort_enabled = state;
    }

    /// Adds braking for 0.0 voltage.
    fn spin_at_voltage(&self, motor_group: &mut MotorGroup, voltage: f64) {
        if voltage == 0.0 {
            motor_group.set_target_all(MotorControl::Brake(BrakeMode::Coast));
        } else {
            motor_group.set_voltage_all_for_types(
                voltage * Motor::V5_MAX_VOLTAGE,
                voltage * Motor::EXP_MAX_VOLTAGE,
            );
        }
    }
    pub fn clear_optical_callback(&mut self) {
        self.optical_callback = default_optical_callback!();
    }
    pub fn set_optical_callback(&mut self, callback: Box<dyn Fn(AllianceColor) -> bool>) {
        self.optical_callback = RefCell::new(callback);
    }
    fn optical_color(&mut self) -> Option<AllianceColor> {
        // TODO: Optical Callbacks

        if let Some(optical) = &self.optical_sensor {
            let optical = optical.borrow();
            let hue = optical.hue();
            let proximity = optical.proximity();
            if let Ok(hue) = hue {
                if let Ok(proximity) = proximity {
                    if proximity < 90.0 {
                        return {
                            let optical_callback = self.optical_callback.get_mut();
                            if (optical_callback)(AllianceColor::None) {
                                self.optical_callback = default_optical_callback!();
                            };
                            Some(AllianceColor::None)
                        };
                    } else if !(20.0..340.0).contains(&hue) {
                        return {
                            let optical_callback = self.optical_callback.get_mut();
                            if (optical_callback)(AllianceColor::Red) {
                                self.optical_callback = default_optical_callback!();
                            };
                            Some(AllianceColor::Red)
                        };
                    } else if (190.0..230.0).contains(&hue) {
                        return {
                            let optical_callback = self.optical_callback.get_mut();
                            if (optical_callback)(AllianceColor::Blue) {
                                self.optical_callback = default_optical_callback!();
                            };
                            Some(AllianceColor::Blue)
                        };
                    }
                }
            }
        }
        None
    }

    fn optical_color_delayed(&mut self) -> Option<AllianceColor> {
        let delay_time: f64 = if let Some(optical_delay) = self.optical_sort_delay {
            optical_delay.as_secs_f64()
        } else {
            0.0
        };
        let now = Instant::now();
        let color = self.optical_color();
        self.optical_color_history.push_back((color, now));

        while let Some((_, timestamp)) = self.optical_color_history.front() {
            // Remove entries older than delay_time.
            if now.duration_since(*timestamp) <= Duration::from_secs_f64(delay_time) {
                break;
            }
            self.optical_color_history.pop_front();
        }

        // Return the remaining color front, or the most recent if necessary.
        self.optical_color_history
            .front()
            .map_or(color, |(c, _)| *c)
    }

    fn distance_color(&mut self) -> Option<AllianceColor> {
        let color = self.optical_color();
        if let Some(color) = color {
            if let Some(distance) = &self.distance_sensor {
                if color != AllianceColor::None {
                    self.distance_stored_color = color;
                }
                if let Ok(object) = distance.object() {
                    if let Some(object) = object {
                        if object.distance < 60 {
                            return Some(color);
                        }
                    } else {
                        // Distinction here is that Some(AllianceColor::None) refers to no object whereas
                        // core::option::Option None refers to an error, or no sensor.
                        return Some(AllianceColor::None);
                    }
                }
            }
        }
        None
    }

    fn distance_color_delayed(&mut self) -> Option<AllianceColor> {
        let delay_time: f64 = if let Some(distance_delay) = self.distance_sort_delay {
            distance_delay.as_secs_f64()
        } else {
            0.0
        };
        let now = Instant::now();
        let color = self.distance_color();
        self.distance_color_history.push_back((color, now));

        while let Some((_, timestamp)) = self.distance_color_history.front() {
            // Remove entries older than delay_time.
            if now.duration_since(*timestamp) <= Duration::from_secs_f64(delay_time) {
                break;
            }
            self.distance_color_history.pop_front();
        }

        // Return the remaining color front, or the most recent if necessary.
        self.distance_color_history
            .front()
            .map_or(color, |(c, _)| *c)
    }

    async fn update(&mut self) {
        let additional_anti_jam_result: bool = match &self.additional_anti_jam_criterion {
            Some(closure) => closure().await,
            None => true,
        };

        // Sorting precedence as follows:
        // If a distance exists, call the delayed distance color.
        // It should return None if there was an error (such as no optical).
        // If None, take the optical color delayed.
        // Otherwise use that value.
        // If optical returns None as well, then set to false.
        let mut should_sort_on_current_epoch = {
            self.is_sort_enabled
                && (self
                    .distance_color_delayed()
                    .or(self.optical_color_delayed())
                    == Some(*self.alliance_color.borrow()))
        };

        let mut motor_group = self.motor_group.borrow_mut();
        if self.voltage > Motor::V5_MAX_VOLTAGE * 2.0 / 3.0 {
            let actual_velocities = motor_group
                .velocity_all()
                .into_iter()
                .map(|v| v.ok())
                .collect::<Vec<_>>();
            let actual_velocity: Option<f64> = avg_valid!(actual_velocities);
            if let Some(actual_velocity) = actual_velocity {
                if actual_velocity.abs()
                    < avg_valid!(motor_group
                        .gearset_all()
                        .iter()
                        .map(|gearset| match gearset {
                            Ok(gearset) => Some(gearset.max_rpm()),
                            Err(_) => None,
                        })
                        .collect::<Vec<_>>())
                    .unwrap_or(0.0)
                        * 0.1
                    && additional_anti_jam_result
                    || self.jam_detected
                {
                    const MIN_ANTI_JAM_REVERSE_TIME: f64 = 180.0;
                    const MAX_ANTI_JAM_REVERSE_TIME: f64 = 320.0;
                    if self.jam_start_time.is_none() {
                        self.jam_start_time = Some(Instant::now());
                    } else if let Some(jam_start_time) = self.jam_start_time {
                        let elapsed = jam_start_time.elapsed().as_secs_f64() * 1000.0;
                        if (MIN_ANTI_JAM_REVERSE_TIME..MAX_ANTI_JAM_REVERSE_TIME).contains(&elapsed)
                        {
                            self.jam_detected = true;
                            self.spin_at_voltage(&mut motor_group, -Motor::V5_MAX_VOLTAGE);
                            return; // Early brake.
                        } else if self.jam_detected && elapsed >= MAX_ANTI_JAM_REVERSE_TIME {
                            self.spin_at_voltage(&mut motor_group, self.voltage);

                            self.jam_detected = false;
                            self.jam_start_time = None;
                        }
                    }
                } else {
                    self.jam_start_time = None;
                    self.jam_detected = false;
                }
            }
        } else {
            self.jam_start_time = None;
            self.jam_detected = false;
        }

        const SORT_DURATION: f64 = 180.0;
        if should_sort_on_current_epoch
            || (self
                .last_sort_time
                .is_some_and(|time| time.elapsed().as_secs_f64() * 1000.0 < SORT_DURATION))
        {
            if !self.was_sort_on_previous_epoch {
                motor_group.set_voltage_all_for_types(
                    -0.05 * Motor::V5_MAX_VOLTAGE,
                    -0.05 * Motor::EXP_MAX_VOLTAGE,
                );
            }
            if should_sort_on_current_epoch {
                self.last_sort_time = Some(Instant::now())
            }
            should_sort_on_current_epoch = true;
        } else if self.previous_intake_voltage != self.voltage || self.was_sort_on_previous_epoch {
            self.spin_at_voltage(&mut motor_group, self.voltage);
        }
        self.previous_intake_voltage = self.voltage;
        self.was_sort_on_previous_epoch = should_sort_on_current_epoch;
    }
    pub fn driver(&mut self, controller_state: &vexide::devices::controller::ControllerState) {
        if controller_state.button_r1.is_pressed() {
            if controller_state.button_r2.is_pressed() {
                if self.previous_intake_button_state != IntakeButtonState::None {
                    self.stop();
                }
                self.previous_intake_button_state = IntakeButtonState::None;
            } else if self.previous_intake_button_state != IntakeButtonState::Spin {
                self.previous_intake_button_state = IntakeButtonState::Spin;
                self.spin();
            }
        } else if controller_state.button_r2.is_pressed() {
            if self.previous_intake_button_state != IntakeButtonState::Reverse {
                self.previous_intake_button_state = IntakeButtonState::Reverse;
                self.spin_reverse();
            }
        } else if self.previous_intake_button_state != IntakeButtonState::None {
            self.previous_intake_button_state = IntakeButtonState::None;
            self.stop();
        }
        if controller_state.button_down.is_pressed() {
            if !self.previous_intake_sort_button_state {
                self.previous_intake_sort_button_state = true;
                self.set_sort_state(!self.is_sort_enabled);
            }
        } else {
            self.previous_intake_sort_button_state = false;
        }
    }
    pub async fn init(&mut self, async_self_rc: Rc<Mutex<Self>>) {
        if let Some(optical) = &self.optical_sensor {
            let mut optical = optical.borrow_mut();
            let _ = optical.set_integration_time(Duration::from_millis(10));
            let _ = optical.set_led_brightness(0.75);
        }
        self.task = Some(vexide::task::spawn({
            let async_self_rc = async_self_rc.clone();
            async move {
                vexide::time::sleep(Motor::UPDATE_INTERVAL).await;
                loop {
                    let start_time = Instant::now();
                    {
                        async_self_rc.lock().await.update().await;
                    }
                    vexide::time::sleep({
                        let mut duration = Instant::elapsed(&start_time).as_secs_f64() * 1000.0;
                        if duration > Motor::UPDATE_INTERVAL.as_secs_f64() * 1000.0 {
                            duration = 0.0;
                        }
                        Duration::from_millis(
                            (Motor::UPDATE_INTERVAL.as_secs_f64() * 1000.0 - duration * 1000.0)
                                as u64,
                        )
                    })
                    .await;
                }
            }
        }));
        // TODO: Optical and distance callbacks.
        // Store callbacks using Box<dyn Fn() -> bool>
    }
}
