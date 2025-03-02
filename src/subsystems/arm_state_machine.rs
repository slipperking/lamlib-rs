use alloc::{collections::BTreeMap, rc::Rc, vec, vec::Vec};
use core::{cell::RefCell, time::Duration};

use vexide::{
    core::{sync::Mutex, time::Instant},
    devices::controller::ControllerState,
    prelude::{BrakeMode, Motor, MotorControl, Position, RotationSensor, Task},
};

use crate::{controllers::ControllerMethod, devices::motor_group::MotorGroup};

#[derive(PartialEq, Clone, Copy, PartialOrd, Ord, Eq)]
pub enum ArmState {
    Off,

    /// Used to override the state machine, such as manual control.
    Free,

    /// Used to tare positions.
    FreeTimedReset,
    Load,
    LoadUp,
    NeutralStakeHold,
    Neutral,
    Alliance,
    ManualMax,
    ManualMin,
}

#[derive(PartialEq, Clone, Copy, PartialOrd, Ord, Eq)]
pub enum ArmButtonCycle {
    Default,
    Full,
    ManualForward,
    ManualReverse,
    None,
}

pub struct ArmStateMachine {
    state: ArmState,
    last_arm_state: ArmState,
    state_reached: bool,
    state_reached_threshold: f64,
    controller: Rc<RefCell<dyn ControllerMethod<f64>>>,

    /// A map between a cycle and a cycle pair.
    /// The pair involves an ArmState, or the default,
    /// and a shared pointer to the sequence (a vector) itself.
    cycle_orders: BTreeMap<ArmButtonCycle, (ArmState, Rc<Vec<ArmState>>)>,
    arm_state_positions: Rc<BTreeMap<ArmState, f64>>,
    free_start_time: Option<Instant>,
    last_intake_button: ArmButtonCycle,
    motor_group: Rc<RefCell<MotorGroup>>,
    rotation_sensor: Rc<RefCell<RotationSensor>>,

    /// The max speed for each state.
    max_speeds: BTreeMap<ArmState, Option<f64>>,

    /// This is the ratio from the rotation sensor to the arm.
    gear_ratio: f64,
    task: Option<Task<()>>,
}

impl ArmStateMachine {
    pub fn new(
        motor_group: Rc<RefCell<MotorGroup>>,
        rotation_sensor: Rc<RefCell<RotationSensor>>,
        gear_ratio: f64,
        controller: Rc<RefCell<dyn ControllerMethod<f64>>>,
    ) -> Self {
        let arm_state_positions: Rc<BTreeMap<ArmState, f64>> = Rc::new(BTreeMap::from([
            (ArmState::Off, 0.0),
            (ArmState::Load, 13.0),
            (ArmState::LoadUp, 46.0),
            (ArmState::NeutralStakeHold, 135.0),
            (ArmState::Neutral, 135.0),
            (ArmState::Alliance, 195.0),
            (ArmState::ManualMax, 215.0),
            (ArmState::ManualMin, 0.0),
        ]));

        Self {
            controller,
            state: ArmState::Off,
            state_reached: true,
            last_arm_state: ArmState::Off,
            cycle_orders: BTreeMap::from([
                (
                    ArmButtonCycle::Default,
                    (
                        ArmState::Neutral,
                        Rc::new(vec![ArmState::Off, ArmState::Load, ArmState::Neutral]),
                    ),
                ),
                (
                    ArmButtonCycle::Full,
                    (
                        ArmState::Load,
                        Rc::new(vec![
                            ArmState::Off,
                            ArmState::Load,
                            ArmState::LoadUp,
                            ArmState::NeutralStakeHold,
                            ArmState::Neutral,
                            ArmState::Alliance,
                        ]),
                    ),
                ),
                (
                    ArmButtonCycle::ManualForward,
                    (ArmState::ManualMax, Rc::new(vec![ArmState::ManualMax])),
                ),
                (
                    ArmButtonCycle::ManualReverse,
                    (ArmState::ManualMin, Rc::new(vec![ArmState::ManualMin])),
                ),
            ]),
            arm_state_positions,
            max_speeds: BTreeMap::from([
                (ArmState::ManualMax, Some(8.0)),
                (ArmState::ManualMin, Some(8.0)),
            ]),
            free_start_time: None,
            last_intake_button: ArmButtonCycle::None,
            state_reached_threshold: 1.5,
            motor_group,
            rotation_sensor,
            gear_ratio,
            task: None,
        }
    }
    pub fn set_state(&mut self, state: ArmState) {
        self.state = state;
    }
    pub fn next_arm_state(&self, current_state: ArmState, button: ArmButtonCycle) -> ArmState {
        if let Some((default, state_cycle)) = self.cycle_orders.get(&button) {
            for (i, state) in state_cycle.iter().enumerate() {
                if *state == current_state {
                    return state_cycle[(i + 1) % state_cycle.len()];
                }
            }
            return *default;
        }
        ArmState::Off
    }

    pub async fn reset_all(&self) {
        self.motor_group
            .borrow_mut()
            .set_position_all(Position::from_degrees(0.0));
        let _ = self
            .rotation_sensor
            .borrow_mut()
            .set_position(Position::from_degrees(0.0));
    }
    pub async fn update(&mut self) {
        if self.state == ArmState::Free || self.state == ArmState::FreeTimedReset {
            if self.last_arm_state != self.state {
                self.motor_group
                    .borrow_mut()
                    .set_target_all(MotorControl::Brake(BrakeMode::Coast));
                self.state_reached = false;
                self.free_start_time = Some(Instant::now());
            }
            if let Some(free_start_time) = self.free_start_time {
                if Instant::elapsed(&free_start_time).as_millis() > 1200
                    && self.state == ArmState::FreeTimedReset
                {
                    self.reset_all().await;
                    self.set_state(ArmState::Off);
                    return;
                }
            }
        } else {
            let max_speed: f64 = self
                .max_speeds
                .get(&self.state)
                .unwrap_or(&Some(f64::INFINITY))
                .unwrap_or(f64::INFINITY);
            let target_position = self.arm_state_positions.get(&self.state);
            if let Some(target_position) = target_position {
                let current_arm_position = self
                    .rotation_sensor
                    .borrow()
                    .position()
                    .unwrap_or_default()
                    .as_degrees();
                let error = target_position - current_arm_position * self.gear_ratio;
                {
                    let controller_output = self
                        .controller
                        .borrow_mut()
                        .update(error)
                        .clamp(-max_speed, max_speed);
                    self.motor_group.borrow_mut().set_voltage_all_for_types(
                        controller_output,
                        controller_output * Motor::EXP_MAX_VOLTAGE / Motor::V5_MAX_VOLTAGE,
                    );
                }
                if error.abs() < self.state_reached_threshold {
                    if self.state == ArmState::Neutral {
                        self.state = ArmState::Off;
                    } else {
                        self.state_reached = true;
                    }
                    if error.abs() < self.state_reached_threshold / 2.0 {
                        self.controller.borrow_mut().reset();
                    }
                } else {
                    self.state_reached = false;
                }
            }
        }

        if self.last_arm_state != self.state {
            self.controller.borrow_mut().reset();
        }
        self.last_arm_state = self.state;
    }
    pub fn opcontrol(&mut self, controller_state: &ControllerState) {
        if controller_state.button_a.is_pressed() {
            if self.last_intake_button != ArmButtonCycle::Default {
                self.last_intake_button = ArmButtonCycle::Default;
                self.set_state(self.next_arm_state(self.state, ArmButtonCycle::Default));
            }
        } else if controller_state.button_x.is_pressed() {
            if self.last_intake_button != ArmButtonCycle::Full {
                self.last_intake_button = ArmButtonCycle::Full;
                self.set_state(self.next_arm_state(self.state, ArmButtonCycle::Full));
            }
        } else if controller_state.button_r1.is_pressed() && controller_state.button_r2.is_pressed()
        {
            if self.last_intake_button != ArmButtonCycle::ManualForward {
                self.last_intake_button = ArmButtonCycle::ManualForward;
                self.set_state(self.next_arm_state(self.state, ArmButtonCycle::ManualForward));
            }
        } else if controller_state.button_y.is_pressed() {
            if self.last_intake_button != ArmButtonCycle::ManualReverse {
                self.last_intake_button = ArmButtonCycle::ManualReverse;
                self.set_state(self.next_arm_state(self.state, ArmButtonCycle::ManualReverse));
            }
        } else {
            if self.last_intake_button == ArmButtonCycle::ManualForward
                || self.last_intake_button == ArmButtonCycle::ManualReverse
            {
                self.set_state(ArmState::Free);
            }
            self.last_intake_button = ArmButtonCycle::None;
        }
    }
    pub async fn init(&mut self, async_self_rc: Rc<Mutex<Self>>) {
        self.reset_all().await;
        self.task = Some(vexide::async_runtime::spawn({
            let async_self_rc = async_self_rc.clone();
            async move {
                vexide::async_runtime::time::sleep(Duration::from_millis(10)).await;
                loop {
                    let start_time = Instant::now();
                    {
                        async_self_rc.lock().await.update().await;
                    }
                    vexide::async_runtime::time::sleep({
                        let mut duration = Instant::elapsed(&start_time).as_millis();
                        if duration > 10 {
                            duration = 0;
                        }
                        Duration::from_millis((10 - duration) as u64)
                    })
                    .await;
                }
            }
        }));
    }
    // TODO: implement task and controller logic.
}
