use alloc::{collections::BTreeMap, rc::Rc, vec, vec::Vec};
use core::cell::RefCell;

use vexide::{
    devices::controller::ControllerState,
    prelude::{BrakeMode, Motor, MotorControl, Position, RotationSensor, SmartDevice, Task},
    time::Instant,
};

use crate::{controllers::FeedbackController, devices::motor_group::MotorGroup};

#[derive(PartialEq, Clone, Copy, PartialOrd, Ord, Eq)]
pub enum LadybrownState {
    Off,

    /// Used to override the state machine, such as manual control.
    Free,

    FreeStopHold,

    /// Used to reset positions.
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

pub struct Ladybrown {
    state: LadybrownState,
    last_arm_state: LadybrownState,
    state_reached: bool,
    state_reached_threshold: f64,
    controller: Rc<RefCell<dyn FeedbackController<f64>>>,

    /// A map between a cycle and a cycle pair.
    /// The pair involves an LadybrownState, or the default,
    /// and a shared pointer to the sequence (a vector) itself.
    cycle_orders: BTreeMap<ArmButtonCycle, (LadybrownState, Rc<Vec<LadybrownState>>)>,
    arm_state_positions: Rc<BTreeMap<LadybrownState, f64>>,
    free_start_time: Option<Instant>,
    last_intake_button: ArmButtonCycle,
    pub motor_group: Rc<RefCell<MotorGroup>>,
    rotation_sensor: Rc<RefCell<RotationSensor>>,

    /// The max speed for each state.
    max_speeds: BTreeMap<LadybrownState, Option<f64>>,

    /// This is the ratio from the rotation sensor to the arm.
    gear_ratio: f64,
    task: Option<Task<()>>,
}

impl Ladybrown {
    pub fn new(
        motor_group: Rc<RefCell<MotorGroup>>,
        rotation_sensor: Rc<RefCell<RotationSensor>>,
        gear_ratio: f64,
        controller: Rc<RefCell<dyn FeedbackController<f64>>>,
    ) -> Self {
        let arm_state_positions: Rc<BTreeMap<LadybrownState, f64>> = Rc::new(BTreeMap::from([
            (LadybrownState::Off, 0.0),
            (LadybrownState::Load, 13.0),
            (LadybrownState::LoadUp, 46.0),
            (LadybrownState::NeutralStakeHold, 135.0),
            (LadybrownState::Neutral, 135.0),
            (LadybrownState::Alliance, 195.0),
            (LadybrownState::ManualMax, 215.0),
            (LadybrownState::ManualMin, 0.0),
        ]));

        Self {
            controller,
            state: LadybrownState::Off,
            state_reached: true,
            last_arm_state: LadybrownState::Off,
            cycle_orders: BTreeMap::from([
                (
                    ArmButtonCycle::Default,
                    (
                        LadybrownState::Neutral,
                        Rc::new(vec![
                            LadybrownState::Off,
                            LadybrownState::Load,
                            LadybrownState::Neutral,
                        ]),
                    ),
                ),
                (
                    ArmButtonCycle::Full,
                    (
                        LadybrownState::Load,
                        Rc::new(vec![
                            LadybrownState::Off,
                            LadybrownState::Load,
                            LadybrownState::LoadUp,
                            LadybrownState::NeutralStakeHold,
                            LadybrownState::Neutral,
                            LadybrownState::Alliance,
                        ]),
                    ),
                ),
                (
                    ArmButtonCycle::ManualForward,
                    (
                        LadybrownState::ManualMax,
                        Rc::new(vec![LadybrownState::ManualMax]),
                    ),
                ),
                (
                    ArmButtonCycle::ManualReverse,
                    (
                        LadybrownState::ManualMin,
                        Rc::new(vec![LadybrownState::ManualMin]),
                    ),
                ),
            ]),
            arm_state_positions,
            max_speeds: BTreeMap::from([
                (LadybrownState::ManualMax, Some(8.0)),
                (LadybrownState::ManualMin, Some(8.0)),
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
    pub fn set_state(&mut self, state: LadybrownState) {
        self.state = state;
    }
    pub fn state(&self) -> LadybrownState {
        self.state
    }
    pub fn next_arm_state(
        &self,
        current_state: LadybrownState,
        button: ArmButtonCycle,
    ) -> LadybrownState {
        if let Some((default, state_cycle)) = self.cycle_orders.get(&button) {
            for (i, state) in state_cycle.iter().enumerate() {
                if *state == current_state {
                    return state_cycle[(i + 1) % state_cycle.len()];
                }
            }
            return *default;
        }
        LadybrownState::Off
    }

    pub fn reset_all(&self) {
        self.motor_group
            .borrow_mut()
            .set_target_all(MotorControl::Brake(BrakeMode::Hold));
        self.motor_group
            .borrow_mut()
            .set_position_all(Position::from_degrees(0.0));
        let _ = self
            .rotation_sensor
            .borrow_mut()
            .set_position(Position::from_degrees(0.0));
    }
    pub fn update(&mut self) {
        if self.state == LadybrownState::Free
            || self.state == LadybrownState::FreeTimedReset
            || self.state == LadybrownState::FreeStopHold
        {
            if self.last_arm_state != self.state {
                self.motor_group
                    .borrow_mut()
                    .set_target_all(MotorControl::Brake(
                        if self.state == LadybrownState::FreeStopHold {
                            BrakeMode::Hold
                        } else {
                            BrakeMode::Coast
                        },
                    ));
                self.state_reached = false;
                self.free_start_time = Some(Instant::now());
            }
            if let Some(free_start_time) = self.free_start_time {
                if Instant::elapsed(&free_start_time).as_millis() > 1200
                    && self.state == LadybrownState::FreeTimedReset
                {
                    self.reset_all();
                    self.set_state(LadybrownState::Off);
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
                    if self.state == LadybrownState::Neutral {
                        self.state = LadybrownState::Off;
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
    pub fn driver(&mut self, controller_state: &ControllerState) {
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
                self.set_state(LadybrownState::FreeStopHold);
            }
            self.last_intake_button = ArmButtonCycle::None;
        }
    }
    pub fn init(&mut self, self_rc_refcell: Rc<RefCell<Self>>) {
        self.reset_all();
        self.task = Some(vexide::task::spawn({
            let self_rc_refcell = self_rc_refcell.clone();
            async move {
                vexide::time::sleep(Motor::UPDATE_INTERVAL).await;
                loop {
                    let start_time = Instant::now();
                    self_rc_refcell.borrow_mut().update();
                    vexide::time::sleep(
                        Motor::UPDATE_INTERVAL.saturating_sub(Instant::elapsed(&start_time)),
                    )
                    .await;
                }
            }
        }));
    }
}
