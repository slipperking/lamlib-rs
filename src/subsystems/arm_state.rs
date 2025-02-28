use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use vexide::{devices::controller::ControllerState, prelude::RotationSensor};

use crate::{controllers::ControllerMethod, devices::motor_group::MotorGroup};

#[derive(PartialEq, Clone, Copy)]
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
}

#[derive(PartialEq, Clone, Copy)]
pub enum ArmButtonCycle {
    Default,
    Full,
    None,
}

pub struct ArmStateMachine<'a> {
    state: ArmState,
    last_arm_state: ArmState,
    state_reached: bool,
    state_reached_threshold: f64,
    controller: &'a mut dyn ControllerMethod,
    cycle_orders: &'a [(ArmButtonCycle, (ArmState, &'a [ArmState]))],
    arm_state_positions: &'a [(ArmState, f32)],
    free_start_time: Option<Duration>,
    last_intake_button: ArmButtonCycle,
    motor_group: Rc<RefCell<MotorGroup>>,
    rotation_sensor: Rc<RefCell<RotationSensor>>,
    gear_ratio: f64,
}

impl<'a> ArmStateMachine<'a> {
    pub fn new(
        motor_group: Rc<RefCell<MotorGroup>>,
        rotation_sensor: Rc<RefCell<RotationSensor>>,
        gear_ratio: f64,
        controller: &'a mut dyn ControllerMethod,
    ) -> Self {
        const CYCLE_ORDERS: &[(ArmButtonCycle, (ArmState, &[ArmState]))] = &[
            (
                ArmButtonCycle::Full,
                (
                    ArmState::Neutral,
                    &[ArmState::Off, ArmState::Load, ArmState::Neutral],
                ),
            ),
            (
                ArmButtonCycle::Full,
                (
                    ArmState::Load,
                    &[
                        ArmState::Off,
                        ArmState::Load,
                        ArmState::LoadUp,
                        ArmState::NeutralStakeHold,
                        ArmState::Neutral,
                        ArmState::Alliance,
                    ],
                ),
            ),
        ];

        // Define the arm state positions
        const ARM_STATE_POSITIONS: &[(ArmState, f32)] = &[
            (ArmState::Off, 0.0),
            (ArmState::Load, 13.0),
            (ArmState::LoadUp, 46.0),
            (ArmState::NeutralStakeHold, 135.0),
            (ArmState::Neutral, 133.0),
            (ArmState::Alliance, 190.0),
        ];

        Self {
            controller,
            state: ArmState::Off,
            state_reached: true,
            last_arm_state: ArmState::Off,
            cycle_orders: CYCLE_ORDERS,
            arm_state_positions: ARM_STATE_POSITIONS,
            free_start_time: None,
            last_intake_button: ArmButtonCycle::None,
            state_reached_threshold: 1.5,
            motor_group,
            rotation_sensor,
            gear_ratio,
        }
    }
    pub fn set_state(&mut self, state: ArmState) {
        self.state = state;
    }
    pub fn next_arm_state(&self, current_state: ArmState, button: ArmButtonCycle) -> ArmState {
        if let Some(&(_, (default_state, state_cycle))) =
            self.cycle_orders.iter().find(|&&(b, _)| b == button)
        {
            for (i, state) in state_cycle.iter().enumerate() {
                if *state == current_state {
                    return state_cycle[(i + 1) % state_cycle.len()];
                }
            }
            default_state
        } else {
            ArmState::Off
        }
    }
    pub fn opcontrol(&mut self, controller_state: Rc<ControllerState>) {
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
        } else {
            self.last_intake_button = ArmButtonCycle::None;
        }
    }
    // TODO: implement task and controller logic.
}
