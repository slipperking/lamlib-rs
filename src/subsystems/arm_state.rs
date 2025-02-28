use alloc::rc::Rc;
use core::{cell::RefCell, time::Duration};

use vexide::prelude::RotationSensor;

use crate::{controllers::ControllerMethod, devices::motor_group::MotorGroup};

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
}

impl<'a> ArmStateMachine<'a> {
    pub fn new(
        motor_group: Rc<RefCell<MotorGroup>>,
        rotation_sensor: Rc<RefCell<RotationSensor>>,
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
        }
    }
}
