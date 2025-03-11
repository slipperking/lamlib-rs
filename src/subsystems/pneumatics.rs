use alloc::rc::Rc;
use core::cell::RefCell;

use vexide::{devices::adi::digital::LogicLevel, prelude::AdiDigitalOut};

#[derive(PartialEq)]
enum PneumaticButtonState {
    None,
    High,
    Low,
    Toggle,
}

/// A wrapper for an adi digital out as for pneumatic solenoids.
/// 
/// Allows for boolean control and repeatable driver control schemes.
pub struct PneumaticWrapper {
    pneumatic: Rc<RefCell<AdiDigitalOut>>,
    previous_button_state: PneumaticButtonState,
}

impl PneumaticWrapper {
    pub fn new(pneumatic: Rc<RefCell<AdiDigitalOut>>) -> Self {
        Self {
            pneumatic,
            previous_button_state: PneumaticButtonState::None,
        }
    }
    pub fn set_level(&self, level: LogicLevel) {
        let _ = self.pneumatic.borrow_mut().set_level(level);
    }
    pub fn set_state(&self, level: bool) {
        let _ = self.pneumatic.borrow_mut().set_level(if level {
            LogicLevel::High
        } else {
            LogicLevel::Low
        });
    }
    /// A driver scheme that uses a button to toggle the logic level of the digital out.
    pub fn driver_toggle(&mut self, button_state: bool) {
        if button_state {
            if self.previous_button_state != PneumaticButtonState::Toggle {
                self.previous_button_state = PneumaticButtonState::Toggle;
                let _ = self.pneumatic.borrow_mut().toggle();
            }
        } else {
            self.previous_button_state = PneumaticButtonState::None;
        }
    }
    /// A driver scheme using two buttons, one for each logic level.
    /// 
    /// The `high_priority_button_state` and `low_priority_button_state` are the
    /// button states of high and low priority. Priority is used to determine 
    /// which button state to apply when both buttons are pressed.
    /// The logic levels refer to what logic level each button state maps to.
    pub fn driver_explicit(
        &mut self,
        high_priority_button_state: bool,
        low_priority_button_state: bool,
        high_priority_logic_level: LogicLevel,
        low_priority_logic_level: LogicLevel,
    ) {
        if high_priority_button_state {
            if self.previous_button_state != PneumaticButtonState::High {
                self.previous_button_state = PneumaticButtonState::High;
                let _ = self
                    .pneumatic
                    .borrow_mut()
                    .set_level(high_priority_logic_level);
            }
        } else if low_priority_button_state {
            if self.previous_button_state != PneumaticButtonState::Low {
                self.previous_button_state = PneumaticButtonState::Low;
                let _ = self
                    .pneumatic
                    .borrow_mut()
                    .set_level(low_priority_logic_level);
            }
        } else {
            self.previous_button_state = PneumaticButtonState::None;
        }
    }
}
