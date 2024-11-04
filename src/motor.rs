//! Defines motor configuration related structures.

use crate::handlers::pac::SIO;
use crate::gpios::*;

/// Defines the current direction of the motor.
pub enum MotorDirection{ LEFT, RIGHT }

/// Defines a rotaty encoder as a state machine.
pub enum EncoderState { IDLE, AFALL, BFALL }

/// Defines the current configuration of the motor.
pub struct MotorConfig {
    pub speed: u8,
    state: EncoderState,
    dir: MotorDirection,
}

use MotorDirection::*;
impl MotorConfig {
    /// Returns the initialized configuration for motor driver.
    pub const fn init() -> Self {
        Self {
            speed: u8::MIN,
            state: EncoderState::IDLE,
            dir: MotorDirection::RIGHT,
        }
    }

    /// Updates the state machine's state and increases or descreases the internal
    /// speed counter if a certain state combination is seen.
    pub fn update_state(&mut self, new_state: EncoderState) {
        use EncoderState::*;
        const MOTOR_SPEED_GAIN: u8 = 5;

        match new_state {
            AFALL => match self.state {
                IDLE => self.state = AFALL,
                AFALL => (),
                BFALL => {
                    self.speed = self.speed.saturating_sub(MOTOR_SPEED_GAIN);
                    self.state = IDLE;
                }
            },
            BFALL => match self.state {
                IDLE => self.state = BFALL,
                BFALL => (),
                AFALL => {
                    self.speed = self.speed.saturating_add(MOTOR_SPEED_GAIN);
                    self.state = IDLE;
                }
            },
            _ => unreachable!(),
        }
    }

    /// Just swaps the current direction to a different one, each time when called.
    pub fn change_direction(&mut self) {
        self.dir = match self.dir {
            LEFT => RIGHT,
            RIGHT => LEFT,
        } 
    }


    /// Updates status LEDs locatec on the board based on the current configuration. 
    pub fn update_leds(&self, sio: &mut SIO) {
        const MOTOR_SPEED_STEP: u8 = u8::MAX / 6;   // 6 possible states.
        const MOTOR_SPEED_LEDS: [usize; 5] = [S1, S2, S3, S4, S5];

        let mut mask: u32 = 0; 

        // Writing values on output pins according to the current direction.
        match self.dir {
            LEFT => mask |= (1 << LEFT_PIN) | (1 << MPIN_A),
            RIGHT => mask |= (1 << RIGHT_PIN) | (1 << MPIN_B),
        }
    
        // Writing values to the motor speed pins according to the current speed.
        MOTOR_SPEED_LEDS
            .into_iter()
            .take((self.speed / MOTOR_SPEED_STEP) as usize)
            .for_each(|led| mask |= 1 << (led as u32));

        sio.gpio_out.write(|w| unsafe { 
            w.gpio_out().bits(mask) 
        });
    }
}
