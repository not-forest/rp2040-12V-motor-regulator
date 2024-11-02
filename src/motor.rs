//! Defines motor configuration related structures.

use crate::handlers::pac::SIO;
use crate::gpios::*;

/// Defines the current direction of the motor.
pub enum MotorDirection{ LEFT, RIGHT }

/// Defines the current configuration of the motor.
pub struct MotorConfig {
    speed: u8,
    dir: MotorDirection,
}

use MotorDirection::*;
impl MotorConfig {
    /// Returns the initialized configuration for motor driver.
    pub const fn init() -> Self {
        Self {
            speed: u8::MIN,
            dir: MotorDirection::RIGHT,
        }
    }

    /// Adjusts the motor speed, based on the values between Apin and Bpin.
    ///
    /// Since falling edge on Apin generates the interrupt, we adjust the speed accordingly
    /// to the current state of Bpin.
    /// - Bpin == '1' → the encoder is rotated clockwise and the speed shall be increased;
    /// - Bpin == '0' → the encoder is rotated counter clockwise and the speed shall be decreased;
    pub fn adjust_speed(&mut self, sio: &SIO) {
        const MOTOR_SPEED_GAIN: u8 = 1;

        if sio.gpio_in.read().bits() & (1 << BPIN) == 1 {
            self.speed = self.speed.saturating_add(MOTOR_SPEED_GAIN);
        } else {
            self.speed = self.speed.saturating_sub(MOTOR_SPEED_GAIN);
        }
    }

    /// Just swaps the current direction to a different one, each time when called.
    pub fn change_direction(&mut self) {
        self.dir = match self.dir {
            LEFT => RIGHT,
            RIGHT => LEFT,
        } 
    }

    /// Returns a value to read into the output SIO register, based on current config state.
    pub fn leds_state_mask(&self) -> u32 {
        const MOTOR_SPEED_STEP: u8 = u8::MAX / 5;
        const MOTOR_SPEED_LEDS: [usize; 5] = [S1, S2, S3, S4, S5];

        let mut out: u32 = 0; 

        // Writing values on output pins according to the current direction.
        match self.dir {
            LEFT => out |= (1 << LEFT_PIN) | (1 << MPIN_A),
            RIGHT => out |= (1 << RIGHT_PIN) | (1 << MPIN_B),
        }
    
        // Writing values to the motor speed pins according to the current speed.
        MOTOR_SPEED_LEDS
            .into_iter()
            .take((self.speed / MOTOR_SPEED_STEP) as usize)
            .for_each(|led| out |= 1 << (led as u32));

        out
    }
}
