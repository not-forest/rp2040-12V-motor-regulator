//! Defines a motor configuration interface, which hold the current state of the h-bridge
//! driver system and allows to modify it.

use crate::pac::{SIO, PWM, PIO0};
use crate::gpios::*;

/// Defines the current direction of the motor.
pub enum MotorDirection{ LEFT, RIGHT }

/// Defines the current configuration of the motor.
///
/// This structure holds the current state of motor's speed and direction, but also holds the
/// ownership for SIO and PWM references. 
pub struct MotorConfig {
    speed: u8,
    dir: MotorDirection,

    pub sio: Option<SIO>,
    pub pwm: Option<PWM>,
    pub pio: Option<PIO0>,
}

use MotorDirection::*;
impl MotorConfig {
    /// Returns the initialized configuration for motor driver.
    pub const fn init() -> Self {
        Self {
            speed: u8::MIN,
            dir: MotorDirection::RIGHT,
            sio: None,
            pwm: None,
            pio: None,
        }
    }

    /// Just swaps the current direction to a different one, each time when called.
    pub fn change_direction(&mut self) {
        self.dir = match self.dir {
            LEFT => RIGHT,
            RIGHT => LEFT,
        } 
    }

    /// Updates the current PWM duty cycle based on the provided closure.
    pub fn update_pwm<F>(&mut self, f: F) where 
        F: FnOnce(u8) -> u8
    {
        self.speed = f(self.speed);
        self
            .pwm
            .as_mut()
            .map(|pwm| 
                pwm.ch[0].cc.write(|w| unsafe {
                    w.a().bits(self.speed as u16)
                })
            );
    }

    /// Updates status LEDs located on the board based on the current configuration. 
    pub fn update_leds(&mut self) {
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

        self.sio
            .as_mut()
            .map(|sio| 
                sio.gpio_out.write(|w| unsafe { 
                    w.gpio_out().bits(mask) 
                })
            );
    }
}
