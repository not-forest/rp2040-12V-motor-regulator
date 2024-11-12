//! Defines a configuration function for PWM support.

use crate::pac;

const PWM_TOP: u16 = u8::MAX as u16;
const XOSC_CRYSTAL_FREQ: f32 = 12_000_000f32;                       // RP2040 Zero external osc.
const MOTOR_PWM_FREQ: f32 = 20_000f32;                              // Desired PWM output frequency.
const MOTOR_PWM_PHASE_CORRECT_FREQ: f32 = MOTOR_PWM_FREQ * 2f32;    // Before the phase correct.
const DIVIDER_VAL: f32 = // This follows the formula: fsys / (fpwm * TOP+1) 
XOSC_CRYSTAL_FREQ / (MOTOR_PWM_PHASE_CORRECT_FREQ * (PWM_TOP as f32 + 1f32));

const DIVIDER_INT: u8 = DIVIDER_VAL as u8;  // Truncates and strips the value to get whole part.
const DIVIDER_FRAC: u8 =                    // Obtains the fraction value at compile time with 4-bit precision.
((DIVIDER_VAL - DIVIDER_INT as f32) * 16f32) as u8;

/// PWM setup function for GP0.
///
/// Sets up the PWM control to output a 20kHz output frequency to feed the
/// motor vis H-Bridge circuit. All constant calculations are stripped at
/// compile time.
pub fn setup(dp: &pac::Peripherals) {
    let pwmch0 = &dp.PWM.ch[0];
    let reset_subsystem = &dp.RESETS;

    // Deasserting the PWM peripheral.
    reset_subsystem.reset.modify(|_, w| {
        w.pwm().clear_bit()
    });

    // Divider value.
    pwmch0.div.write(|w|
        w.int().variant(DIVIDER_INT)
            .frac().variant(DIVIDER_FRAC)
    );

    // Counter TOP value.
    pwmch0.top.write(|w| 
        w.top().variant(PWM_TOP)
    );

    // Enabling the PWM with phase correct.
    pwmch0.csr.write(|w| 
        w.en().set_bit()
            .ph_correct().set_bit()
    );
}
