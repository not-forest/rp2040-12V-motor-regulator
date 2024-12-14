//! Defines GPIO setup function for the circuit application.
//!
//! Each constant value has an underlying pin number, corresponding to the one defined
//! in the schematic circuit.

use crate::pac;

/// Controls motor's speed via PWM.
const MPIN_PWM: usize       = 0;
/// Controls motor's rotation direction. Rotates anti-clockwise when '1'.
pub const MPIN_A: usize     = 1;
/// Controls motor's rotation direction. Rotates clockwise when '1'.
pub const MPIN_B: usize     = 6;
/// Speed status LED 1.
pub const S1: usize         = 8;
/// Speed status LED 2.
pub const S2: usize         = 9;
/// Speed status LED 3.
pub const S3: usize         = 10;
/// Speed status LED 4.
pub const S4: usize         = 11;
/// Speed status LED 5.
pub const S5: usize         = 12;
/// Left motor's rotation direction status LED.
pub const LEFT_PIN: usize   = 13;
/// Right motor's rotation direction status LED.
pub const RIGHT_PIN: usize  = 14;

/// Pulled-up SPDT switch input from the rotary encoder.
const SW: usize             = 15;
/// Oncoming square signal from the rotary encoder pin A.
pub const APIN: usize       = 26;
/// Oncoming square signal from the rotary encoder pin B.
pub const BPIN: usize       = 27;

/// Circuit output GPIOS connected to SIO.
const OUTPUT_GPIOS: [usize; 9] = [MPIN_A, MPIN_B, S1, S2, S3, S4, S5, LEFT_PIN, RIGHT_PIN];
/// Circuit input GPIOS connected to SIO.
const INPUT_GPIOS: [usize; 3] = [SW, APIN, BPIN];

pub const GPIO15_EDGE_HIGH: u32 = 1 << 31;

/// GPIO configuration part.
pub fn setup(dp: &pac::Peripherals) { 
    let io_bank = &dp.IO_BANK0;
    let pads_bank = &dp.PADS_BANK0;
    let sio = &dp.SIO;
    let reset_subsystem = &dp.RESETS;

    // Making sure no bits are remembered from previous firmware.
    sio.gpio_oe.reset();
    sio.gpio_out.reset();

    // Deasserting GPIO related peripherals out of the reset state.
    reset_subsystem.reset.modify(|_, w| 
        w.io_bank0().clear_bit()
            .pads_bank0().clear_bit()
    );

    // Configuration of all output pins.
    OUTPUT_GPIOS.into_iter().for_each(|pin| {
        io_bank.gpio[pin].gpio_ctrl.write(|w| 
            w.funcsel().sio()
                .oeover().enable()
        );
        // Enabling the output driver via GPIO_OUT registers.
        sio.gpio_oe_set.write(|w| w.gpio_oe_set().variant(1 << pin));
    });
    // Configuration of all input pins.
    INPUT_GPIOS.into_iter().for_each(|pin| {
        // None of the input pins shall have internal pulldowns.
        pads_bank.gpio[pin].write(|w|
            w.pde().clear_bit()
        );

        io_bank.gpio[pin].gpio_ctrl.write(|w|
            w.oeover().disable()
        );

        // Disabling the output driver via GPIO_OUT registers.
        sio.gpio_oe_clr.write(|w| w.gpio_oe_clr().variant(1 << pin));
    });

    // PWM pin is connected to the PWM peripheral.
    io_bank.gpio[MPIN_PWM].gpio_ctrl.write(|w| 
        w.funcsel().pwm()
            .oeover().enable()
    );

    // Connecting APIN and BPIN to the PIO0 block for encoder control.
    [APIN, BPIN].into_iter().for_each(|pin| 
        io_bank.gpio[pin].gpio_ctrl.write(|w|
            w.funcsel().pio0()
                .oeover().disable()
        )
    );

    // Enabling internal pullup on SW pin, since switch is floating.
    pads_bank.gpio[SW].write(|w| 
        w.pde().clear_bit()
            .pue().set_bit()
    );

    // Enables interrupt on SW pin: GP15 will generate an interrupt 
    // when LOW-to-HIGH translation (Switch on rotary encoder is used.)
    io_bank.proc0_inte[1].write(|w| unsafe {w.bits(GPIO15_EDGE_HIGH)});
}
