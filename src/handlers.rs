//! Main interrupt-driver system logic.
//!
//! Mutates the current state of the system when one of three possible interrupts are encountered.
//! Adjusts status leds in the main program loop after each interrupt.

use core::cell::RefCell;
use crate::motor::MotorConfig;
use crate::{pac, cortex};

use crate::{gpios::{self, *}, pwm, xosc, pio};

use cortex::{
    interrupt::{self as int, Mutex}, 
    peripheral::NVIC,
};
use pac::interrupt;

// Motor speed added to the current spead on each encoder's rotation.
const MOTOR_GAIN_SPEED: u8 = 5;

/// Shared static motor config variable. Controls the PWM duty cycle and amount of speed leds to light.
static MOTOR_CFG: Mutex<RefCell<MotorConfig>> = Mutex::new(RefCell::new(MotorConfig::init()));

/// System setup function.
///
/// This function sets up the following:
/// - Swaps the clock source from internal ring oscillator to external XOCS.
/// - obtains the gpio port peripheral and configures it for I/Os;
/// - loads the program to the pio state machine for rotary encoder management.
/// - configures the PWM to output a 20kHz regulated signal with phase correct enabled.
/// - puts the SIO interface into a mutual exclusive reference cell.
/// - unmasks required interrupts in NVIC;
pub fn setup(dp: pac::Peripherals) {
    xosc::setup(&dp);
    gpios::setup(&dp);
    pio::setup(&dp);
    pwm::setup(&dp);

    /* Storing shared resources. */
    int::free(|cs| {
        let mut motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();

        // Moving the ownership of those interfaces to the motor config.
        motor_cfg.sio.replace(dp.SIO);
        motor_cfg.pwm.replace(dp.PWM);
        motor_cfg.pio.replace(dp.PIO0);
    });

    /* Enabling interrupts. */
    unsafe { 
        NVIC::unmask(pac::interrupt::IO_IRQ_BANK0); 
        NVIC::unmask(pac::interrupt::PIO0_IRQ_0);
        NVIC::unmask(pac::interrupt::PIO0_IRQ_1);
    };
}

/// System loop function.
///
/// Will be called after interrupt handler is done, therefore used to change
/// the state of LEDs.
pub fn main() -> ! {
    loop {
        // Changing the leds status according to the new configuration.
        int::free(|cs| {
            let mut motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();

            motor_cfg.update_leds();
        });

        cortex_m::asm::wfi();
    }
}

/// GPIO interrupt handler.
///
/// Will only be called by GPIO15 click input.
#[interrupt]
fn IO_IRQ_BANK0() {
    // IO bank is not shared between other interrupts, so this is safe. 
    let io_bank = unsafe { &*pac::IO_BANK0::ptr() };

    int::free(|cs| {
        let mut motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();

        if io_bank.proc0_ints[1].read().bits() & GPIO15_EDGE_HIGH != 0 {
            // Changing the motor direction.
            motor_cfg.change_direction();
            cortex_m::asm::delay(12000); // This will cause one ms. debounce delay
            io_bank.intr[1].write(|w| unsafe { w.bits(GPIO15_EDGE_HIGH) });
        }
    });
}

/// PIO0 IRQ0 interrupt.
///
/// Caused by PIO internal program when the rotary encoder is rotated counter clockwise.
#[interrupt]
fn PIO0_IRQ_0() {
    int::free(|cs| {
        let mut motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();
        // Decrementing the PWM duty cycle.
        motor_cfg.update_pwm(|s| s.saturating_sub(MOTOR_GAIN_SPEED));
        
        // IRQ0 must be cleared.
        motor_cfg.pio
            .as_ref()
            .map(|pio0|
                pio0.irq.write(|w| w.irq().variant(1 << 0))
            );
    });
}

/// PIO0 IRQ1 interrupt.
///
/// Caused by PIO internal program when the rotary encoder is rotated clockwise.
#[interrupt]
fn PIO0_IRQ_1() {
    int::free(|cs| {
        let mut motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();
        // Incrementing the PWM duty cycle.
        motor_cfg.update_pwm(|s| s.saturating_add(MOTOR_GAIN_SPEED));
        
        // IRQ1 must be cleared.
        motor_cfg.pio
            .as_ref()
            .map(|pio0| 
                pio0.irq.write(|w| w.irq().variant(1 << 1))
            );
    });
}
