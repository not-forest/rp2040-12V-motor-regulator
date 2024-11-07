//! Main interrupt-driver circuit logic.
//!
//!

use core::cell::RefCell;
use crate::motor::{MotorConfig, EncoderState};
use crate::{pac, cortex};

use crate::gpios::{self, *};
use crate::pwm;
use crate::xosc;

use cortex::{
    interrupt::{self as int, Mutex}, 
    peripheral::NVIC,
};
use pac::interrupt;

/// Shared static motor config variable. Controls the PWM duty cycle and amount of speed leds to light.
static MOTOR_CFG: Mutex<RefCell<MotorConfig>> = Mutex::new(RefCell::new(MotorConfig::init()));

/// System setup function.
///
/// This function sets up the following:
/// - Swaps the clock source from internal ring oscillator to external XOCS.
/// - obtains the port peripheral and configures it for I/Os;
/// - configures the PWM to output a 20kHz regulated signal with phase correct enabled.
/// - puts the SIO interface into a mutual exclusive reference cell.
/// - unmasks required interrupts in NVIC;
pub fn setup(dp: pac::Peripherals, core: cortex::Peripherals) {
    xosc::setup(&dp);
    gpios::setup(&dp);
    pwm::setup(&dp);

    /* Storing shared resources. */
    int::free(|cs| {
        let mut motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();

        // Moving the ownership of those interfaces to the motor config.
        motor_cfg.sio.replace(dp.SIO);
        motor_cfg.pwm.replace(dp.PWM);
    });

    unsafe { NVIC::unmask(pac::interrupt::IO_IRQ_BANK0) };
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

            motor_cfg.update_pwm();
            motor_cfg.update_leds();
        });

        cortex_m::asm::wfi();
    }
}

/// GPIO interrupt handler.
///
/// Will only be called by GP15 and GP26 input pins. Handles both encoder rotation and switch
/// click. Does not change the state of output pins, because it is handleded by the main loop, only
/// mutates the motor configuration structure, according to the obtained input.
#[interrupt]
fn IO_IRQ_BANK0() {
    int::free(|cs| {
        // Only this handler uses io bank after the setup, so this is fine. 
        let io_bank = unsafe { &*pac::IO_BANK0::ptr() };
        let mut motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();

        if io_bank.proc0_ints[3].read().bits() & GPIO26_EDGE_LOW != 0 {
            // Changing the motor speed according to the encoder's rotation direction. 
            motor_cfg.update_state(EncoderState::AFALL);

            // This interrupt flag must be cleared.
            io_bank.intr[3].write(|w| unsafe { w.bits(GPIO26_EDGE_LOW) });
        }

        if io_bank.proc0_ints[3].read().bits() & GPIO27_EDGE_LOW != 0 {
            // Changing the motor speed according to the encoder's rotation direction. 
            motor_cfg.update_state(EncoderState::BFALL);

            // This interrupt flag must be cleared.
            io_bank.intr[3].write(|w| unsafe { w.bits(GPIO27_EDGE_LOW) });
        }

        if io_bank.proc0_ints[1].read().bits() & GPIO15_LEVEL_LOW != 0 {
            // Changing the motor direction.
            motor_cfg.change_direction();
        }
    });
}
