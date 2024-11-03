//! Main interrupt-driver circuit logic.

use core::cell::RefCell;
use crate::gpios::*;
use crate::motor::MotorConfig;

pub use cortex_m as cortex;
pub use rp2040_hal::pac as pac;

use cortex::{
    interrupt::{self as int, Mutex}, 
    peripheral::NVIC,
};
use pac::interrupt;

// Panic handler.
panic_custom::define_panic!(|_| ());

/// Second stage bootloader.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Shared SIO Interface block.
static SIO_INT: Mutex<RefCell<Option<pac::SIO>>> = Mutex::new(RefCell::new(None));
/// Shared static motor config variable. Controls the PWM duty cycle and amount of speed leds to light.
static MOTOR_CFG: Mutex<RefCell<MotorConfig>> = Mutex::new(RefCell::new(MotorConfig::init()));

/// System setup function.
///
/// This function sets up the following:
/// - enables watchdog timer;
/// - obtains the port peripheral and configures it for I/Os;
/// - unmasks required interrupts in NVIC;
/// - puts the SIO interface into a mutual exclusive reference cell.
pub fn setup(dp: pac::Peripherals, core: cortex::Peripherals) {
    gpio_setup(&dp);

    /* Final interrupt free setup closure. */
    int::free(|cs| unsafe {
        SIO_INT.borrow(cs).replace(Some(dp.SIO));

        NVIC::unmask(pac::interrupt::IO_IRQ_BANK0);
    });
}

/// System loop function.
///
/// Will be called after interrupt handler is done, therefore used to change
/// the state of LEDs.
pub fn main() -> ! {
    loop {
        // Changing the leds status according to the new configuration.
        int::free(|cs| {
            let motor_cfg = MOTOR_CFG.borrow(cs).borrow_mut();
            let mut siorf = SIO_INT.borrow(cs).borrow_mut();

            if let Some(sio) = siorf.as_mut() {
                sio.gpio_out.write(|w| unsafe { 
                    w.gpio_out().bits(motor_cfg.leds_state_mask()) 
                });
            }
        });

        cortex_m::asm::wfi();
    }
}

/// GPIO configuration part.
fn gpio_setup(dp: &pac::Peripherals) { 
    let io_bank = &dp.IO_BANK0;
    let pads_bank = &dp.PADS_BANK0;
    let sio = &dp.SIO;
    let reset_subsystem = &dp.RESETS;

    // Making sure no bits are remembered from previous firmware.
    sio.gpio_oe.reset();
    sio.gpio_out.reset();

    // Deasserting the peripheral out of the reset state via reset subsystem.
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
        sio.gpio_oe_set.write(|w| unsafe { w.bits(1 << pin) });
    });
    // Configuration of all input pins.
    INPUT_GPIOS.into_iter().for_each(|pin| {
        io_bank.gpio[pin].gpio_ctrl.write(|w|
            w.funcsel().sio()
                .oeover().disable()
        );
    
        // None of the input pins shall have internal pulldowns.
        pads_bank.gpio[pin].write(|w|
            w.pde().clear_bit()
        );

        // Disabling the output driver via GPIO_OUT registers.
        sio.gpio_oe_clr.write(|w| unsafe { w.bits(1 << pin) });
    });

    // PWM pin is connected to the PWM peripheral.
    io_bank.gpio[MPIN_PWM].gpio_ctrl.write(|w| 
        w.funcsel().pwm()
    );

    // Enabling internal pullup on SW pin, since switch is floating.
    pads_bank.gpio[SW].write(|w| 
        w.pde().clear_bit()
            .pue().set_bit()
    );

    // Enables interrupt on two input pins.
    //
    // GP15 → Generates interrupt when LOW (Switch on rotary encoder is used.)
    // GP26 → Generates interrupt on falling edge. (Rotary encoder is rotated.)
    io_bank.proc0_inte[1].write(|w| unsafe {w.bits(GPIO15_LEVEL_LOW)});
    io_bank.proc0_inte[3].write(|w| unsafe {w.bits(GPIO26_EDGE_LOW)});
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
        let siorf = SIO_INT.borrow(cs).borrow();

        if let Some(sio) = siorf.as_ref() {
            // Checking the interrupt source.
            if io_bank.proc0_ints[1].read().bits() & GPIO15_LEVEL_LOW != 0 {
                // Changing the motor direction.
                motor_cfg.change_direction();
            } else if io_bank.proc0_ints[3].read().bits() & GPIO26_EDGE_LOW != 0 {
                // Changing the motor speed according to the encoder's rotation direction. 
                motor_cfg.adjust_speed(&sio);

                // This interrupt flag must be cleared.
                io_bank.intr[3].write(|w| unsafe { w.bits(GPIO26_EDGE_LOW) });
            }
        }
    });
}
