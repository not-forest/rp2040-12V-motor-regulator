//! Main code entry area.
//!
//! RP2040 HAL libraries handles the bootloading part. The whole system logic is implemented within
//! the handlers.rs module.
#![no_std]
#![no_main]

mod gpios;
mod pwm;

mod handlers;
mod motor;
mod xosc;

pub use cortex_m as cortex;
pub use rp2040_hal::pac as pac;

// Panic handler.
panic_custom::define_panic!(|_| ());

/// Second stage bootloader.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rp2040_hal::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let core = cortex::Peripherals::take().unwrap();
    
    // Overall system setup.
    handlers::setup(dp, core);
    handlers::main();
}
