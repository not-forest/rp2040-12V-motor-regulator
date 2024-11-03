//! Main code entry area.
//!
//! RP2040 HAL libraries handles the bootloading part. The whole system logic is implemented within
//! the handlers.rs module.
#![no_std]
#![no_main]

mod handlers;
mod motor;
mod gpios;

// Panic handler.
panic_custom::define_panic!(|_| ());

/// Second stage bootloader.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

#[rp2040_hal::entry]
fn main() -> ! {
    let dp = handlers::pac::Peripherals::take().unwrap();
    let core = handlers::cortex::Peripherals::take().unwrap();
    
    // Overall system setup.
    handlers::setup(dp, core);
    handlers::main();
}
