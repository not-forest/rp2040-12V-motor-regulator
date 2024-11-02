//! Main code entry area.
//!
//! RP2040 HAL libraries handles the bootloading part. The whole system logic is implemented within
//! the handlers.rs module.
#![no_std]
#![no_main]

mod handlers;
mod motor;
mod gpios;

#[rp2040_hal::entry]
fn main() -> ! {
    let dp = handlers::pac::Peripherals::take().unwrap();
    let core = handlers::cortex::Peripherals::take().unwrap();
    
    // Overall system setup.
    handlers::setup(dp, core);
    handlers::main();
}
