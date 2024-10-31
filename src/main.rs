//! Main code entry area.
//!
//! Since the whole system is interrupt-driven, performs a setup procedure and halts.
#![no_std]
#![no_main]

mod handlers;

#[rp2040_hal::entry]
fn main() -> ! {
    defmt::info!("Hello world.");
    loop {}
}
