//! Defines a setup function for external clock use as a system clock.

use crate::pac;

const XOSC_STARTUP_DELAY: u16 = 47;  // (fCrystal * tStable) ÷ 256
//
/// Switches the system clock from the internal ROSC to external crystal oscillator supplied
/// with the development board. This then provides a stable 12MHz signal to the whole system,
/// which allows to calculate PWM frequency afterwards.
pub fn setup(dp: &pac::Peripherals) {
    let xosc = &dp.XOSC;
    let clocks = &dp.CLOCKS;

    // Setting the delay for 12MHz clock.
    xosc.startup.write(|w|
        w.delay().variant(XOSC_STARTUP_DELAY)
    );

    // Enabling the oscillator
    xosc.ctrl.write(|w| 
        w.freq_range()._1_15mhz()
            .enable().enable()
    );

    // Wait until the oscillator becomes stable.
    while xosc.status.read().stable().bit_is_clear() {}

    // Swapping the reference clock source and enabling the system clock.
    clocks.clk_ref_ctrl.write(|w|
        w.src().xosc_clksrc()
    );
    clocks.clk_sys_ctrl.write(|w|
        w.src().clk_ref()
    );
}
