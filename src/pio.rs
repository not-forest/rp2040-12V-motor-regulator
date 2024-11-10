//! Compiles the PIO assembler state machine program at firmware compile time and
//! initializes the PIO block to handle the rotary encoder input independently from
//! the main core.
//!
//! It acts as both a setup function for enabling the PIO peripheral and a program
//! loader function.

use crate::pac;
use crate::gpios::*;

const XOSC_CRYSTAL_FREQ: f32 = 12_000_000f32;   // RP2040 Zero external osc.
const SM_FREQ: f32 = 2_500f32;                  // Desired frequency for the state machine of 2.5 kHz
const DIVIDER_VAL: f32 =                        // fsys / fsm
    XOSC_CRYSTAL_FREQ / SM_FREQ;

const DIVIDER_INT: u16 = DIVIDER_VAL as u16;    // Truncates and strips the value to get whole part.
const DIVIDER_FRAC: u8 =                        // Obtains the fraction value at compile time with one byte precision.
        ((DIVIDER_VAL - DIVIDER_INT as f32) * 256f32) as u8;

/// Setting up the PIO0 to work as a separate program, which handles the rotary encoder
/// input signals. It will cause an interrupt on IRQ0 when counter clockwise rotation is 
/// encountered, and IRQ1 when clockwise.
pub fn setup(dp: &pac::Peripherals) {
    let reset_subsystem = &dp.RESETS;
    let pio0 = &dp.PIO0;
    let sm0 = &pio0.sm[0];

    // Compiles the assembler code into raw bytes from the file. It is equivalent of writing 
    // machine code manually to the constant array of words
    let pio_file = pio_proc::pio_file!("./src/encoder_sm.pio");

    // Deasserting the PIO peripheral. It takes longer than the others, so we shall also 
    // wait until a proper reset is done.
    reset_subsystem.reset.modify(|_, w| {
        w.pio0().clear_bit()
    });
    while reset_subsystem.reset_done.read().pio0().bit_is_clear() {}

    // Copying the compiled program to the instruction memory of the PIO0 block.
    pio0.instr_mem
        .iter()
        .take(pio_file.program.code.len())
        .enumerate()
        .for_each(|(i, reg)| 
            reg.write(|w| unsafe {
                w.instr_mem0().bits(pio_file.program.code[i])
            })
        );

    // Mapping the IN base offset for the first state machine to be APIN.
    // This will cause that BPIN is at offset 1, allowing the state machine
    // to read APIN and BPIN as two first bits respectively.
    sm0.sm_pinctrl.write(|w| unsafe { 
        w.in_base().bits(APIN as u8)
    });
    // Setting the clock divider for the state machine.
    sm0.sm_clkdiv.write(|w| unsafe { 
        w.int().bits(DIVIDER_INT)
            .frac().bits(DIVIDER_FRAC)
    });
    // Setting additional execution flags for the state machine.
    sm0.sm_execctrl.write(|w| unsafe {
        w.wrap_top().bits(pio_file.program.wrap.source)
            .wrap_bottom().bits(pio_file.program.wrap.target)
    });
    // Causing the ISR register to shift values from the LSB. This is required for the program.
    sm0.sm_shiftctrl.write(|w|
        w.in_shiftdir().clear_bit()
    );

    // Mapping external PIO IRQ0 line to the internal state machine IRQ0 line.
    pio0.sm_irq[0].irq_inte.write(|w| 
        w.sm0().set_bit()
    ); 
    // Mapping external PIO IRQ1 line to the internal state machine IRQ1 line.
    pio0.sm_irq[1].irq_inte.write(|w| 
        w.sm1().set_bit()
    ); 

    // Restarting the state machine's internal state.
    pio0.ctrl.modify(|_, w| unsafe {
        w.bits(1 << 4)
    });
    // Restarting state machine's clock divider from an initial phase of 0.
    pio0.ctrl.modify(|_, w| unsafe {
        w.bits(1 << 8)
    });

    // Force the state machine to jump to the program start.
    sm0.sm_instr.write(|w| unsafe {
        w.sm0_instr().bits(pio_file.public_defines.entry as u16)
    });

    // Enabling the state machine's logic.
    pio0.ctrl.modify(|_, w| unsafe {
        w.bits(1)
    });
}
