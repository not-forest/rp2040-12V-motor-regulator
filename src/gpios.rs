//! Defines GPIO enums and their logical use within this system.
//!
//! Each constant value has an underlying pin number, corresponding to the one defined
//! in the schematic circuit.

/// Controls motor's speed via PWM.
pub const MPIN_PWM: usize   = 0;
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
pub const SW: usize         = 15;
/// Oncoming square signal from the rotary encoder pin A.
pub const APIN: usize       = 26;
/// Oncoming square signal from the rotary encoder pin B.
pub const BPIN: usize       = 27;

/// Circuit output GPIOS connected to SIO.
pub const OUTPUT_GPIOS: [usize; 9] = [MPIN_A, MPIN_B, S1, S2, S3, S4, S5, LEFT_PIN, RIGHT_PIN];
/// Circuit input GPIOS connected to SIO.
pub const INPUT_GPIOS: [usize; 3] = [SW, APIN, BPIN];

/* Used to enable interrupts on certain gpio pins. */
pub const GPIO15_LEVEL_LOW: u32 = 1 << 28;
pub const GPIO26_EDGE_LOW: u32 = 1 << 10;
pub const GPIO27_EDGE_LOW: u32 = 1 << 14;
