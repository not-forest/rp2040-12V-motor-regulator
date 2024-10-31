//! Main interrupt-driver circuit logic.
//!
//!

use panic_probe as _;

/// Second stage bootloader.
#[link_section = ".boot2"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
