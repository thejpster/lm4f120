//! # Flash drivers for the LM4F120H5QR
//!
//! The LM4F120 has a flash memory controller which supports:
//!
//! * 1 KiB page erase
//! * 32-bit word write
//! * 32 word (128 byte) block write

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

use cortex_m::asm::nop;
use tm4c123x;

// ****************************************************************************
//
// Public Types
//
// ****************************************************************************

/// Represents an address in Flash memory. As flash memory starts at
/// 0x0000_0000, flash addresses are equal to CPU physical addresses.
pub struct FlashAddress(u32);

pub enum Error {
    /// Given address did not have the alignment
    /// specified here (as a u32).
    BadAlignment(u32),
    /// Buffer given was too long, it should be the length given.
    BufferTooLong(usize),
    /// Buffer given was too short, it should be the length given.
    BufferTooShort(usize),
}

// ****************************************************************************
//
// Public Data
//
// ****************************************************************************

// None

// ****************************************************************************
//
// Private Types
//
// ****************************************************************************

// None

// ****************************************************************************
//
// Private Data
//
// ****************************************************************************

// const PAGE_LENGTH: usize = 128;
const FLASH_KEY: u16 = 0x71D5;
// const FLASH_KEY: u16 = 0xA442;

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// Erase a 1 KiB page of flash. The given address must be on 1 KiB boundary
/// (i.e. a multiple of 1024).
pub fn erase_page(address: FlashAddress) -> Result<(), Error> {
    if (address.0 & 1023) != 0 {
        return Err(Error::BadAlignment(1024))
    }

    unsafe {
        let reg = get_registers();
        // Write the page address to the FMA register
        reg.fma.write(|w| w.offset().bits(address.0));
        // Write the flash memory key and the erase bit
        reg.fmc.modify(|_, w| w.erase().bit(true).wrkey().bits(FLASH_KEY));
        // Poll the FMC register until the ERASE bit is cleared
        while reg.fmc.read().erase().bit() {
            nop();
        }
    }

    Ok(())
}

/// Write a 32-bit value to flash at the given address. The address
/// must be on a 4-byte boundary (i.e. a multiple of 4).
pub fn write_word(address: FlashAddress, word: u32) -> Result<(), Error> {
    if (address.0 & 3) != 0 {
        return Err(Error::BadAlignment(4))
    }

    unsafe {
        let reg = get_registers();
        // Write the data to the FMD register
        reg.fmd.write(|w| w.data().bits(word));
        // Write the target address to the FMA register
        reg.fma.write(|w| w.offset().bits(address.0));
        // Write the flash memory key and the write bit
        reg.fmc.modify(|_, w| w.write().bit(true).wrkey().bits(FLASH_KEY));
        // Poll the FMC register until the WRITE bit is cleared
        while reg.fmc.read().write().bit() {
            nop();
        }
    }

    Ok(())

}

/// Write a 128 byte buffer to flash at the given address. The address
/// must be on a 128-byte boundary (i.e. a multiple of 128) and the
/// buffer must have a length equal to 128. Pad with 0xFF if
/// your data is short.
// pub fn write_page(address: FlashAddress, data: &[u8]) -> Result<(), Error> {
//     if (address.0 & 127) != 0 {
//         return Err(Error::BadAlignment(128));
//     } else if data.len() < PAGE_LENGTH {
//         return Err(Error::BufferTooShort(PAGE_LENGTH));
//     } else if data.len() > PAGE_LENGTH {
//         return Err(Error::BufferTooLong(PAGE_LENGTH));
//     }

//     unsafe {
//         let reg = get_registers();
//         // Write the data to the FMBx registers
//         // I'm not sure the tmc4c123x crate has these correct, as it only
//         // exposes a single fmbn, not 32 fmbx registers.
//         reg.fmd.write(|w| w.data().bits(word));
//         // Write the target address to the FMA register
//         reg.fma.write(|w| w.offset().bits(address.0));
//         // Write the flash memory key and the write bit
//         reg.fmc.modify(|_, w| w.write().bit(true).wrkey().bits(FLASH_KEY));
//         // Poll the FMC register until the WRITE bit is cleared
//         while reg.fmc.read().write().bit() {
//             nop();
//         }
//     }

//     Ok(())

// }

// ****************************************************************************
//
// Private Functions
//
// ****************************************************************************

/// Get a reference to the UART control register struct in the chip.
unsafe fn get_registers() -> &'static tm4c123x::flash_ctrl::RegisterBlock {
    &*tm4c123x::FLASH_CTRL.get()
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
