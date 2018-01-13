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
pub struct FlashAddress(pub u32);

#[derive(Debug)]
pub enum Error {
    /// A hardware error occured. The value is the
    /// contents of the FCRIS register.
    HardwareError(u32),
    /// Given address did not have the alignment
    /// specified here (as a u32).
    BadAlignment(u32),
    /// Buffer given was too long, it should be the length given.
    BufferTooLong(usize),
    /// Buffer given was too short, it should be the length given.
    BufferTooShort(usize),
}

#[derive(Debug)]
pub enum ProtectMode {
    ExecuteOnly,
    ReadOnly,
    ReadWrite
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
// const FLASH_KEY: u16 = 0x71D5;
const FLASH_KEY: u16 = 0xA442;

const FLASH_PROTECT_BLOCK_SIZE: u32 = 2048;
const FLASH_PROTECT_NUM_BLOCKS: u32 = 32;
const FLASH_PROTECT_BANK_SIZE: u32 = FLASH_PROTECT_BLOCK_SIZE * FLASH_PROTECT_NUM_BLOCKS;
const FLASH_PROTECT_NUM_BANKS: u32 = 4;

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// Erase a 1 KiB page of flash. The given address must be on 1 KiB boundary
/// (i.e. a multiple of 1024).
pub fn erase_page(address: FlashAddress) -> Result<(), Error> {
    if (address.0 & 1023) != 0 {
        return Err(Error::BadAlignment(1024));
    }

    unsafe {
        let reg = get_registers();
        clear_bits(reg);
        // Write the page address to the FMA register
        reg.fma.write(|w| w.offset().bits(address.0));
        // Write the flash memory key and the erase bit
        reg.fmc.modify(
            |_, w| w.erase().bit(true).wrkey().bits(FLASH_KEY),
        );
        // Poll the FMC register until the ERASE bit is cleared
        while reg.fmc.read().erase().bit() {
            nop();
        }
        get_status(reg)
    }
}

/// Write a 32-bit value to flash at the given address. The address
/// must be on a 4-byte boundary (i.e. a multiple of 4).
pub fn write_word(address: FlashAddress, word: u32) -> Result<(), Error> {
    if (address.0 & 3) != 0 {
        return Err(Error::BadAlignment(4));
    }

    unsafe {
        let reg = get_registers();
        clear_bits(reg);
        // Write the target address to the FMA register
        reg.fma.write(|w| w.offset().bits(address.0));
        // Write the data to the FMD register
        reg.fmd.write(|w| w.data().bits(word));
        // Write the flash memory key and the write bit
        reg.fmc.modify(
            |_, w| w.write().bit(true).wrkey().bits(FLASH_KEY),
        );
        // Poll the FMC register until the WRITE bit is cleared
        while reg.fmc.read().write().bit() {
            nop();
        }
        get_status(reg)
    }
}

/// Get the protection status of the block containing the given address.
pub fn get_protection(address: FlashAddress) -> ProtectMode {
    let bank = (address.0 / FLASH_PROTECT_BANK_SIZE) % FLASH_PROTECT_NUM_BANKS;
    let bank_offset = address.0 & (FLASH_PROTECT_BANK_SIZE - 1);
    let block = bank_offset / FLASH_PROTECT_BLOCK_SIZE;
    let reg = unsafe {
        get_registers()
    };

    let read_bits = match bank {
        0 => reg.fmpre0.read().bits(),
        1 => reg.fmpre1.read().bits(),
        2 => reg.fmpre2.read().bits(),
        3 => reg.fmpre3.read().bits(),
        _ => unreachable!(),
    };
    let exec_bits = match bank {
        0 => reg.fmppe0.read().bits(),
        1 => reg.fmppe1.read().bits(),
        2 => reg.fmppe2.read().bits(),
        3 => reg.fmppe3.read().bits(),
        _ => unreachable!(),
    };
    let read_enabled = (read_bits & (1 << block)) == 1;
    let exec_enabled = (exec_bits & (1 << block)) == 1;

    if read_enabled {
        if exec_enabled {
            ProtectMode::ReadWrite
        } else {
            ProtectMode::ReadOnly
        }
    } else {
        ProtectMode::ExecuteOnly
    }

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

/// Clear all the status bits
fn clear_bits(reg: &'static tm4c123x::flash_ctrl::RegisterBlock) {
    reg.fcmisc.write(|w| {
        w.amisc()
            .bit(true)
            .voltmisc()
            .bit(true)
            .invdmisc()
            .bit(true)
            .progmisc()
            .bit(true)
            .ermisc()
            .bit(true)
    });
}

/// Get pass/fail from the controller
fn get_status(reg: &'static tm4c123x::flash_ctrl::RegisterBlock) -> Result<(), Error> {
    let fcris = reg.fcris.read();
    if fcris.aris().bit() || fcris.voltris().bit() || fcris.invdris().bit() ||
        fcris.progris().bit() || fcris.erris().bit()
    {
        Err(Error::HardwareError(fcris.bits()))
    } else {
        Ok(())
    }
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
