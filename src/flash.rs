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
use core::fmt;
use core::intrinsics::volatile_store;

// ****************************************************************************
//
// Public Types
//
// ****************************************************************************

/// Represents an address in Flash memory. As flash memory starts at
/// 0x0000_0000, flash addresses are equal to CPU physical addresses.
#[derive(Clone, Copy)]
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
    ReadWrite,
}

// ****************************************************************************
//
// Public Data
//
// ****************************************************************************

/// Maximum number of u32s we can write to flash in one go.
pub const PAGE_LENGTH_WORDS: usize = 32;

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
        reg.fmc
            .modify(|_, w| w.erase().bit(true).wrkey().bits(FLASH_KEY));
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
        // Write the data to the buffer
        volatile_store(0x400FD100 as *mut u32, word);
        // Write the flash memory key and the write bit
        volatile_store(0x400FD020 as *mut u32, 0xA4420001);
        // Poll the FMC register until the WRITE bit is cleared
        while reg.fmc2.read().wrbuf().bit() {
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
    let reg = unsafe { get_registers() };

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
    let read_enabled = (read_bits & (1 << block)) != 0;
    let exec_enabled = (exec_bits & (1 << block)) != 0;

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

/// Write a <= 128 byte (<= 32 word) buffer to flash at the given address. The
/// address must be on a 4-byte boundary and the buffer must have a length
/// less than or equal to 32 words.
pub fn write_page<I>(address: FlashAddress, data: I) -> Result<(), Error>
where
    I: Iterator<Item = u32>,
{
    if (address.0 & 3) != 0 {
        return Err(Error::BadAlignment(4));
    }

    unsafe {
        let reg = get_registers();
        // Write the target address to the FMA register
        reg.fma.write(|w| w.offset().bits(address.0));
        // Write the data to the FMBx registers
        let mut fmbx = 0x400FD100 as *mut u32;
        let mut count = 0;
        for word in data {
            if count >= PAGE_LENGTH_WORDS {
                return Err(Error::BufferTooLong(PAGE_LENGTH_WORDS));
            }
            volatile_store(fmbx, word);
            fmbx = ((fmbx as usize) + 4) as *mut u32;
            count += 1;
        }
        // Write the flash memory key and the write bit
        volatile_store(0x400FD020 as *mut u32, 0xA4420001);
        // Poll the FMC register until the WRITE bit is cleared
        while reg.fmc2.read().wrbuf().bit() {
            nop();
        }
    }

    Ok(())
}

/// Get the size of flash in bytes.
pub fn get_flash_size() -> usize {
    let reg = unsafe { get_registers() };
    1024 * match reg.fsize.read().bits() {
        0x0003 => 8,
        0x0007 => 16,
        0x000F => 32,
        0x001F => 64,
        0x002F => 96,
        0x003F => 128,
        0x005F => 192,
        0x007F => 256,
        _ => 0,
    }
}

// Get the size of SRAM in bytes.
pub fn get_sram_size() -> usize {
    let reg = unsafe { get_registers() };
    1024 * match reg.ssize.read().bits() {
        0x0007 => 2,
        0x000F => 4,
        0x0017 => 6,
        0x001F => 8,
        0x002F => 12,
        0x003F => 16,
        0x004F => 20,
        0x005F => 24,
        0x007F => 32,
        _ => 0,
    }
}

// ****************************************************************************
//
// Private Functions
//
// ****************************************************************************
/// Get a reference to the UART control register struct in the chip.
unsafe fn get_registers() -> &'static tm4c123x::flash_ctrl::RegisterBlock {
    &*tm4c123x::FLASH_CTRL::ptr()
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
    if fcris.aris().bit() || fcris.voltris().bit() || fcris.invdris().bit() || fcris.progris().bit()
        || fcris.erris().bit()
    {
        Err(Error::HardwareError(fcris.bits()))
    } else {
        Ok(())
    }
}

impl fmt::Display for FlashAddress {
    fn fmt(&self, fmt: &mut fmt::Formatter) -> fmt::Result {
        write!(fmt, "0x{:08x}", self.0)
    }
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
