//! CPU definitions for the LM4F120

#![feature(asm)]
#![feature(core_intrinsics)]
#![feature(never_type)]
#![no_std]

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

extern crate cortex_m;
extern crate embedded_hal;
#[macro_use]
extern crate lazy_static;
#[macro_use(block)]
extern crate nb;
extern crate tm4c123x;
extern crate volatile_register;

pub mod gpio;
pub mod pll;
pub mod registers;
pub mod uart;
pub mod timer;
pub mod systick;
pub mod cortex_m4f;
pub mod flash;

pub use cortex_m4f::fpu;

use cortex_m::asm::nop;

// ****************************************************************************
//
// Public Types
//
// ****************************************************************************

// None

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

// None

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// Busy-waits for the given period.
///
/// Currently this function spins with a empirical number
/// of NOPS per millisecond. It should really use a timer.
///
/// * `ms` - The period to wait, in milliseconds
pub fn delay(ms: i32) {
    for _ in 0..ms * 250 {
        nop();
    }
}

// ****************************************************************************
//
// Private Functions
//
// ****************************************************************************

// None

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
