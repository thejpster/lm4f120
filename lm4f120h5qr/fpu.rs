//! This is a template file.

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

use core::intrinsics::{volatile_store, volatile_load};
use super::registers;

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

pub fn init() {
    // Enable full access to the FPU
    unsafe {
        let mut reg = volatile_load(registers::NVIC_CPAC_R);
        reg &= !(registers::NVIC_CPAC_CP11_M | registers::NVIC_CPAC_CP10_M);
        reg |= registers::NVIC_CPAC_CP11_FULL | registers::NVIC_CPAC_CP10_FULL;
        volatile_store(registers::NVIC_CPAC_R, reg);
    }

    // Enable lazy-stacking of FPU registers
    // Stack space is allocated, but they aren't pushed until the first FPU operation
    unsafe {
        let mut reg = volatile_load(registers::NVIC_FPCC_R);
        reg |= registers::NVIC_FPCC_ASPEN | registers::NVIC_FPCC_LSPEN;
        volatile_store(registers::NVIC_FPCC_R, reg);
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
