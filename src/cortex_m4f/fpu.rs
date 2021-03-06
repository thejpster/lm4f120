//! # FPU support for the Cortex-M4F

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

use cortex_m;

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

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPAC register.
//
// *****************************************************************************
const NVIC_CPAC_CP11_M: usize = 0x00C00000; // CP11 Coprocessor Access Privilege
const NVIC_CPAC_CP11_FULL: usize = 0x00C00000; // Full Access
const NVIC_CPAC_CP10_M: usize = 0x00300000; // CP10 Coprocessor Access Privilege
const NVIC_CPAC_CP10_FULL: usize = 0x00300000; // Full Access

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// Enable full access to the FPU
pub fn init() {
    unsafe {
		let p = &*cortex_m::peripheral::SCB::ptr();
        p.cpacr.modify(|r| {
            (r & !(NVIC_CPAC_CP11_M as u32 | NVIC_CPAC_CP10_M as u32))
                | (NVIC_CPAC_CP11_FULL as u32 | NVIC_CPAC_CP10_FULL as u32)
        });
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
