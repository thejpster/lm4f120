//! # SysTick for the Cortex-M4F
//!
//! Each Cortex-M4 has a timer peripheral typically used for OS scheduling tick.
//! Here we configure it as a countdown timer that overflows every 2**24 ticks
//! (so about once a second at 16MHz), and maintain a separate atomic overflow
//! count to accurately track time since power-up.

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

use core::sync::atomic::{AtomicUsize, Ordering, ATOMIC_USIZE_INIT};
use cortex_m::peripheral as cm_periph;

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

/// SysTick is a 24-bit timer
pub const SYSTICK_MAX: usize = (1 << 24) - 1;

lazy_static! {
    /// total number of times SysTick has wrapped
    pub static ref SYSTICK_WRAP_COUNT:AtomicUsize = ATOMIC_USIZE_INIT;
}

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
// The following are defines for the bit fields in the NVIC_ST_CTRL register.
//
// *****************************************************************************
const NVIC_ST_CTRL_INTEN: usize = 0x00000002; // Interrupt Enable
const NVIC_ST_CTRL_ENABLE: usize = 0x00000001; // Enable

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// Initialises the SysTick system.
///
/// We configure SysTick to run at PIOSC / 4, with the full 24 bit range.
pub fn init() {
    unsafe {
        let syst = cm_periph::syst_mut();
        syst.rvr.write(SYSTICK_MAX as u32);
        // A write to current resets the timer
        syst.cvr.write(0);
        // Set to multi-shot mode, with interrupts on and on the PIOSC / 4
        syst.csr.write((NVIC_ST_CTRL_ENABLE | NVIC_ST_CTRL_INTEN) as u32);
    }
}

/// Should be attached to the SysTick vector in the interrupt vector table.
/// Called when SysTick hits zero. Increments an overflow counter atomically.
pub fn isr() {
    SYSTICK_WRAP_COUNT.fetch_add(1, Ordering::Relaxed);
}

/// Returns how many times SysTick has overflowed.
pub fn get_overflows() -> usize {
    SYSTICK_WRAP_COUNT.load(Ordering::Relaxed)
}

/// Gets the current SysTick value
pub fn get_ticks() -> usize {
    cm_periph::syst().cvr.read() as usize
}

/// Returns (overflows, ticks), correctly handling the case that it overflowed
/// between the two separate reads that are required.
pub fn get_overflows_ticks() -> (usize, usize) {
    let overflow1 = get_overflows();
    let ticks = get_ticks();
    let overflow2 = get_overflows();
    if overflow1 != overflow2 {
        // A overflow occurred while we were reading the tick register
        // Should be safe to try again
        (overflow2, get_ticks())
    } else {
        // No overflow, good to go
        (overflow1, ticks)
    }
}


/// Calculates the elapsed period in SysTicks between `start` and the current value.
pub fn get_since(start: usize) -> usize {
    let now = get_ticks();
    // SysTick counts down! This subtraction is opposite to what you expect.
    let delta = start.wrapping_sub(now) & SYSTICK_MAX;
    delta
}

/// How long since the system booted in ticks.
/// The u64 is good for 584,000 years.
pub fn run_time_ticks() -> u64 {
    let (overflows, ticks) = get_overflows_ticks();
    let mut result: u64;
    result = overflows as u64;
    result *= (SYSTICK_MAX + 1) as u64;
    result += (SYSTICK_MAX - ticks) as u64;
    result
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
