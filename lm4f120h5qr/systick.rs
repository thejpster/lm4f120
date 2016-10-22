//! # Systick for the LM4F120
//!
//! This just wraps the generic Cortex-M4 systick, but it
//! understands the PLL clock rate (which the generic code cannot)

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

pub use super::super::cortex_m4f::systick::*;
use super::pll;

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

/// SysTick runs at / 4, so at 16MHz that's 4MHz
pub const SYSTICK_CLOCK: usize = pll::CRYSTAL_CLOCK_HZ / 4;

/// At 4MHz, four ticks per microseconds.
pub const SYSTICK_CLOCK_PER_US: usize = SYSTICK_CLOCK / 1_000_000;

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

/// Converts from SysTicks to microseconds
pub fn ticks_to_usecs(ticks: usize) -> usize {
    ticks / SYSTICK_CLOCK_PER_US
}

/// Converts from microseconds to SysTicks
pub fn usecs_to_ticks(usecs: usize) -> usize {
    usecs * SYSTICK_CLOCK_PER_US
}

/// Busy-waits for the given period.
///
/// Uses SysTick to wait the correct amount of time.
///
/// * `ms` - The period to wait, in milliseconds
pub fn delay(ms: u32) {
    // We can manage 4 seconds before SysTick wraps
    // We divide it up into seconds.
    let seconds = ms / 1000;
    let usec = (ms % 1000) * 1000;
    for _ in 0..seconds {
        delay_usec(1_000_000);
    }
    delay_usec(usec);
}

/// Busy-waits a specified number of microseconds.
/// `usec` must be less than 2**22 otherwise SysTick
/// will overflow.
pub fn delay_usec(usec: u32) {
    let start = get_ticks();
    let ticks = usecs_to_ticks(usec as usize);
    loop {
        if get_since(start) >= ticks {
            break;
        }
    }
}

/// How long since the system booted in microseconds.
/// The u64 is good for 584,000 years.
pub fn run_time_us() -> u64 {
    let mut result = run_time_ticks();
    result /= SYSTICK_CLOCK_PER_US as u64;
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
