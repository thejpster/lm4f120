//! # Timers for the LM4F120
//!

//! The Stellaris core has six 16/32-bit timers and six 32/64-bit wide timers.
//! Each timer provides two timers that can operate independently, or be
//! chained together to form a single double-width timer. The Cortex-M4 core
//! also its own separate SysTick timer. This is a 24-bit timer with its own
//! ISR.

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

use core::intrinsics::{volatile_store, volatile_load};
use core::sync::atomic::{AtomicUsize, Ordering, ATOMIC_USIZE_INIT};
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

/// SysTick is a 24-bit timer
pub const SYSTICK_MAX:usize = (1 << 24) - 1;

/// SysTick runs at 16MHz / 4 = 4MHz
pub const SYSTICK_CLOCK:usize = 4_000_000;

/// At 4MHz, four ticks per microseconds.
pub const SYSTICK_CLOCK_PER_US:usize = 4;

lazy_static! {
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

// None

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// Initialises the SysTick system.
///
/// We configure SysTick to run at 4MHz, with the full 24 bit range.
pub fn init() {
    unsafe {
        // SysTick counts down from max to zero
        volatile_store(registers::NVIC_ST_RELOAD_R, SYSTICK_MAX as usize);
        // A write to current resets the timer
        volatile_store(registers::NVIC_ST_CURRENT_R, 0);
        // Set to multi-shot mode, with interrupts on and on the PIOSC / 4
        volatile_store(registers::NVIC_ST_CTRL_R, registers::NVIC_ST_CTRL_ENABLE | registers::NVIC_ST_CTRL_INTEN);
    }
}

/// Converts from SysTicks to microseconds
pub fn ticks_to_usecs(ticks: usize) -> usize {
    ticks / SYSTICK_CLOCK_PER_US
}

/// Converts from microseconds to SysTicks
pub fn usecs_to_ticks(usecs: usize) -> usize {
    usecs * SYSTICK_CLOCK_PER_US
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
    let result = unsafe {
        volatile_load(registers::NVIC_ST_CURRENT_R)
    };
    result
}

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
pub fn delay_usec(usec: u32)
{
    let start = get_ticks();
    let ticks = usecs_to_ticks(usec as usize);
    loop {
        if get_since(start) >= ticks {
            break
        }
    }
}

/// How long since the system booted in microseconds.
/// The u64 is good for 584,000 years.
pub fn run_time_us() -> u64 {
    let (overflows, ticks) = get_overflows_ticks();
    let mut result:u64;
    result = overflows as u64;
    result *= (SYSTICK_MAX + 1) as u64;
    result += (SYSTICK_MAX - ticks) as u64;
    result >>= 2;
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
