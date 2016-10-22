//! # Timers for the LM4F120H5QR
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

use core::intrinsics::volatile_store;

use cortex_m::asm::nop;

use super::registers as reg;

// ****************************************************************************
//
// Public Types
//
// ****************************************************************************

/// This chip has 6 16/32bit Timers
#[derive(PartialEq, Clone, Copy)]
#[allow(missing_docs)]
pub enum TimerId {
    Timer0A,
    Timer1A,
    Timer2A,
    Timer3A,
    Timer4A,
    Timer5A,
    Timer0B,
    Timer1B,
    Timer2B,
    Timer3B,
    Timer4B,
    Timer5B,
    Timer0,
    Timer1,
    Timer2,
    Timer3,
    Timer4,
    Timer5,
}

/// This chip has 6 32/64bit Wide Timers, but they are currently not
/// supported.
#[allow(missing_docs)]
pub enum WideTimerId {
    Timer0A,
    Timer1A,
    Timer2A,
    Timer3A,
    Timer4A,
    Timer5A,
    Timer0B,
    Timer1B,
    Timer2B,
    Timer3B,
    Timer4B,
    Timer5B,
    Timer0,
    Timer1,
    Timer2,
    Timer3,
    Timer4,
    Timer5,
}

/// Represents a 16/32-bit Timer
pub struct Timer {
    id: TimerId,
    reg: &'static mut reg::TimerRegisters,
    period: u32,
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

// None

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

impl Timer {
    /// Create a new Timer object. The caller is responsible for ensuring
    /// that only one object exists per TimerId.
    pub fn new(id: TimerId) -> Timer {
        let mut t = Timer {
            id: id,
            reg: get_timer_registers(id),
            period: 0,
        };
        t.init();
        t
    }

    fn init(&mut self) {
        unsafe { self.enable_clock() }
    }

    fn get_clock_gating_mask(&self) -> usize {
        match self.id {
            TimerId::Timer0A | TimerId::Timer0B | TimerId::Timer0 => reg::SYSCTL_RCGCTIMER_R0,
            TimerId::Timer1A | TimerId::Timer1B | TimerId::Timer1 => reg::SYSCTL_RCGCTIMER_R1,
            TimerId::Timer2A | TimerId::Timer2B | TimerId::Timer2 => reg::SYSCTL_RCGCTIMER_R2,
            TimerId::Timer3A | TimerId::Timer3B | TimerId::Timer3 => reg::SYSCTL_RCGCTIMER_R3,
            TimerId::Timer4A | TimerId::Timer4B | TimerId::Timer4 => reg::SYSCTL_RCGCTIMER_R4,
            TimerId::Timer5A | TimerId::Timer5B | TimerId::Timer5 => reg::SYSCTL_RCGCTIMER_R5,
        }
    }

    unsafe fn enable_clock(&mut self) {
        volatile_store(reg::SYSCTL_RCGCTIMER_R, self.get_clock_gating_mask());
        nop();
        nop();
        nop();
    }

    /// Activate the PWM output, with the specified period (in clock ticks)
    pub fn enable_pwm(&mut self, period: u32) {
        self.period = period;

        if self.use_timer_a() {
            self.reg.ctl.modify(|r| r & !reg::TIMER_CTL_TAEN);
            if self.is_double_width() {
                self.reg.cfg.write(reg::TIMER_CFG_32_BIT_TIMER);
            } else {
                self.reg.cfg.write(reg::TIMER_CFG_16_BIT);
            }
            self.reg.tamr.modify(|mut r| {
                r |= reg::TIMER_TAMR_TAAMS;
                r &= !reg::TIMER_TAMR_TACMR;
                r &= !reg::TIMER_TAMR_TAMR_M;
                r |= reg::TIMER_TAMR_TAMR_PERIOD;
                r |= reg::TIMER_TAMR_TAPWMIE;
                r |= reg::TIMER_TAMR_TAMRSU;
                r
            });
            self.set_pwm(period);
            self.reg.ctl.modify(|r| r | reg::TIMER_CTL_TAEN);
        } else {
            self.reg.ctl.modify(|r| r & !reg::TIMER_CTL_TBEN);
            if self.is_double_width() {
                self.reg.cfg.write(reg::TIMER_CFG_32_BIT_TIMER);
            } else {
                self.reg.cfg.write(reg::TIMER_CFG_16_BIT);
            }
            self.reg.tbmr.modify(|mut r| {
                r |= reg::TIMER_TBMR_TBAMS;
                r &= !reg::TIMER_TBMR_TBCMR;
                r &= !reg::TIMER_TBMR_TBMR_M;
                r |= reg::TIMER_TBMR_TBMR_PERIOD;
                r |= reg::TIMER_TBMR_TBPWMIE;
                r |= reg::TIMER_TBMR_TBMRSU;
                r
            });
            self.set_pwm(period);
            self.reg.ctl.modify(|r| r | reg::TIMER_CTL_TBEN);
        }
    }

    /// Set the PWM period for the timer (in clock ticks)
    pub fn set_pwm(&mut self, on_time: u32) {
        if self.use_timer_a() {
            self.reg.tamatchr.write((self.period - on_time) as usize);
            self.reg.tailr.write(self.period as usize);
        } else {
            self.reg.tbmatchr.write((self.period - on_time) as usize);
            self.reg.tbilr.write(self.period as usize);
        }
    }

    /// Read the current timer value
    pub fn get_timer(&self) -> u32 {
        if !self.use_timer_a() {
            self.reg.tar.read() as u32
        } else {
            self.reg.tbr.read() as u32
        }
    }

    /// Check if we are using a timer in double-width mode (as they need
    /// handling differently to the two single-width timers).
    fn is_double_width(&self) -> bool {
        match self.id {
            TimerId::Timer0 => true,
            TimerId::Timer1 => true,
            TimerId::Timer2 => true,
            TimerId::Timer3 => true,
            TimerId::Timer4 => true,
            TimerId::Timer5 => true,
            _ => false,
        }
    }

    /// Check if we should use Timer A registers, or the Timer B registers.
    fn use_timer_a(&self) -> bool {
        match self.id {
            TimerId::Timer0B => false,
            TimerId::Timer1B => false,
            TimerId::Timer2B => false,
            TimerId::Timer3B => false,
            TimerId::Timer4B => false,
            TimerId::Timer5B => false,
            _ => true,
        }
    }
}

// ****************************************************************************
//
// Private Functions
//
// ****************************************************************************

/// Get a reference to the UART control register struct in the chip.
fn get_timer_registers(timer_id: TimerId) -> &'static mut reg::TimerRegisters {
    unsafe {
        match timer_id {
            TimerId::Timer0A | TimerId::Timer0B | TimerId::Timer0 => {
                &mut *(reg::TIMER0_CFG_R as *mut reg::TimerRegisters)
            }
            TimerId::Timer1A | TimerId::Timer1B | TimerId::Timer1 => {
                &mut *(reg::TIMER1_CFG_R as *mut reg::TimerRegisters)
            }
            TimerId::Timer2A | TimerId::Timer2B | TimerId::Timer2 => {
                &mut *(reg::TIMER2_CFG_R as *mut reg::TimerRegisters)
            }
            TimerId::Timer3A | TimerId::Timer3B | TimerId::Timer3 => {
                &mut *(reg::TIMER3_CFG_R as *mut reg::TimerRegisters)
            }
            TimerId::Timer4A | TimerId::Timer4B | TimerId::Timer4 => {
                &mut *(reg::TIMER4_CFG_R as *mut reg::TimerRegisters)
            }
            TimerId::Timer5A | TimerId::Timer5B | TimerId::Timer5 => {
                &mut *(reg::TIMER5_CFG_R as *mut reg::TimerRegisters)
            }
        }
    }
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
