//! # UART for the LM4F120H5QR

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

use core::fmt;
use core::intrinsics::{volatile_load, volatile_store};

use embedded_serial::{self, BlockingTx, NonBlockingRx};
use cortex_m::asm::nop;

use super::gpio;
use super::pll;
use super::registers as reg;

use tm4c123x::uart0;

// ****************************************************************************
//
// Public Types
//
// ****************************************************************************

/// This chip has 8 UARTs
#[derive(PartialEq, Clone, Copy)]
#[allow(missing_docs)]
pub enum UartId {
    Uart0,
    Uart1,
    Uart2,
    Uart3,
    Uart4,
    Uart5,
    Uart6,
    Uart7,
}

/// Controls a single UART
/// Only supports 8/N/1 - who needs anything else?
pub struct Uart {
    id: UartId,
    baud: u32,
    nl_mode: NewlineMode,
    reg: &'static uart0::RegisterBlock,
}

/// writeln!() emits LF chars, so this is useful
/// if you're writing text with your UART
#[derive(PartialEq, Clone, Copy)]
pub enum NewlineMode {
    /// Emit octets as received
    Binary,
    /// Emit an extra CR before every LF
    SwapLFtoCRLF,
}

// ****************************************************************************
//
// Private Types
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
// Public Functions
//
// ****************************************************************************

/// Represents a single UART on the LM4F120
impl Uart {
    /// Create a new Uart object. The caller is responsible for ensuring
    /// that only one object exists per UartId. The UART is set to
    /// 8 data bits, 1 stop bit, no parity and is not configurable.
    /// Optionally, newline translation can be performed on outbound data
    /// - this will cause writeln!() to emit a CRLF.
    pub fn new(id: UartId, baud: u32, nl_mode: NewlineMode) -> Uart {
        let mut uart = Uart {
            id: id,
            baud: baud,
            nl_mode: nl_mode,
            reg: get_uart_registers(id),
        };
        uart.init();
        uart
    }

    /// Configure the hardware
    fn init(&mut self) -> () {
        // Do GPIO pin muxing
        gpio::enable_uart(self.id);
        // Enable UART module in RCGUART register p306
        unsafe {
            self.enable_clock();

            // Disable UART and all features
            self.reg.ctl.reset();
            // Calculate the baud rate values
            // baud_div = CLOCK_RATE / (16 * baud_rate);
            // baud_int = round(baud_div * 64)
            let baud_int: u32 = (((pll::get_clock_hz() * 8) / self.baud) + 1) / 2;
            // Store the upper and lower parts of the divider
            self.reg
                .ibrd
                .write(|w| w.divint().bits((baud_int / 64) as u16));
            self.reg
                .fbrd
                .write(|w| w.divfrac().bits((baud_int % 64) as u8));
            // Set the UART Line Control register value
            // 8N1 + FIFO enabled
            self.reg.lcrh.write(|w| w.wlen()._8().fen().bit(true));
            // Enable
            self.reg
                .ctl
                .write(|w| w.rxe().bit(true).txe().bit(true).uarten().bit(true));
        }
    }

    /// Select which bits to enable in the clock gating register
    fn get_clock_gating_mask(&self) -> usize {
        match self.id {
            UartId::Uart0 => reg::SYSCTL_RCGCUART_R0,
            UartId::Uart1 => reg::SYSCTL_RCGCUART_R1,
            UartId::Uart2 => reg::SYSCTL_RCGCUART_R2,
            UartId::Uart3 => reg::SYSCTL_RCGCUART_R3,
            UartId::Uart4 => reg::SYSCTL_RCGCUART_R4,
            UartId::Uart5 => reg::SYSCTL_RCGCUART_R5,
            UartId::Uart6 => reg::SYSCTL_RCGCUART_R6,
            UartId::Uart7 => reg::SYSCTL_RCGCUART_R7,
        }
    }

    /// Enable the module in the real-time clock gating registers.
    unsafe fn enable_clock(&mut self) {
        volatile_store(reg::SYSCTL_RCGCUART_R, self.get_clock_gating_mask());
        while volatile_load(reg::SYSCTL_RCGCUART_R) != self.get_clock_gating_mask() {
            nop();
        }
    }

    #[deprecated]
    pub fn read_single(&mut self) -> Option<u8> {
        self.getc_try().unwrap()
    }
}

impl embedded_serial::BlockingTx for Uart {
    type Error = !;

    /// Emit a single octet, busy-waiting if the FIFO is full.
    /// Never returns `Err`.
    fn putc(&mut self, value: u8) -> Result<(), Self::Error> {
        while self.reg.fr.read().txff().bit() {
            nop();
        }
        self.reg.dr.write(|w| unsafe { w.data().bits(value) });
        Ok(())
    }
}

impl embedded_serial::NonBlockingTx for Uart {
    type Error = !;

    /// Attempts to write to the UART. Returns `Ok(None)` if the transmiter
    /// not ready, or `Ok(Some(value))`. Never returns `Err`.
    fn putc_try(&mut self, value: u8) -> Result<Option<u8>, Self::Error> {
        if self.reg.fr.read().txff().bit() {
            Ok(None)
        } else {
            self.reg.dr.write(|w| unsafe { w.data().bits(value) });
            Ok(Some(value))
        }
    }
}

impl embedded_serial::BlockingRx for Uart {
    type Error = !;

    /// Read a single octet, busy-waiting if the FIFO is empty.
    /// Never returns `Err`.
    fn getc(&mut self) -> Result<u8, Self::Error> {
        while self.reg.fr.read().rxfe().bit() {
            nop();
        }
        Ok(self.reg.dr.read().data().bits())
    }
}

impl embedded_serial::NonBlockingRx for Uart {
    type Error = !;

    /// Attempts to read from the UART. Returns `Ok(None)` if the FIFO is
    /// empty, or `Ok(octet)`. Never returns `Err`.
    fn getc_try(&mut self) -> Result<Option<u8>, Self::Error> {
        if self.reg.fr.read().rxfe().bit() {
            Ok(None)
        } else {
            Ok(Some(self.reg.dr.read().data().bits()))
        }
    }
}

/// Allows the Uart to be passed to 'write!()' and friends.
impl fmt::Write for Uart {
    fn write_str(&mut self, s: &str) -> fmt::Result {
        match self.nl_mode {
            NewlineMode::Binary => self.puts(s).unwrap(),
            NewlineMode::SwapLFtoCRLF => {
                for byte in s.bytes() {
                    if byte == 0x0A {
                        // Prefix every \n with a \r
                        self.putc(0x0D).unwrap()
                    }
                    self.putc(byte).unwrap()
                }
            }
        }
        Ok(())
    }
}

/// Called when UART 0 interrupt fires
pub unsafe extern "C" fn uart0_isr() {}

// ****************************************************************************
//
// Private Functions
//
// ****************************************************************************

/// Get a reference to the UART control register struct in the chip.
fn get_uart_registers(uart_id: UartId) -> &'static uart0::RegisterBlock {
    use tm4c123x::{UART0, UART1, UART2, UART3, UART4, UART5, UART6, UART7};
    unsafe {
        match uart_id {
            UartId::Uart0 => &*UART0.get(),
            UartId::Uart1 => &*UART1.get(),
            UartId::Uart2 => &*UART2.get(),
            UartId::Uart3 => &*UART3.get(),
            UartId::Uart4 => &*UART4.get(),
            UartId::Uart5 => &*UART5.get(),
            UartId::Uart6 => &*UART6.get(),
            UartId::Uart7 => &*UART7.get(),
        }
    }
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
