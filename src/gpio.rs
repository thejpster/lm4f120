//! # GPIO for the TI LM4F120H5QR
//!
//! Supports GPIO, mapping in a UART and putting pins in Timer mode.

// ****************************************************************************
//
// Imports
//
// ****************************************************************************

use core::intrinsics::{volatile_load, volatile_store};

use cortex_m::asm::nop;

use super::registers as reg;
use super::uart::UartId;

use embedded_hal::digital::OutputPin;

// ****************************************************************************
//
// Public Types
//
// ****************************************************************************

/// Describes a pin within a port
/// This chip has 8 pins per port.
#[derive(PartialEq, Copy, Clone)]
pub enum PinId {
    Pin0,
    Pin1,
    Pin2,
    Pin3,
    Pin4,
    Pin5,
    Pin6,
    Pin7,
}

/// Describes a Port and a single pin within it
#[derive(PartialEq, Copy, Clone)]
pub enum PinPortId {
    PortA(PinId),
    PortB(PinId),
    PortC(PinId),
    PortD(PinId),
    PortE(PinId),
    PortF(PinId),
}

pub struct PinPort {
    id: PinPortId,
}

/// Describes a pin's direction
#[derive(PartialEq, Clone, Copy)]
pub enum PinMode {
    /// An input with a pull-up or pull-down
    InputPull(Level),
    /// An input with no pull
    Input,
    /// A totem-pole output
    Output,
    /// Pin is driven by a peripheral (i.e. is no longer a GPIO)
    Peripheral,
}

/// Describes what a pin can be set to
#[derive(PartialEq, Clone, Copy)]
pub enum Level {
    /// A logic high (i.e. 3.3v)
    High,
    /// A logic low (i.e. 0v)
    Low,
}

pub struct Pins {
    pub pa0: Option<PinPort>,
    pub pa1: Option<PinPort>,
    pub pf0: Option<PinPort>,
    pub pf1: Option<PinPort>,
    pub pf2: Option<PinPort>,
    pub pf3: Option<PinPort>,
    pub pf4: Option<PinPort>,
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

use core::sync::atomic::{AtomicBool, Ordering, ATOMIC_BOOL_INIT};
static PERIPHS_TAKEN: AtomicBool = ATOMIC_BOOL_INIT;

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// Grab all the GPIO pins.
/// You can only call this once; the second time it returns None.
pub fn take() -> Option<Pins> {
    if PERIPHS_TAKEN.compare_and_swap(false, true, Ordering::Relaxed) == false {
        Some(Pins {
            pa0: Some(PinPort {
                id: PinPortId::PortA(PinId::Pin0),
            }),
            pa1: Some(PinPort {
                id: PinPortId::PortA(PinId::Pin1),
            }),
            pf0: Some(PinPort {
                id: PinPortId::PortF(PinId::Pin0),
            }),
            pf1: Some(PinPort {
                id: PinPortId::PortF(PinId::Pin1),
            }),
            pf2: Some(PinPort {
                id: PinPortId::PortF(PinId::Pin2),
            }),
            pf3: Some(PinPort {
                id: PinPortId::PortF(PinId::Pin3),
            }),
            pf4: Some(PinPort {
                id: PinPortId::PortF(PinId::Pin4),
            }),
        })
    } else {
        None
    }
}

impl PinPort {
    /// Set the direction (input or output) on a given pin in a port
    pub fn set_direction(&mut self, mode: PinMode) {
        match mode {
            PinMode::InputPull(Level::High) => make_input_pullup(self.id),
            PinMode::InputPull(Level::Low) => make_input_pulldown(self.id),
            PinMode::Input => make_input(self.id),
            PinMode::Output => make_output(self.id, Level::Low),
            PinMode::Peripheral => make_peripheral(self.id),
        }
    }

    /// Set the output value for an output pin
    pub fn set(&mut self, level: Level) {
        let mask = get_pin_mask(self.id);
        let gpio_reg = get_port_registers(self.id);
        match level {
            Level::Low => unsafe { gpio_reg.data_mask[mask].write(0) },
            Level::High => unsafe { gpio_reg.data_mask[mask].write(0xFF) },
        }
    }

    /// Read the level of an input pin
    pub fn read(&self) -> Level {
        let mask = get_pin_mask(self.id);
        let gpio_reg = get_port_registers(self.id);
        if unsafe { gpio_reg.data_mask[mask].read() } == 0 {
            Level::Low
        } else {
            Level::High
        }
    }

    /// Enable a pin as a Timer Compare pin (e.g. if you want to use it as
    /// a PWM output). We assume you've already set it as `PinMode::Peripheral`
    /// using `set_direction()`.
    pub fn enable_ccp(&mut self) {
        let gpio_reg = get_port_registers(self.id);
        enable_port(self.id);
        match self.id {
            PinPortId::PortB(PinId::Pin0) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB0_M) | reg::GPIO_PCTL_PB0_T2CCP0);
            },
            PinPortId::PortB(PinId::Pin1) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB1_M) | reg::GPIO_PCTL_PB1_T2CCP1);
            },
            PinPortId::PortB(PinId::Pin2) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB2_M) | reg::GPIO_PCTL_PB2_T3CCP0);
            },
            PinPortId::PortB(PinId::Pin3) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB3_M) | reg::GPIO_PCTL_PB3_T3CCP1);
            },
            PinPortId::PortB(PinId::Pin4) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB4_M) | reg::GPIO_PCTL_PB4_T1CCP0);
            },
            PinPortId::PortB(PinId::Pin5) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB5_M) | reg::GPIO_PCTL_PB5_T1CCP1);
            },
            PinPortId::PortB(PinId::Pin6) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB6_M) | reg::GPIO_PCTL_PB6_T0CCP0);
            },
            PinPortId::PortB(PinId::Pin7) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB7_M) | reg::GPIO_PCTL_PB7_T0CCP1);
            },
            PinPortId::PortC(PinId::Pin0) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC0_M) | reg::GPIO_PCTL_PC0_T4CCP0);
            },
            PinPortId::PortC(PinId::Pin1) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC1_M) | reg::GPIO_PCTL_PC1_T4CCP1);
            },
            PinPortId::PortC(PinId::Pin2) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC2_M) | reg::GPIO_PCTL_PC2_T5CCP0);
            },
            PinPortId::PortC(PinId::Pin3) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC3_M) | reg::GPIO_PCTL_PC3_T5CCP1);
            },
            PinPortId::PortF(PinId::Pin1) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PF1_M) | reg::GPIO_PCTL_PF1_T0CCP1);
            },
            PinPortId::PortF(PinId::Pin2) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PF2_M) | reg::GPIO_PCTL_PF2_T1CCP0);
            },
            PinPortId::PortF(PinId::Pin3) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PF3_M) | reg::GPIO_PCTL_PF3_T1CCP1);
            },
            PinPortId::PortF(PinId::Pin4) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PF4_M) | reg::GPIO_PCTL_PF4_T2CCP0);
            },
            _ => {
                unimplemented!();
            }
        }
    }
}

impl OutputPin for PinPort {
    /// Check if pin is currently set high.
    fn is_high(&self) -> bool {
        self.read() == Level::High
    }

    /// Check if pin is currently set low.
    fn is_low(&self) -> bool {
        self.read() == Level::Low
    }

    /// Set pin logic low.
    fn set_low(&mut self) {
        self.set(Level::Low);
    }

    /// Set pin logic high.
    fn set_high(&mut self) {
        self.set(Level::High);
    }
}

/// Re-configure the pinmuxing so that the given Uart appears
/// on its normal set of pins.
///
/// Only Uart0 is supported at the moment, and it appears on
/// A0 and A1.
pub fn enable_uart(id: UartId, pin_a: &mut PinPort, pin_b: &mut PinPort) {
    match (id, pin_a.id, pin_b.id) {
        (UartId::Uart0, PinPortId::PortA(PinId::Pin0), PinPortId::PortA(PinId::Pin1)) => {
            enable_port(pin_a.id);
            let gpio_reg = get_port_registers(pin_a.id);
            unsafe {
                gpio_reg.afsel.modify(|r| r | (1 << 1) | (1 << 0));
                gpio_reg.den.modify(|r| r | (1 << 1) | (1 << 0));
                gpio_reg.pctl.modify(|r| {
                    (r & !(reg::GPIO_PCTL_PA0_M | reg::GPIO_PCTL_PA1_M))
                        | (reg::GPIO_PCTL_PA0_U0RX | reg::GPIO_PCTL_PA1_U0TX)
                });
            }
        }
        _ => {
            unimplemented!();
        }
    }
}

// ****************************************************************************
//
// Private Functions
//
// ****************************************************************************

/// Convert a port to a bit mask
/// Port A is 1, PortF is 32
fn get_port_mask(port: PinPortId) -> usize {
    match port {
        PinPortId::PortA(_) => 1 << 0,
        PinPortId::PortB(_) => 1 << 1,
        PinPortId::PortC(_) => 1 << 2,
        PinPortId::PortD(_) => 1 << 3,
        PinPortId::PortE(_) => 1 << 4,
        PinPortId::PortF(_) => 1 << 5,
    }
}

/// Convert a pin to a bit mask
/// Pin0 is 0, Pin7 is 128
fn get_pin_mask(pinport: PinPortId) -> usize {
    let pin = match pinport {
        PinPortId::PortA(ref x) => x,
        PinPortId::PortB(ref x) => x,
        PinPortId::PortC(ref x) => x,
        PinPortId::PortD(ref x) => x,
        PinPortId::PortE(ref x) => x,
        PinPortId::PortF(ref x) => x,
    };
    match *pin {
        PinId::Pin0 => 1 << 0 as usize,
        PinId::Pin1 => 1 << 1 as usize,
        PinId::Pin2 => 1 << 2 as usize,
        PinId::Pin3 => 1 << 3 as usize,
        PinId::Pin4 => 1 << 4 as usize,
        PinId::Pin5 => 1 << 5 as usize,
        PinId::Pin6 => 1 << 6 as usize,
        PinId::Pin7 => 1 << 7 as usize,
    }
}

fn get_pctl_mask(pinport: PinPortId) -> usize {
    let pin = match pinport {
        PinPortId::PortA(x) => x,
        PinPortId::PortB(x) => x,
        PinPortId::PortC(x) => x,
        PinPortId::PortD(x) => x,
        PinPortId::PortE(x) => x,
        PinPortId::PortF(x) => x,
    };
    match pin {
        PinId::Pin0 => 7 << 0 as usize,
        PinId::Pin1 => 7 << 4 as usize,
        PinId::Pin2 => 7 << 8 as usize,
        PinId::Pin3 => 7 << 12 as usize,
        PinId::Pin4 => 7 << 16 as usize,
        PinId::Pin5 => 7 << 20 as usize,
        PinId::Pin6 => 7 << 24 as usize,
        PinId::Pin7 => 7 << 28 as usize,
    }
}

fn enable_port(port: PinPortId) {
    let mask = get_port_mask(port);
    unsafe {
        volatile_store(
            reg::SYSCTL_RCGCGPIO_R,
            mask | volatile_load(reg::SYSCTL_RCGCGPIO_R),
        );
        while (volatile_load(reg::SYSCTL_RCGCGPIO_R) & mask) == 0 {
            nop();
        }
    }
}

fn force_gpio_periph(pinport: PinPortId, gpio_reg: &mut reg::GpioRegisters) {
    let mask = get_pin_mask(pinport);
    let pctl_mask = get_pctl_mask(pinport);
    unsafe {
        gpio_reg.afsel.modify(|r| r & !mask);
        gpio_reg.pctl.modify(|r| r & !pctl_mask);
    }
}

fn make_input(pinport: PinPortId) {
    enable_port(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    force_gpio_periph(pinport, gpio_reg);
    unsafe {
        if pinport == PinPortId::PortF(PinId::Pin0) {
            // The GPIO for button one is multiplexed with NMI so we
            // have to 'unlock' it before we can use it
            gpio_reg.lock.write(reg::GPIO_LOCK_KEY);
            gpio_reg.cr.modify(|r| r | mask);
            gpio_reg.lock.write(0);
        }
        gpio_reg.den.modify(|r| r | mask);
        gpio_reg.dir.modify(|r| r & !mask);
    }
}

fn make_peripheral(pinport: PinPortId) {
    enable_port(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    unsafe {
        gpio_reg.afsel.modify(|r| r | mask);
        gpio_reg.den.modify(|r| r | mask);
        gpio_reg.dir.modify(|r| r & !mask);
    }
}

fn make_input_pullup(pinport: PinPortId) {
    make_input(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    unsafe {
        gpio_reg.dr2r.modify(|r| r | mask);
        gpio_reg.pur.modify(|r| r | mask);
    }
}

fn make_input_pulldown(pinport: PinPortId) {
    make_input(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    unsafe {
        gpio_reg.dr2r.modify(|r| r | mask);
        gpio_reg.pur.modify(|r| r & !mask);
    }
}

fn make_output(pinport: PinPortId, level: Level) {
    enable_port(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    force_gpio_periph(pinport, gpio_reg);
    unsafe {
        match level {
            Level::Low => gpio_reg.data_mask[mask].write(0),
            Level::High => gpio_reg.data_mask[mask].write(0xFF),
        }
        gpio_reg.dir.modify(|r| r | mask);
        gpio_reg.den.modify(|r| r | mask);
    }
}

/// Convert a GPIO port into a reference to the registers which control that port
fn get_port_registers(port: PinPortId) -> &'static mut reg::GpioRegisters {
    unsafe {
        match port {
            PinPortId::PortA(_) => &mut *(reg::GPIO_PORTA_DATA_BITS_R as *mut reg::GpioRegisters),
            PinPortId::PortB(_) => &mut *(reg::GPIO_PORTB_DATA_BITS_R as *mut reg::GpioRegisters),
            PinPortId::PortC(_) => &mut *(reg::GPIO_PORTC_DATA_BITS_R as *mut reg::GpioRegisters),
            PinPortId::PortD(_) => &mut *(reg::GPIO_PORTD_DATA_BITS_R as *mut reg::GpioRegisters),
            PinPortId::PortE(_) => &mut *(reg::GPIO_PORTE_DATA_BITS_R as *mut reg::GpioRegisters),
            PinPortId::PortF(_) => &mut *(reg::GPIO_PORTF_DATA_BITS_R as *mut reg::GpioRegisters),
        }
    }
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
