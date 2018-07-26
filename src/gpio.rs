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

use embedded_hal::digital::{StatefulOutputPin, OutputPin};

// ****************************************************************************
//
// Public Types
//
// ****************************************************************************

/// Describes a pin within a port
/// This chip has 8 pins per port.
#[derive(PartialEq, Clone, Copy)]
pub enum Pin {
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
#[derive(PartialEq, Clone, Copy)]
pub enum PinPort {
    PortA(Pin),
    PortB(Pin),
    PortC(Pin),
    PortD(Pin),
    PortE(Pin),
    PortF(Pin),
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

// ****************************************************************************
//
// Private Types
//
// ****************************************************************************

// None

// ****************************************************************************
//
// Public Functions
//
// ****************************************************************************

/// init() is empty for now, but it might be needed some day.
pub fn init() {}

impl PinPort {
    /// Set the direction (input or output) on a given pin in a port
    pub fn set_direction(&mut self, mode: PinMode) {
        match mode {
            PinMode::InputPull(Level::High) => make_input_pullup(self),
            PinMode::InputPull(Level::Low) => make_input_pulldown(self),
            PinMode::Input => make_input(self),
            PinMode::Output => make_output(self, Level::Low),
            PinMode::Peripheral => make_peripheral(self),
        }
    }

    /// Set the output value for an output pin
    pub fn set(&mut self, level: Level) {
        let mask = get_pin_mask(self);
        let gpio_reg = get_port_registers(self);
        match level {
            Level::Low => unsafe { gpio_reg.data_mask[mask].write(0) },
            Level::High => unsafe { gpio_reg.data_mask[mask].write(0xFF) },
        }
    }

    /// Read the level of an input pin
    pub fn read(&self) -> Level {
        let mask = get_pin_mask(self);
        let gpio_reg = get_port_registers(self);
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
        let gpio_reg = get_port_registers(self);
        enable_port(self);
        match self {
            &mut PinPort::PortB(Pin::Pin0) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB0_M) | reg::GPIO_PCTL_PB0_T2CCP0);
            },
            &mut PinPort::PortB(Pin::Pin1) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB1_M) | reg::GPIO_PCTL_PB1_T2CCP1);
            },
            &mut PinPort::PortB(Pin::Pin2) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB2_M) | reg::GPIO_PCTL_PB2_T3CCP0);
            },
            &mut PinPort::PortB(Pin::Pin3) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB3_M) | reg::GPIO_PCTL_PB3_T3CCP1);
            },
            &mut PinPort::PortB(Pin::Pin4) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB4_M) | reg::GPIO_PCTL_PB4_T1CCP0);
            },
            &mut PinPort::PortB(Pin::Pin5) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB5_M) | reg::GPIO_PCTL_PB5_T1CCP1);
            },
            &mut PinPort::PortB(Pin::Pin6) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB6_M) | reg::GPIO_PCTL_PB6_T0CCP0);
            },
            &mut PinPort::PortB(Pin::Pin7) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PB7_M) | reg::GPIO_PCTL_PB7_T0CCP1);
            },
            &mut PinPort::PortC(Pin::Pin0) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC0_M) | reg::GPIO_PCTL_PC0_T4CCP0);
            },
            &mut PinPort::PortC(Pin::Pin1) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC1_M) | reg::GPIO_PCTL_PC1_T4CCP1);
            },
            &mut PinPort::PortC(Pin::Pin2) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC2_M) | reg::GPIO_PCTL_PC2_T5CCP0);
            },
            &mut PinPort::PortC(Pin::Pin3) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PC3_M) | reg::GPIO_PCTL_PC3_T5CCP1);
            },
            &mut PinPort::PortF(Pin::Pin1) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PF1_M) | reg::GPIO_PCTL_PF1_T0CCP1);
            },
            &mut PinPort::PortF(Pin::Pin2) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PF2_M) | reg::GPIO_PCTL_PF2_T1CCP0);
            },
            &mut PinPort::PortF(Pin::Pin3) => unsafe {
                gpio_reg
                    .pctl
                    .modify(|r| (r & !reg::GPIO_PCTL_PF3_M) | reg::GPIO_PCTL_PF3_T1CCP1);
            },
            &mut PinPort::PortF(Pin::Pin4) => unsafe {
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

impl StatefulOutputPin for PinPort {
    /// Check if pin is currently set high.
    fn is_set_high(&self) -> bool {
        self.read() == Level::High
    }

    /// Check if pin is currently set low.
    fn is_set_low(&self) -> bool {
        self.read() == Level::Low
    }
}

impl OutputPin for PinPort {
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
pub fn enable_uart(id: UartId) {
    match id {
        UartId::Uart0 => {
            enable_port(&mut PinPort::PortA(Pin::Pin0));
            let gpio_reg = get_port_registers(&mut PinPort::PortA(Pin::Pin1));
            unsafe {
                gpio_reg.afsel.modify(|r| r | (1 << 1) | (1 << 0));
                gpio_reg.den.modify(|r| r | (1 << 1) | (1 << 0));
                gpio_reg.pctl.modify(|r| {
                    (r & !(reg::GPIO_PCTL_PA0_M | reg::GPIO_PCTL_PA1_M))
                        | (reg::GPIO_PCTL_PA0_U0RX | reg::GPIO_PCTL_PA1_U0TX)
                });
            }
        }
        UartId::Uart1 => {
            unimplemented!();
        }
        UartId::Uart2 => {
            unimplemented!();
        }
        UartId::Uart3 => {
            unimplemented!();
        }
        UartId::Uart4 => {
            unimplemented!();
        }
        UartId::Uart5 => {
            unimplemented!();
        }
        UartId::Uart6 => {
            unimplemented!();
        }
        UartId::Uart7 => {
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
fn get_port_mask(port: &PinPort) -> usize {
    match port {
        &PinPort::PortA(_) => 1 << 0,
        &PinPort::PortB(_) => 1 << 1,
        &PinPort::PortC(_) => 1 << 2,
        &PinPort::PortD(_) => 1 << 3,
        &PinPort::PortE(_) => 1 << 4,
        &PinPort::PortF(_) => 1 << 5,
    }
}

/// Convert a pin to a bit mask
/// Pin0 is 0, Pin7 is 128
fn get_pin_mask(pinport: &PinPort) -> usize {
    let pin = match pinport {
        &PinPort::PortA(ref x) => x,
        &PinPort::PortB(ref x) => x,
        &PinPort::PortC(ref x) => x,
        &PinPort::PortD(ref x) => x,
        &PinPort::PortE(ref x) => x,
        &PinPort::PortF(ref x) => x,
    };
    match *pin {
        Pin::Pin0 => 1 << 0 as usize,
        Pin::Pin1 => 1 << 1 as usize,
        Pin::Pin2 => 1 << 2 as usize,
        Pin::Pin3 => 1 << 3 as usize,
        Pin::Pin4 => 1 << 4 as usize,
        Pin::Pin5 => 1 << 5 as usize,
        Pin::Pin6 => 1 << 6 as usize,
        Pin::Pin7 => 1 << 7 as usize,
    }
}

fn get_pctl_mask(pinport: &PinPort) -> usize {
    let pin = match pinport {
        &PinPort::PortA(ref x) => x,
        &PinPort::PortB(ref x) => x,
        &PinPort::PortC(ref x) => x,
        &PinPort::PortD(ref x) => x,
        &PinPort::PortE(ref x) => x,
        &PinPort::PortF(ref x) => x,
    };
    match pin {
        &Pin::Pin0 => 7 << 0 as usize,
        &Pin::Pin1 => 7 << 4 as usize,
        &Pin::Pin2 => 7 << 8 as usize,
        &Pin::Pin3 => 7 << 12 as usize,
        &Pin::Pin4 => 7 << 16 as usize,
        &Pin::Pin5 => 7 << 20 as usize,
        &Pin::Pin6 => 7 << 24 as usize,
        &Pin::Pin7 => 7 << 28 as usize,
    }
}

fn enable_port(port: &mut PinPort) {
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

fn force_gpio_periph(pinport: &mut PinPort, gpio_reg: &mut reg::GpioRegisters) {
    let mask = get_pin_mask(pinport);
    let pctl_mask = get_pctl_mask(pinport);
    unsafe {
        gpio_reg.afsel.modify(|r| r & !mask);
        gpio_reg.pctl.modify(|r| r & !pctl_mask);
    }
}

fn make_input(pinport: &mut PinPort) {
    enable_port(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    force_gpio_periph(pinport, gpio_reg);
    unsafe {
        if *pinport == PinPort::PortF(Pin::Pin0) {
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

fn make_peripheral(pinport: &mut PinPort) {
    enable_port(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    unsafe {
        gpio_reg.afsel.modify(|r| r | mask);
        gpio_reg.den.modify(|r| r | mask);
        gpio_reg.dir.modify(|r| r & !mask);
    }
}

fn make_input_pullup(pinport: &mut PinPort) {
    make_input(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    unsafe {
        gpio_reg.dr2r.modify(|r| r | mask);
        gpio_reg.pur.modify(|r| r | mask);
    }
}

fn make_input_pulldown(pinport: &mut PinPort) {
    make_input(pinport);
    let mask = get_pin_mask(pinport);
    let gpio_reg = get_port_registers(pinport);
    unsafe {
        gpio_reg.dr2r.modify(|r| r | mask);
        gpio_reg.pur.modify(|r| r & !mask);
    }
}

fn make_output(pinport: &mut PinPort, level: Level) {
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
fn get_port_registers(port: &PinPort) -> &'static mut reg::GpioRegisters {
    unsafe {
        match port {
            &PinPort::PortA(_) => &mut *(reg::GPIO_PORTA_DATA_BITS_R as *mut reg::GpioRegisters),
            &PinPort::PortB(_) => &mut *(reg::GPIO_PORTB_DATA_BITS_R as *mut reg::GpioRegisters),
            &PinPort::PortC(_) => &mut *(reg::GPIO_PORTC_DATA_BITS_R as *mut reg::GpioRegisters),
            &PinPort::PortD(_) => &mut *(reg::GPIO_PORTD_DATA_BITS_R as *mut reg::GpioRegisters),
            &PinPort::PortE(_) => &mut *(reg::GPIO_PORTE_DATA_BITS_R as *mut reg::GpioRegisters),
            &PinPort::PortF(_) => &mut *(reg::GPIO_PORTF_DATA_BITS_R as *mut reg::GpioRegisters),
        }
    }
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
