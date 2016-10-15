// *****************************************************************************
//
// lm4f120h5qr.h - LM4F120H5QR Register Definitions
//
// Copyright (c) 2011-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 9453 of the Stellaris Firmware Development Package.
//
// Ported from C to Rust by theJPster <github@thejpster.org.uk>
//
// *****************************************************************************

#![allow(dead_code)]

use volatile_register::{RO, RW};

// *****************************************************************************
//
// Watchdog Timer registers (WATCHDOG0)
//
// *****************************************************************************
pub const WATCHDOG0_LOAD_R: *mut usize = 0x40000000 as *mut usize;
pub const WATCHDOG0_VALUE_R: *mut usize = 0x40000004 as *mut usize;
pub const WATCHDOG0_CTL_R: *mut usize = 0x40000008 as *mut usize;
pub const WATCHDOG0_ICR_R: *mut usize = 0x4000000C as *mut usize;
pub const WATCHDOG0_RIS_R: *mut usize = 0x40000010 as *mut usize;
pub const WATCHDOG0_MIS_R: *mut usize = 0x40000014 as *mut usize;
pub const WATCHDOG0_TEST_R: *mut usize = 0x40000418 as *mut usize;
pub const WATCHDOG0_LOCK_R: *mut usize = 0x40000C00 as *mut usize;

// *****************************************************************************
//
// Watchdog Timer registers (WATCHDOG1)
//
// *****************************************************************************
pub const WATCHDOG1_LOAD_R: *mut usize = 0x40001000 as *mut usize;
pub const WATCHDOG1_VALUE_R: *mut usize = 0x40001004 as *mut usize;
pub const WATCHDOG1_CTL_R: *mut usize = 0x40001008 as *mut usize;
pub const WATCHDOG1_ICR_R: *mut usize = 0x4000100C as *mut usize;
pub const WATCHDOG1_RIS_R: *mut usize = 0x40001010 as *mut usize;
pub const WATCHDOG1_MIS_R: *mut usize = 0x40001014 as *mut usize;
pub const WATCHDOG1_TEST_R: *mut usize = 0x40001418 as *mut usize;
pub const WATCHDOG1_LOCK_R: *mut usize = 0x40001C00 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTA)
//
// *****************************************************************************
pub const GPIO_PORTA_DATA_BITS_R: *mut usize = 0x40004000 as *mut usize;
pub const GPIO_PORTA_DATA_R: *mut usize = 0x400043FC as *mut usize;
pub const GPIO_PORTA_DIR_R: *mut usize = 0x40004400 as *mut usize;
pub const GPIO_PORTA_IS_R: *mut usize = 0x40004404 as *mut usize;
pub const GPIO_PORTA_IBE_R: *mut usize = 0x40004408 as *mut usize;
pub const GPIO_PORTA_IEV_R: *mut usize = 0x4000440C as *mut usize;
pub const GPIO_PORTA_IM_R: *mut usize = 0x40004410 as *mut usize;
pub const GPIO_PORTA_RIS_R: *mut usize = 0x40004414 as *mut usize;
pub const GPIO_PORTA_MIS_R: *mut usize = 0x40004418 as *mut usize;
pub const GPIO_PORTA_ICR_R: *mut usize = 0x4000441C as *mut usize;
pub const GPIO_PORTA_AFSEL_R: *mut usize = 0x40004420 as *mut usize;
pub const GPIO_PORTA_DR2R_R: *mut usize = 0x40004500 as *mut usize;
pub const GPIO_PORTA_DR4R_R: *mut usize = 0x40004504 as *mut usize;
pub const GPIO_PORTA_DR8R_R: *mut usize = 0x40004508 as *mut usize;
pub const GPIO_PORTA_ODR_R: *mut usize = 0x4000450C as *mut usize;
pub const GPIO_PORTA_PUR_R: *mut usize = 0x40004510 as *mut usize;
pub const GPIO_PORTA_PDR_R: *mut usize = 0x40004514 as *mut usize;
pub const GPIO_PORTA_SLR_R: *mut usize = 0x40004518 as *mut usize;
pub const GPIO_PORTA_DEN_R: *mut usize = 0x4000451C as *mut usize;
pub const GPIO_PORTA_LOCK_R: *mut usize = 0x40004520 as *mut usize;
pub const GPIO_PORTA_CR_R: *mut usize = 0x40004524 as *mut usize;
pub const GPIO_PORTA_AMSEL_R: *mut usize = 0x40004528 as *mut usize;
pub const GPIO_PORTA_PCTL_R: *mut usize = 0x4000452C as *mut usize;
pub const GPIO_PORTA_ADCCTL_R: *mut usize = 0x40004530 as *mut usize;
pub const GPIO_PORTA_DMACTL_R: *mut usize = 0x40004534 as *mut usize;
pub const GPIO_PORTA_SI_R: *mut usize = 0x40004538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTB)
//
// *****************************************************************************
pub const GPIO_PORTB_DATA_BITS_R: *mut usize = 0x40005000 as *mut usize;
pub const GPIO_PORTB_DATA_R: *mut usize = 0x400053FC as *mut usize;
pub const GPIO_PORTB_DIR_R: *mut usize = 0x40005400 as *mut usize;
pub const GPIO_PORTB_IS_R: *mut usize = 0x40005404 as *mut usize;
pub const GPIO_PORTB_IBE_R: *mut usize = 0x40005408 as *mut usize;
pub const GPIO_PORTB_IEV_R: *mut usize = 0x4000540C as *mut usize;
pub const GPIO_PORTB_IM_R: *mut usize = 0x40005410 as *mut usize;
pub const GPIO_PORTB_RIS_R: *mut usize = 0x40005414 as *mut usize;
pub const GPIO_PORTB_MIS_R: *mut usize = 0x40005418 as *mut usize;
pub const GPIO_PORTB_ICR_R: *mut usize = 0x4000541C as *mut usize;
pub const GPIO_PORTB_AFSEL_R: *mut usize = 0x40005420 as *mut usize;
pub const GPIO_PORTB_DR2R_R: *mut usize = 0x40005500 as *mut usize;
pub const GPIO_PORTB_DR4R_R: *mut usize = 0x40005504 as *mut usize;
pub const GPIO_PORTB_DR8R_R: *mut usize = 0x40005508 as *mut usize;
pub const GPIO_PORTB_ODR_R: *mut usize = 0x4000550C as *mut usize;
pub const GPIO_PORTB_PUR_R: *mut usize = 0x40005510 as *mut usize;
pub const GPIO_PORTB_PDR_R: *mut usize = 0x40005514 as *mut usize;
pub const GPIO_PORTB_SLR_R: *mut usize = 0x40005518 as *mut usize;
pub const GPIO_PORTB_DEN_R: *mut usize = 0x4000551C as *mut usize;
pub const GPIO_PORTB_LOCK_R: *mut usize = 0x40005520 as *mut usize;
pub const GPIO_PORTB_CR_R: *mut usize = 0x40005524 as *mut usize;
pub const GPIO_PORTB_AMSEL_R: *mut usize = 0x40005528 as *mut usize;
pub const GPIO_PORTB_PCTL_R: *mut usize = 0x4000552C as *mut usize;
pub const GPIO_PORTB_ADCCTL_R: *mut usize = 0x40005530 as *mut usize;
pub const GPIO_PORTB_DMACTL_R: *mut usize = 0x40005534 as *mut usize;
pub const GPIO_PORTB_SI_R: *mut usize = 0x40005538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTC)
//
// *****************************************************************************
pub const GPIO_PORTC_DATA_BITS_R: *mut usize = 0x40006000 as *mut usize;
pub const GPIO_PORTC_DATA_R: *mut usize = 0x400063FC as *mut usize;
pub const GPIO_PORTC_DIR_R: *mut usize = 0x40006400 as *mut usize;
pub const GPIO_PORTC_IS_R: *mut usize = 0x40006404 as *mut usize;
pub const GPIO_PORTC_IBE_R: *mut usize = 0x40006408 as *mut usize;
pub const GPIO_PORTC_IEV_R: *mut usize = 0x4000640C as *mut usize;
pub const GPIO_PORTC_IM_R: *mut usize = 0x40006410 as *mut usize;
pub const GPIO_PORTC_RIS_R: *mut usize = 0x40006414 as *mut usize;
pub const GPIO_PORTC_MIS_R: *mut usize = 0x40006418 as *mut usize;
pub const GPIO_PORTC_ICR_R: *mut usize = 0x4000641C as *mut usize;
pub const GPIO_PORTC_AFSEL_R: *mut usize = 0x40006420 as *mut usize;
pub const GPIO_PORTC_DR2R_R: *mut usize = 0x40006500 as *mut usize;
pub const GPIO_PORTC_DR4R_R: *mut usize = 0x40006504 as *mut usize;
pub const GPIO_PORTC_DR8R_R: *mut usize = 0x40006508 as *mut usize;
pub const GPIO_PORTC_ODR_R: *mut usize = 0x4000650C as *mut usize;
pub const GPIO_PORTC_PUR_R: *mut usize = 0x40006510 as *mut usize;
pub const GPIO_PORTC_PDR_R: *mut usize = 0x40006514 as *mut usize;
pub const GPIO_PORTC_SLR_R: *mut usize = 0x40006518 as *mut usize;
pub const GPIO_PORTC_DEN_R: *mut usize = 0x4000651C as *mut usize;
pub const GPIO_PORTC_LOCK_R: *mut usize = 0x40006520 as *mut usize;
pub const GPIO_PORTC_CR_R: *mut usize = 0x40006524 as *mut usize;
pub const GPIO_PORTC_AMSEL_R: *mut usize = 0x40006528 as *mut usize;
pub const GPIO_PORTC_PCTL_R: *mut usize = 0x4000652C as *mut usize;
pub const GPIO_PORTC_ADCCTL_R: *mut usize = 0x40006530 as *mut usize;
pub const GPIO_PORTC_DMACTL_R: *mut usize = 0x40006534 as *mut usize;
pub const GPIO_PORTC_SI_R: *mut usize = 0x40006538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTD)
//
// *****************************************************************************
pub const GPIO_PORTD_DATA_BITS_R: *mut usize = 0x40007000 as *mut usize;
pub const GPIO_PORTD_DATA_R: *mut usize = 0x400073FC as *mut usize;
pub const GPIO_PORTD_DIR_R: *mut usize = 0x40007400 as *mut usize;
pub const GPIO_PORTD_IS_R: *mut usize = 0x40007404 as *mut usize;
pub const GPIO_PORTD_IBE_R: *mut usize = 0x40007408 as *mut usize;
pub const GPIO_PORTD_IEV_R: *mut usize = 0x4000740C as *mut usize;
pub const GPIO_PORTD_IM_R: *mut usize = 0x40007410 as *mut usize;
pub const GPIO_PORTD_RIS_R: *mut usize = 0x40007414 as *mut usize;
pub const GPIO_PORTD_MIS_R: *mut usize = 0x40007418 as *mut usize;
pub const GPIO_PORTD_ICR_R: *mut usize = 0x4000741C as *mut usize;
pub const GPIO_PORTD_AFSEL_R: *mut usize = 0x40007420 as *mut usize;
pub const GPIO_PORTD_DR2R_R: *mut usize = 0x40007500 as *mut usize;
pub const GPIO_PORTD_DR4R_R: *mut usize = 0x40007504 as *mut usize;
pub const GPIO_PORTD_DR8R_R: *mut usize = 0x40007508 as *mut usize;
pub const GPIO_PORTD_ODR_R: *mut usize = 0x4000750C as *mut usize;
pub const GPIO_PORTD_PUR_R: *mut usize = 0x40007510 as *mut usize;
pub const GPIO_PORTD_PDR_R: *mut usize = 0x40007514 as *mut usize;
pub const GPIO_PORTD_SLR_R: *mut usize = 0x40007518 as *mut usize;
pub const GPIO_PORTD_DEN_R: *mut usize = 0x4000751C as *mut usize;
pub const GPIO_PORTD_LOCK_R: *mut usize = 0x40007520 as *mut usize;
pub const GPIO_PORTD_CR_R: *mut usize = 0x40007524 as *mut usize;
pub const GPIO_PORTD_AMSEL_R: *mut usize = 0x40007528 as *mut usize;
pub const GPIO_PORTD_PCTL_R: *mut usize = 0x4000752C as *mut usize;
pub const GPIO_PORTD_ADCCTL_R: *mut usize = 0x40007530 as *mut usize;
pub const GPIO_PORTD_DMACTL_R: *mut usize = 0x40007534 as *mut usize;
pub const GPIO_PORTD_SI_R: *mut usize = 0x40007538 as *mut usize;

// *****************************************************************************
//
// SSI registers (SSI0)
//
// *****************************************************************************
pub const SSI0_CR0_R: *mut usize = 0x40008000 as *mut usize;
pub const SSI0_CR1_R: *mut usize = 0x40008004 as *mut usize;
pub const SSI0_DR_R: *mut usize = 0x40008008 as *mut usize;
pub const SSI0_SR_R: *mut usize = 0x4000800C as *mut usize;
pub const SSI0_CPSR_R: *mut usize = 0x40008010 as *mut usize;
pub const SSI0_IM_R: *mut usize = 0x40008014 as *mut usize;
pub const SSI0_RIS_R: *mut usize = 0x40008018 as *mut usize;
pub const SSI0_MIS_R: *mut usize = 0x4000801C as *mut usize;
pub const SSI0_ICR_R: *mut usize = 0x40008020 as *mut usize;
pub const SSI0_DMACTL_R: *mut usize = 0x40008024 as *mut usize;
pub const SSI0_CC_R: *mut usize = 0x40008FC8 as *mut usize;

// *****************************************************************************
//
// SSI registers (SSI1)
//
// *****************************************************************************
pub const SSI1_CR0_R: *mut usize = 0x40009000 as *mut usize;
pub const SSI1_CR1_R: *mut usize = 0x40009004 as *mut usize;
pub const SSI1_DR_R: *mut usize = 0x40009008 as *mut usize;
pub const SSI1_SR_R: *mut usize = 0x4000900C as *mut usize;
pub const SSI1_CPSR_R: *mut usize = 0x40009010 as *mut usize;
pub const SSI1_IM_R: *mut usize = 0x40009014 as *mut usize;
pub const SSI1_RIS_R: *mut usize = 0x40009018 as *mut usize;
pub const SSI1_MIS_R: *mut usize = 0x4000901C as *mut usize;
pub const SSI1_ICR_R: *mut usize = 0x40009020 as *mut usize;
pub const SSI1_DMACTL_R: *mut usize = 0x40009024 as *mut usize;
pub const SSI1_CC_R: *mut usize = 0x40009FC8 as *mut usize;

// *****************************************************************************
//
// SSI registers (SSI2)
//
// *****************************************************************************
pub const SSI2_CR0_R: *mut usize = 0x4000A000 as *mut usize;
pub const SSI2_CR1_R: *mut usize = 0x4000A004 as *mut usize;
pub const SSI2_DR_R: *mut usize = 0x4000A008 as *mut usize;
pub const SSI2_SR_R: *mut usize = 0x4000A00C as *mut usize;
pub const SSI2_CPSR_R: *mut usize = 0x4000A010 as *mut usize;
pub const SSI2_IM_R: *mut usize = 0x4000A014 as *mut usize;
pub const SSI2_RIS_R: *mut usize = 0x4000A018 as *mut usize;
pub const SSI2_MIS_R: *mut usize = 0x4000A01C as *mut usize;
pub const SSI2_ICR_R: *mut usize = 0x4000A020 as *mut usize;
pub const SSI2_DMACTL_R: *mut usize = 0x4000A024 as *mut usize;
pub const SSI2_CC_R: *mut usize = 0x4000AFC8 as *mut usize;

// *****************************************************************************
//
// SSI registers (SSI3)
//
// *****************************************************************************
pub const SSI3_CR0_R: *mut usize = 0x4000B000 as *mut usize;
pub const SSI3_CR1_R: *mut usize = 0x4000B004 as *mut usize;
pub const SSI3_DR_R: *mut usize = 0x4000B008 as *mut usize;
pub const SSI3_SR_R: *mut usize = 0x4000B00C as *mut usize;
pub const SSI3_CPSR_R: *mut usize = 0x4000B010 as *mut usize;
pub const SSI3_IM_R: *mut usize = 0x4000B014 as *mut usize;
pub const SSI3_RIS_R: *mut usize = 0x4000B018 as *mut usize;
pub const SSI3_MIS_R: *mut usize = 0x4000B01C as *mut usize;
pub const SSI3_ICR_R: *mut usize = 0x4000B020 as *mut usize;
pub const SSI3_DMACTL_R: *mut usize = 0x4000B024 as *mut usize;
pub const SSI3_CC_R: *mut usize = 0x4000BFC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART0)
//
// *****************************************************************************

/// The data register. Data written here is added to the FIFO.
/// Reading here reads data + 4-bit status from the IFO.
pub const UART0_DR_R: *mut usize = 0x4000C000 as *mut usize;

/// UART Receive Statu
pub const UART0_RSR_R: *mut usize = 0x4000C004 as *mut usize;

/// UART Error Clea
pub const UART0_ECR_R: *mut usize = 0x4000C004 as *mut usize;

/// UART Flag
pub const UART0_FR_R: *mut usize = 0x4000C018 as *mut usize;

/// UART IrDA Low-Power Registe
pub const UART0_ILPR_R: *mut usize = 0x4000C020 as *mut usize;

/// UART Integer Baud-Rate Diviso
pub const UART0_IBRD_R: *mut usize = 0x4000C024 as *mut usize;

/// UART Fractional Baud-Rate Diviso
pub const UART0_FBRD_R: *mut usize = 0x4000C028 as *mut usize;

/// UART Line Contro
pub const UART0_LCRH_R: *mut usize = 0x4000C02C as *mut usize;

/// UART Contro
pub const UART0_CTL_R: *mut usize = 0x4000C030 as *mut usize;

/// UART Interrupt FIFO Level Selec
pub const UART0_IFLS_R: *mut usize = 0x4000C034 as *mut usize;

/// UART Interrupt Mas
pub const UART0_IM_R: *mut usize = 0x4000C038 as *mut usize;

/// UART Raw Interrupt Statu
pub const UART0_RIS_R: *mut usize = 0x4000C03C as *mut usize;

/// UART Masked Interrupt Statu
pub const UART0_MIS_R: *mut usize = 0x4000C040 as *mut usize;

/// UART Interrupt Clea
pub const UART0_ICR_R: *mut usize = 0x4000C044 as *mut usize;

/// UART DMA Contro
pub const UART0_DMACTL_R: *mut usize = 0x4000C048 as *mut usize;

/// ??? - Not in datashee
pub const UART0_LCTL_R: *mut usize = 0x4000C090 as *mut usize;

/// ??? - Not in datashee
pub const UART0_LSS_R: *mut usize = 0x4000C094 as *mut usize;

/// ??? - Not in datashee
pub const UART0_LTIM_R: *mut usize = 0x4000C098 as *mut usize;

/// UART 9-bit Self Addres
pub const UART0_9BITADDR_R: *mut usize = 0x4000C0A4 as *mut usize;

/// UART 9-bit Self Address Mas
pub const UART0_9BITAMASK_R: *mut usize = 0x4000C0A8 as *mut usize;

/// UART Peripheral Propertie
pub const UART0_PP_R: *mut usize = 0x4000CFC0 as *mut usize;

/// UART Clock Configuratio
pub const UART0_CC_R: *mut usize = 0x4000CFC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART1)
//
// *****************************************************************************
pub const UART1_DR_R: *mut usize = 0x4000D000 as *mut usize;
pub const UART1_RSR_R: *mut usize = 0x4000D004 as *mut usize;
pub const UART1_ECR_R: *mut usize = 0x4000D004 as *mut usize;
pub const UART1_FR_R: *mut usize = 0x4000D018 as *mut usize;
pub const UART1_ILPR_R: *mut usize = 0x4000D020 as *mut usize;
pub const UART1_IBRD_R: *mut usize = 0x4000D024 as *mut usize;
pub const UART1_FBRD_R: *mut usize = 0x4000D028 as *mut usize;
pub const UART1_LCRH_R: *mut usize = 0x4000D02C as *mut usize;
pub const UART1_CTL_R: *mut usize = 0x4000D030 as *mut usize;
pub const UART1_IFLS_R: *mut usize = 0x4000D034 as *mut usize;
pub const UART1_IM_R: *mut usize = 0x4000D038 as *mut usize;
pub const UART1_RIS_R: *mut usize = 0x4000D03C as *mut usize;
pub const UART1_MIS_R: *mut usize = 0x4000D040 as *mut usize;
pub const UART1_ICR_R: *mut usize = 0x4000D044 as *mut usize;
pub const UART1_DMACTL_R: *mut usize = 0x4000D048 as *mut usize;
pub const UART1_LCTL_R: *mut usize = 0x4000D090 as *mut usize;
pub const UART1_LSS_R: *mut usize = 0x4000D094 as *mut usize;
pub const UART1_LTIM_R: *mut usize = 0x4000D098 as *mut usize;
pub const UART1_9BITADDR_R: *mut usize = 0x4000D0A4 as *mut usize;
pub const UART1_9BITAMASK_R: *mut usize = 0x4000D0A8 as *mut usize;
pub const UART1_PP_R: *mut usize = 0x4000DFC0 as *mut usize;
pub const UART1_CC_R: *mut usize = 0x4000DFC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART2)
//
// *****************************************************************************
pub const UART2_DR_R: *mut usize = 0x4000E000 as *mut usize;
pub const UART2_RSR_R: *mut usize = 0x4000E004 as *mut usize;
pub const UART2_ECR_R: *mut usize = 0x4000E004 as *mut usize;
pub const UART2_FR_R: *mut usize = 0x4000E018 as *mut usize;
pub const UART2_ILPR_R: *mut usize = 0x4000E020 as *mut usize;
pub const UART2_IBRD_R: *mut usize = 0x4000E024 as *mut usize;
pub const UART2_FBRD_R: *mut usize = 0x4000E028 as *mut usize;
pub const UART2_LCRH_R: *mut usize = 0x4000E02C as *mut usize;
pub const UART2_CTL_R: *mut usize = 0x4000E030 as *mut usize;
pub const UART2_IFLS_R: *mut usize = 0x4000E034 as *mut usize;
pub const UART2_IM_R: *mut usize = 0x4000E038 as *mut usize;
pub const UART2_RIS_R: *mut usize = 0x4000E03C as *mut usize;
pub const UART2_MIS_R: *mut usize = 0x4000E040 as *mut usize;
pub const UART2_ICR_R: *mut usize = 0x4000E044 as *mut usize;
pub const UART2_DMACTL_R: *mut usize = 0x4000E048 as *mut usize;
pub const UART2_LCTL_R: *mut usize = 0x4000E090 as *mut usize;
pub const UART2_LSS_R: *mut usize = 0x4000E094 as *mut usize;
pub const UART2_LTIM_R: *mut usize = 0x4000E098 as *mut usize;
pub const UART2_9BITADDR_R: *mut usize = 0x4000E0A4 as *mut usize;
pub const UART2_9BITAMASK_R: *mut usize = 0x4000E0A8 as *mut usize;
pub const UART2_PP_R: *mut usize = 0x4000EFC0 as *mut usize;
pub const UART2_CC_R: *mut usize = 0x4000EFC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART3)
//
// *****************************************************************************
pub const UART3_DR_R: *mut usize = 0x4000F000 as *mut usize;
pub const UART3_RSR_R: *mut usize = 0x4000F004 as *mut usize;
pub const UART3_ECR_R: *mut usize = 0x4000F004 as *mut usize;
pub const UART3_FR_R: *mut usize = 0x4000F018 as *mut usize;
pub const UART3_ILPR_R: *mut usize = 0x4000F020 as *mut usize;
pub const UART3_IBRD_R: *mut usize = 0x4000F024 as *mut usize;
pub const UART3_FBRD_R: *mut usize = 0x4000F028 as *mut usize;
pub const UART3_LCRH_R: *mut usize = 0x4000F02C as *mut usize;
pub const UART3_CTL_R: *mut usize = 0x4000F030 as *mut usize;
pub const UART3_IFLS_R: *mut usize = 0x4000F034 as *mut usize;
pub const UART3_IM_R: *mut usize = 0x4000F038 as *mut usize;
pub const UART3_RIS_R: *mut usize = 0x4000F03C as *mut usize;
pub const UART3_MIS_R: *mut usize = 0x4000F040 as *mut usize;
pub const UART3_ICR_R: *mut usize = 0x4000F044 as *mut usize;
pub const UART3_DMACTL_R: *mut usize = 0x4000F048 as *mut usize;
pub const UART3_LCTL_R: *mut usize = 0x4000F090 as *mut usize;
pub const UART3_LSS_R: *mut usize = 0x4000F094 as *mut usize;
pub const UART3_LTIM_R: *mut usize = 0x4000F098 as *mut usize;
pub const UART3_9BITADDR_R: *mut usize = 0x4000F0A4 as *mut usize;
pub const UART3_9BITAMASK_R: *mut usize = 0x4000F0A8 as *mut usize;
pub const UART3_PP_R: *mut usize = 0x4000FFC0 as *mut usize;
pub const UART3_CC_R: *mut usize = 0x4000FFC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART4)
//
// *****************************************************************************
pub const UART4_DR_R: *mut usize = 0x40010000 as *mut usize;
pub const UART4_RSR_R: *mut usize = 0x40010004 as *mut usize;
pub const UART4_ECR_R: *mut usize = 0x40010004 as *mut usize;
pub const UART4_FR_R: *mut usize = 0x40010018 as *mut usize;
pub const UART4_ILPR_R: *mut usize = 0x40010020 as *mut usize;
pub const UART4_IBRD_R: *mut usize = 0x40010024 as *mut usize;
pub const UART4_FBRD_R: *mut usize = 0x40010028 as *mut usize;
pub const UART4_LCRH_R: *mut usize = 0x4001002C as *mut usize;
pub const UART4_CTL_R: *mut usize = 0x40010030 as *mut usize;
pub const UART4_IFLS_R: *mut usize = 0x40010034 as *mut usize;
pub const UART4_IM_R: *mut usize = 0x40010038 as *mut usize;
pub const UART4_RIS_R: *mut usize = 0x4001003C as *mut usize;
pub const UART4_MIS_R: *mut usize = 0x40010040 as *mut usize;
pub const UART4_ICR_R: *mut usize = 0x40010044 as *mut usize;
pub const UART4_DMACTL_R: *mut usize = 0x40010048 as *mut usize;
pub const UART4_LCTL_R: *mut usize = 0x40010090 as *mut usize;
pub const UART4_LSS_R: *mut usize = 0x40010094 as *mut usize;
pub const UART4_LTIM_R: *mut usize = 0x40010098 as *mut usize;
pub const UART4_9BITADDR_R: *mut usize = 0x400100A4 as *mut usize;
pub const UART4_9BITAMASK_R: *mut usize = 0x400100A8 as *mut usize;
pub const UART4_PP_R: *mut usize = 0x40010FC0 as *mut usize;
pub const UART4_CC_R: *mut usize = 0x40010FC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART5)
//
// *****************************************************************************
pub const UART5_DR_R: *mut usize = 0x40011000 as *mut usize;
pub const UART5_RSR_R: *mut usize = 0x40011004 as *mut usize;
pub const UART5_ECR_R: *mut usize = 0x40011004 as *mut usize;
pub const UART5_FR_R: *mut usize = 0x40011018 as *mut usize;
pub const UART5_ILPR_R: *mut usize = 0x40011020 as *mut usize;
pub const UART5_IBRD_R: *mut usize = 0x40011024 as *mut usize;
pub const UART5_FBRD_R: *mut usize = 0x40011028 as *mut usize;
pub const UART5_LCRH_R: *mut usize = 0x4001102C as *mut usize;
pub const UART5_CTL_R: *mut usize = 0x40011030 as *mut usize;
pub const UART5_IFLS_R: *mut usize = 0x40011034 as *mut usize;
pub const UART5_IM_R: *mut usize = 0x40011038 as *mut usize;
pub const UART5_RIS_R: *mut usize = 0x4001103C as *mut usize;
pub const UART5_MIS_R: *mut usize = 0x40011040 as *mut usize;
pub const UART5_ICR_R: *mut usize = 0x40011044 as *mut usize;
pub const UART5_DMACTL_R: *mut usize = 0x40011048 as *mut usize;
pub const UART5_LCTL_R: *mut usize = 0x40011090 as *mut usize;
pub const UART5_LSS_R: *mut usize = 0x40011094 as *mut usize;
pub const UART5_LTIM_R: *mut usize = 0x40011098 as *mut usize;
pub const UART5_9BITADDR_R: *mut usize = 0x400110A4 as *mut usize;
pub const UART5_9BITAMASK_R: *mut usize = 0x400110A8 as *mut usize;
pub const UART5_PP_R: *mut usize = 0x40011FC0 as *mut usize;
pub const UART5_CC_R: *mut usize = 0x40011FC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART6)
//
// *****************************************************************************
pub const UART6_DR_R: *mut usize = 0x40012000 as *mut usize;
pub const UART6_RSR_R: *mut usize = 0x40012004 as *mut usize;
pub const UART6_ECR_R: *mut usize = 0x40012004 as *mut usize;
pub const UART6_FR_R: *mut usize = 0x40012018 as *mut usize;
pub const UART6_ILPR_R: *mut usize = 0x40012020 as *mut usize;
pub const UART6_IBRD_R: *mut usize = 0x40012024 as *mut usize;
pub const UART6_FBRD_R: *mut usize = 0x40012028 as *mut usize;
pub const UART6_LCRH_R: *mut usize = 0x4001202C as *mut usize;
pub const UART6_CTL_R: *mut usize = 0x40012030 as *mut usize;
pub const UART6_IFLS_R: *mut usize = 0x40012034 as *mut usize;
pub const UART6_IM_R: *mut usize = 0x40012038 as *mut usize;
pub const UART6_RIS_R: *mut usize = 0x4001203C as *mut usize;
pub const UART6_MIS_R: *mut usize = 0x40012040 as *mut usize;
pub const UART6_ICR_R: *mut usize = 0x40012044 as *mut usize;
pub const UART6_DMACTL_R: *mut usize = 0x40012048 as *mut usize;
pub const UART6_LCTL_R: *mut usize = 0x40012090 as *mut usize;
pub const UART6_LSS_R: *mut usize = 0x40012094 as *mut usize;
pub const UART6_LTIM_R: *mut usize = 0x40012098 as *mut usize;
pub const UART6_9BITADDR_R: *mut usize = 0x400120A4 as *mut usize;
pub const UART6_9BITAMASK_R: *mut usize = 0x400120A8 as *mut usize;
pub const UART6_PP_R: *mut usize = 0x40012FC0 as *mut usize;
pub const UART6_CC_R: *mut usize = 0x40012FC8 as *mut usize;

// *****************************************************************************
//
// UART registers (UART7)
//
// *****************************************************************************
pub const UART7_DR_R: *mut usize = 0x40013000 as *mut usize;
pub const UART7_RSR_R: *mut usize = 0x40013004 as *mut usize;
pub const UART7_ECR_R: *mut usize = 0x40013004 as *mut usize;
pub const UART7_FR_R: *mut usize = 0x40013018 as *mut usize;
pub const UART7_ILPR_R: *mut usize = 0x40013020 as *mut usize;
pub const UART7_IBRD_R: *mut usize = 0x40013024 as *mut usize;
pub const UART7_FBRD_R: *mut usize = 0x40013028 as *mut usize;
pub const UART7_LCRH_R: *mut usize = 0x4001302C as *mut usize;
pub const UART7_CTL_R: *mut usize = 0x40013030 as *mut usize;
pub const UART7_IFLS_R: *mut usize = 0x40013034 as *mut usize;
pub const UART7_IM_R: *mut usize = 0x40013038 as *mut usize;
pub const UART7_RIS_R: *mut usize = 0x4001303C as *mut usize;
pub const UART7_MIS_R: *mut usize = 0x40013040 as *mut usize;
pub const UART7_ICR_R: *mut usize = 0x40013044 as *mut usize;
pub const UART7_DMACTL_R: *mut usize = 0x40013048 as *mut usize;
pub const UART7_LCTL_R: *mut usize = 0x40013090 as *mut usize;
pub const UART7_LSS_R: *mut usize = 0x40013094 as *mut usize;
pub const UART7_LTIM_R: *mut usize = 0x40013098 as *mut usize;
pub const UART7_9BITADDR_R: *mut usize = 0x400130A4 as *mut usize;
pub const UART7_9BITAMASK_R: *mut usize = 0x400130A8 as *mut usize;
pub const UART7_PP_R: *mut usize = 0x40013FC0 as *mut usize;
pub const UART7_CC_R: *mut usize = 0x40013FC8 as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C0 MASTER)
//
// *****************************************************************************
pub const I2C0_MASTER_MSA_R: *mut usize = 0x40020000 as *mut usize;
pub const I2C0_MASTER_MCS_R: *mut usize = 0x40020004 as *mut usize;
pub const I2C0_MASTER_MDR_R: *mut usize = 0x40020008 as *mut usize;
pub const I2C0_MASTER_MTPR_R: *mut usize = 0x4002000C as *mut usize;
pub const I2C0_MASTER_MIMR_R: *mut usize = 0x40020010 as *mut usize;
pub const I2C0_MASTER_MRIS_R: *mut usize = 0x40020014 as *mut usize;
pub const I2C0_MASTER_MMIS_R: *mut usize = 0x40020018 as *mut usize;
pub const I2C0_MASTER_MICR_R: *mut usize = 0x4002001C as *mut usize;
pub const I2C0_MASTER_MCR_R: *mut usize = 0x40020020 as *mut usize;
pub const I2C0_MASTER_MCLKOCNT_R: *mut usize = 0x40020024 as *mut usize;
pub const I2C0_MASTER_MBMON_R: *mut usize = 0x4002002C as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C0 SLAVE)
//
// *****************************************************************************
pub const I2C0_SLAVE_SOAR_R: *mut usize = 0x40020800 as *mut usize;
pub const I2C0_SLAVE_SCSR_R: *mut usize = 0x40020804 as *mut usize;
pub const I2C0_SLAVE_SDR_R: *mut usize = 0x40020808 as *mut usize;
pub const I2C0_SLAVE_SIMR_R: *mut usize = 0x4002080C as *mut usize;
pub const I2C0_SLAVE_SRIS_R: *mut usize = 0x40020810 as *mut usize;
pub const I2C0_SLAVE_SMIS_R: *mut usize = 0x40020814 as *mut usize;
pub const I2C0_SLAVE_SICR_R: *mut usize = 0x40020818 as *mut usize;
pub const I2C0_SLAVE_SOAR2_R: *mut usize = 0x4002081C as *mut usize;
pub const I2C0_SLAVE_SACKCTL_R: *mut usize = 0x40020820 as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C1 MASTER)
//
// *****************************************************************************
pub const I2C1_MASTER_MSA_R: *mut usize = 0x40021000 as *mut usize;
pub const I2C1_MASTER_MCS_R: *mut usize = 0x40021004 as *mut usize;
pub const I2C1_MASTER_MDR_R: *mut usize = 0x40021008 as *mut usize;
pub const I2C1_MASTER_MTPR_R: *mut usize = 0x4002100C as *mut usize;
pub const I2C1_MASTER_MIMR_R: *mut usize = 0x40021010 as *mut usize;
pub const I2C1_MASTER_MRIS_R: *mut usize = 0x40021014 as *mut usize;
pub const I2C1_MASTER_MMIS_R: *mut usize = 0x40021018 as *mut usize;
pub const I2C1_MASTER_MICR_R: *mut usize = 0x4002101C as *mut usize;
pub const I2C1_MASTER_MCR_R: *mut usize = 0x40021020 as *mut usize;
pub const I2C1_MASTER_MCLKOCNT_R: *mut usize = 0x40021024 as *mut usize;
pub const I2C1_MASTER_MBMON_R: *mut usize = 0x4002102C as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C1 SLAVE)
//
// *****************************************************************************
pub const I2C1_SLAVE_SOAR_R: *mut usize = 0x40021800 as *mut usize;
pub const I2C1_SLAVE_SCSR_R: *mut usize = 0x40021804 as *mut usize;
pub const I2C1_SLAVE_SDR_R: *mut usize = 0x40021808 as *mut usize;
pub const I2C1_SLAVE_SIMR_R: *mut usize = 0x4002180C as *mut usize;
pub const I2C1_SLAVE_SRIS_R: *mut usize = 0x40021810 as *mut usize;
pub const I2C1_SLAVE_SMIS_R: *mut usize = 0x40021814 as *mut usize;
pub const I2C1_SLAVE_SICR_R: *mut usize = 0x40021818 as *mut usize;
pub const I2C1_SLAVE_SOAR2_R: *mut usize = 0x4002181C as *mut usize;
pub const I2C1_SLAVE_SACKCTL_R: *mut usize = 0x40021820 as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C2 MASTER)
//
// *****************************************************************************
pub const I2C2_MASTER_MSA_R: *mut usize = 0x40022000 as *mut usize;
pub const I2C2_MASTER_MCS_R: *mut usize = 0x40022004 as *mut usize;
pub const I2C2_MASTER_MDR_R: *mut usize = 0x40022008 as *mut usize;
pub const I2C2_MASTER_MTPR_R: *mut usize = 0x4002200C as *mut usize;
pub const I2C2_MASTER_MIMR_R: *mut usize = 0x40022010 as *mut usize;
pub const I2C2_MASTER_MRIS_R: *mut usize = 0x40022014 as *mut usize;
pub const I2C2_MASTER_MMIS_R: *mut usize = 0x40022018 as *mut usize;
pub const I2C2_MASTER_MICR_R: *mut usize = 0x4002201C as *mut usize;
pub const I2C2_MASTER_MCR_R: *mut usize = 0x40022020 as *mut usize;
pub const I2C2_MASTER_MCLKOCNT_R: *mut usize = 0x40022024 as *mut usize;
pub const I2C2_MASTER_MBMON_R: *mut usize = 0x4002202C as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C2 SLAVE)
//
// *****************************************************************************
pub const I2C2_SLAVE_SOAR_R: *mut usize = 0x40022800 as *mut usize;
pub const I2C2_SLAVE_SCSR_R: *mut usize = 0x40022804 as *mut usize;
pub const I2C2_SLAVE_SDR_R: *mut usize = 0x40022808 as *mut usize;
pub const I2C2_SLAVE_SIMR_R: *mut usize = 0x4002280C as *mut usize;
pub const I2C2_SLAVE_SRIS_R: *mut usize = 0x40022810 as *mut usize;
pub const I2C2_SLAVE_SMIS_R: *mut usize = 0x40022814 as *mut usize;
pub const I2C2_SLAVE_SICR_R: *mut usize = 0x40022818 as *mut usize;
pub const I2C2_SLAVE_SOAR2_R: *mut usize = 0x4002281C as *mut usize;
pub const I2C2_SLAVE_SACKCTL_R: *mut usize = 0x40022820 as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C3 MASTER)
//
// *****************************************************************************
pub const I2C3_MASTER_MSA_R: *mut usize = 0x40023000 as *mut usize;
pub const I2C3_MASTER_MCS_R: *mut usize = 0x40023004 as *mut usize;
pub const I2C3_MASTER_MDR_R: *mut usize = 0x40023008 as *mut usize;
pub const I2C3_MASTER_MTPR_R: *mut usize = 0x4002300C as *mut usize;
pub const I2C3_MASTER_MIMR_R: *mut usize = 0x40023010 as *mut usize;
pub const I2C3_MASTER_MRIS_R: *mut usize = 0x40023014 as *mut usize;
pub const I2C3_MASTER_MMIS_R: *mut usize = 0x40023018 as *mut usize;
pub const I2C3_MASTER_MICR_R: *mut usize = 0x4002301C as *mut usize;
pub const I2C3_MASTER_MCR_R: *mut usize = 0x40023020 as *mut usize;
pub const I2C3_MASTER_MCLKOCNT_R: *mut usize = 0x40023024 as *mut usize;
pub const I2C3_MASTER_MBMON_R: *mut usize = 0x4002302C as *mut usize;

// *****************************************************************************
//
// I2C registers (I2C3 SLAVE)
//
// *****************************************************************************
pub const I2C3_SLAVE_SOAR_R: *mut usize = 0x40023800 as *mut usize;
pub const I2C3_SLAVE_SCSR_R: *mut usize = 0x40023804 as *mut usize;
pub const I2C3_SLAVE_SDR_R: *mut usize = 0x40023808 as *mut usize;
pub const I2C3_SLAVE_SIMR_R: *mut usize = 0x4002380C as *mut usize;
pub const I2C3_SLAVE_SRIS_R: *mut usize = 0x40023810 as *mut usize;
pub const I2C3_SLAVE_SMIS_R: *mut usize = 0x40023814 as *mut usize;
pub const I2C3_SLAVE_SICR_R: *mut usize = 0x40023818 as *mut usize;
pub const I2C3_SLAVE_SOAR2_R: *mut usize = 0x4002381C as *mut usize;
pub const I2C3_SLAVE_SACKCTL_R: *mut usize = 0x40023820 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTE)
//
// *****************************************************************************
pub const GPIO_PORTE_DATA_BITS_R: *mut usize = 0x40024000 as *mut usize;
pub const GPIO_PORTE_DATA_R: *mut usize = 0x400243FC as *mut usize;
pub const GPIO_PORTE_DIR_R: *mut usize = 0x40024400 as *mut usize;
pub const GPIO_PORTE_IS_R: *mut usize = 0x40024404 as *mut usize;
pub const GPIO_PORTE_IBE_R: *mut usize = 0x40024408 as *mut usize;
pub const GPIO_PORTE_IEV_R: *mut usize = 0x4002440C as *mut usize;
pub const GPIO_PORTE_IM_R: *mut usize = 0x40024410 as *mut usize;
pub const GPIO_PORTE_RIS_R: *mut usize = 0x40024414 as *mut usize;
pub const GPIO_PORTE_MIS_R: *mut usize = 0x40024418 as *mut usize;
pub const GPIO_PORTE_ICR_R: *mut usize = 0x4002441C as *mut usize;
pub const GPIO_PORTE_AFSEL_R: *mut usize = 0x40024420 as *mut usize;
pub const GPIO_PORTE_DR2R_R: *mut usize = 0x40024500 as *mut usize;
pub const GPIO_PORTE_DR4R_R: *mut usize = 0x40024504 as *mut usize;
pub const GPIO_PORTE_DR8R_R: *mut usize = 0x40024508 as *mut usize;
pub const GPIO_PORTE_ODR_R: *mut usize = 0x4002450C as *mut usize;
pub const GPIO_PORTE_PUR_R: *mut usize = 0x40024510 as *mut usize;
pub const GPIO_PORTE_PDR_R: *mut usize = 0x40024514 as *mut usize;
pub const GPIO_PORTE_SLR_R: *mut usize = 0x40024518 as *mut usize;
pub const GPIO_PORTE_DEN_R: *mut usize = 0x4002451C as *mut usize;
pub const GPIO_PORTE_LOCK_R: *mut usize = 0x40024520 as *mut usize;
pub const GPIO_PORTE_CR_R: *mut usize = 0x40024524 as *mut usize;
pub const GPIO_PORTE_AMSEL_R: *mut usize = 0x40024528 as *mut usize;
pub const GPIO_PORTE_PCTL_R: *mut usize = 0x4002452C as *mut usize;
pub const GPIO_PORTE_ADCCTL_R: *mut usize = 0x40024530 as *mut usize;
pub const GPIO_PORTE_DMACTL_R: *mut usize = 0x40024534 as *mut usize;
pub const GPIO_PORTE_SI_R: *mut usize = 0x40024538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTF)
//
// *****************************************************************************
pub const GPIO_PORTF_DATA_BITS_R: *mut usize = 0x40025000 as *mut usize;
pub const GPIO_PORTF_DATA_R: *mut usize = 0x400253FC as *mut usize;
pub const GPIO_PORTF_DIR_R: *mut usize = 0x40025400 as *mut usize;
pub const GPIO_PORTF_IS_R: *mut usize = 0x40025404 as *mut usize;
pub const GPIO_PORTF_IBE_R: *mut usize = 0x40025408 as *mut usize;
pub const GPIO_PORTF_IEV_R: *mut usize = 0x4002540C as *mut usize;
pub const GPIO_PORTF_IM_R: *mut usize = 0x40025410 as *mut usize;
pub const GPIO_PORTF_RIS_R: *mut usize = 0x40025414 as *mut usize;
pub const GPIO_PORTF_MIS_R: *mut usize = 0x40025418 as *mut usize;
pub const GPIO_PORTF_ICR_R: *mut usize = 0x4002541C as *mut usize;
pub const GPIO_PORTF_AFSEL_R: *mut usize = 0x40025420 as *mut usize;
pub const GPIO_PORTF_DR2R_R: *mut usize = 0x40025500 as *mut usize;
pub const GPIO_PORTF_DR4R_R: *mut usize = 0x40025504 as *mut usize;
pub const GPIO_PORTF_DR8R_R: *mut usize = 0x40025508 as *mut usize;
pub const GPIO_PORTF_ODR_R: *mut usize = 0x4002550C as *mut usize;
pub const GPIO_PORTF_PUR_R: *mut usize = 0x40025510 as *mut usize;
pub const GPIO_PORTF_PDR_R: *mut usize = 0x40025514 as *mut usize;
pub const GPIO_PORTF_SLR_R: *mut usize = 0x40025518 as *mut usize;
pub const GPIO_PORTF_DEN_R: *mut usize = 0x4002551C as *mut usize;
pub const GPIO_PORTF_LOCK_R: *mut usize = 0x40025520 as *mut usize;
pub const GPIO_PORTF_CR_R: *mut usize = 0x40025524 as *mut usize;
pub const GPIO_PORTF_AMSEL_R: *mut usize = 0x40025528 as *mut usize;
pub const GPIO_PORTF_PCTL_R: *mut usize = 0x4002552C as *mut usize;
pub const GPIO_PORTF_ADCCTL_R: *mut usize = 0x40025530 as *mut usize;
pub const GPIO_PORTF_DMACTL_R: *mut usize = 0x40025534 as *mut usize;
pub const GPIO_PORTF_SI_R: *mut usize = 0x40025538 as *mut usize;

// *****************************************************************************
//
// Timer registers (TIMER0)
//
// *****************************************************************************
pub const TIMER0_CFG_R: *mut usize = 0x40030000 as *mut usize;
pub const TIMER0_TAMR_R: *mut usize = 0x40030004 as *mut usize;
pub const TIMER0_TBMR_R: *mut usize = 0x40030008 as *mut usize;
pub const TIMER0_CTL_R: *mut usize = 0x4003000C as *mut usize;
pub const TIMER0_SYNC_R: *mut usize = 0x40030010 as *mut usize;
pub const TIMER0_IMR_R: *mut usize = 0x40030018 as *mut usize;
pub const TIMER0_RIS_R: *mut usize = 0x4003001C as *mut usize;
pub const TIMER0_MIS_R: *mut usize = 0x40030020 as *mut usize;
pub const TIMER0_ICR_R: *mut usize = 0x40030024 as *mut usize;
pub const TIMER0_TAILR_R: *mut usize = 0x40030028 as *mut usize;
pub const TIMER0_TBILR_R: *mut usize = 0x4003002C as *mut usize;
pub const TIMER0_TAMATCHR_R: *mut usize = 0x40030030 as *mut usize;
pub const TIMER0_TBMATCHR_R: *mut usize = 0x40030034 as *mut usize;
pub const TIMER0_TAPR_R: *mut usize = 0x40030038 as *mut usize;
pub const TIMER0_TBPR_R: *mut usize = 0x4003003C as *mut usize;
pub const TIMER0_TAPMR_R: *mut usize = 0x40030040 as *mut usize;
pub const TIMER0_TBPMR_R: *mut usize = 0x40030044 as *mut usize;
pub const TIMER0_TAR_R: *mut usize = 0x40030048 as *mut usize;
pub const TIMER0_TBR_R: *mut usize = 0x4003004C as *mut usize;
pub const TIMER0_TAV_R: *mut usize = 0x40030050 as *mut usize;
pub const TIMER0_TBV_R: *mut usize = 0x40030054 as *mut usize;
pub const TIMER0_RTCPD_R: *mut usize = 0x40030058 as *mut usize;
pub const TIMER0_TAPS_R: *mut usize = 0x4003005C as *mut usize;
pub const TIMER0_TBPS_R: *mut usize = 0x40030060 as *mut usize;
pub const TIMER0_TAPV_R: *mut usize = 0x40030064 as *mut usize;
pub const TIMER0_TBPV_R: *mut usize = 0x40030068 as *mut usize;
pub const TIMER0_PP_R: *mut usize = 0x40030FC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (TIMER1)
//
// *****************************************************************************
pub const TIMER1_CFG_R: *mut usize = 0x40031000 as *mut usize;
pub const TIMER1_TAMR_R: *mut usize = 0x40031004 as *mut usize;
pub const TIMER1_TBMR_R: *mut usize = 0x40031008 as *mut usize;
pub const TIMER1_CTL_R: *mut usize = 0x4003100C as *mut usize;
pub const TIMER1_SYNC_R: *mut usize = 0x40031010 as *mut usize;
pub const TIMER1_IMR_R: *mut usize = 0x40031018 as *mut usize;
pub const TIMER1_RIS_R: *mut usize = 0x4003101C as *mut usize;
pub const TIMER1_MIS_R: *mut usize = 0x40031020 as *mut usize;
pub const TIMER1_ICR_R: *mut usize = 0x40031024 as *mut usize;
pub const TIMER1_TAILR_R: *mut usize = 0x40031028 as *mut usize;
pub const TIMER1_TBILR_R: *mut usize = 0x4003102C as *mut usize;
pub const TIMER1_TAMATCHR_R: *mut usize = 0x40031030 as *mut usize;
pub const TIMER1_TBMATCHR_R: *mut usize = 0x40031034 as *mut usize;
pub const TIMER1_TAPR_R: *mut usize = 0x40031038 as *mut usize;
pub const TIMER1_TBPR_R: *mut usize = 0x4003103C as *mut usize;
pub const TIMER1_TAPMR_R: *mut usize = 0x40031040 as *mut usize;
pub const TIMER1_TBPMR_R: *mut usize = 0x40031044 as *mut usize;
pub const TIMER1_TAR_R: *mut usize = 0x40031048 as *mut usize;
pub const TIMER1_TBR_R: *mut usize = 0x4003104C as *mut usize;
pub const TIMER1_TAV_R: *mut usize = 0x40031050 as *mut usize;
pub const TIMER1_TBV_R: *mut usize = 0x40031054 as *mut usize;
pub const TIMER1_RTCPD_R: *mut usize = 0x40031058 as *mut usize;
pub const TIMER1_TAPS_R: *mut usize = 0x4003105C as *mut usize;
pub const TIMER1_TBPS_R: *mut usize = 0x40031060 as *mut usize;
pub const TIMER1_TAPV_R: *mut usize = 0x40031064 as *mut usize;
pub const TIMER1_TBPV_R: *mut usize = 0x40031068 as *mut usize;
pub const TIMER1_PP_R: *mut usize = 0x40031FC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (TIMER2)
//
// *****************************************************************************
pub const TIMER2_CFG_R: *mut usize = 0x40032000 as *mut usize;
pub const TIMER2_TAMR_R: *mut usize = 0x40032004 as *mut usize;
pub const TIMER2_TBMR_R: *mut usize = 0x40032008 as *mut usize;
pub const TIMER2_CTL_R: *mut usize = 0x4003200C as *mut usize;
pub const TIMER2_SYNC_R: *mut usize = 0x40032010 as *mut usize;
pub const TIMER2_IMR_R: *mut usize = 0x40032018 as *mut usize;
pub const TIMER2_RIS_R: *mut usize = 0x4003201C as *mut usize;
pub const TIMER2_MIS_R: *mut usize = 0x40032020 as *mut usize;
pub const TIMER2_ICR_R: *mut usize = 0x40032024 as *mut usize;
pub const TIMER2_TAILR_R: *mut usize = 0x40032028 as *mut usize;
pub const TIMER2_TBILR_R: *mut usize = 0x4003202C as *mut usize;
pub const TIMER2_TAMATCHR_R: *mut usize = 0x40032030 as *mut usize;
pub const TIMER2_TBMATCHR_R: *mut usize = 0x40032034 as *mut usize;
pub const TIMER2_TAPR_R: *mut usize = 0x40032038 as *mut usize;
pub const TIMER2_TBPR_R: *mut usize = 0x4003203C as *mut usize;
pub const TIMER2_TAPMR_R: *mut usize = 0x40032040 as *mut usize;
pub const TIMER2_TBPMR_R: *mut usize = 0x40032044 as *mut usize;
pub const TIMER2_TAR_R: *mut usize = 0x40032048 as *mut usize;
pub const TIMER2_TBR_R: *mut usize = 0x4003204C as *mut usize;
pub const TIMER2_TAV_R: *mut usize = 0x40032050 as *mut usize;
pub const TIMER2_TBV_R: *mut usize = 0x40032054 as *mut usize;
pub const TIMER2_RTCPD_R: *mut usize = 0x40032058 as *mut usize;
pub const TIMER2_TAPS_R: *mut usize = 0x4003205C as *mut usize;
pub const TIMER2_TBPS_R: *mut usize = 0x40032060 as *mut usize;
pub const TIMER2_TAPV_R: *mut usize = 0x40032064 as *mut usize;
pub const TIMER2_TBPV_R: *mut usize = 0x40032068 as *mut usize;
pub const TIMER2_PP_R: *mut usize = 0x40032FC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (TIMER3)
//
// *****************************************************************************
pub const TIMER3_CFG_R: *mut usize = 0x40033000 as *mut usize;
pub const TIMER3_TAMR_R: *mut usize = 0x40033004 as *mut usize;
pub const TIMER3_TBMR_R: *mut usize = 0x40033008 as *mut usize;
pub const TIMER3_CTL_R: *mut usize = 0x4003300C as *mut usize;
pub const TIMER3_SYNC_R: *mut usize = 0x40033010 as *mut usize;
pub const TIMER3_IMR_R: *mut usize = 0x40033018 as *mut usize;
pub const TIMER3_RIS_R: *mut usize = 0x4003301C as *mut usize;
pub const TIMER3_MIS_R: *mut usize = 0x40033020 as *mut usize;
pub const TIMER3_ICR_R: *mut usize = 0x40033024 as *mut usize;
pub const TIMER3_TAILR_R: *mut usize = 0x40033028 as *mut usize;
pub const TIMER3_TBILR_R: *mut usize = 0x4003302C as *mut usize;
pub const TIMER3_TAMATCHR_R: *mut usize = 0x40033030 as *mut usize;
pub const TIMER3_TBMATCHR_R: *mut usize = 0x40033034 as *mut usize;
pub const TIMER3_TAPR_R: *mut usize = 0x40033038 as *mut usize;
pub const TIMER3_TBPR_R: *mut usize = 0x4003303C as *mut usize;
pub const TIMER3_TAPMR_R: *mut usize = 0x40033040 as *mut usize;
pub const TIMER3_TBPMR_R: *mut usize = 0x40033044 as *mut usize;
pub const TIMER3_TAR_R: *mut usize = 0x40033048 as *mut usize;
pub const TIMER3_TBR_R: *mut usize = 0x4003304C as *mut usize;
pub const TIMER3_TAV_R: *mut usize = 0x40033050 as *mut usize;
pub const TIMER3_TBV_R: *mut usize = 0x40033054 as *mut usize;
pub const TIMER3_RTCPD_R: *mut usize = 0x40033058 as *mut usize;
pub const TIMER3_TAPS_R: *mut usize = 0x4003305C as *mut usize;
pub const TIMER3_TBPS_R: *mut usize = 0x40033060 as *mut usize;
pub const TIMER3_TAPV_R: *mut usize = 0x40033064 as *mut usize;
pub const TIMER3_TBPV_R: *mut usize = 0x40033068 as *mut usize;
pub const TIMER3_PP_R: *mut usize = 0x40033FC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (TIMER4)
//
// *****************************************************************************
pub const TIMER4_CFG_R: *mut usize = 0x40034000 as *mut usize;
pub const TIMER4_TAMR_R: *mut usize = 0x40034004 as *mut usize;
pub const TIMER4_TBMR_R: *mut usize = 0x40034008 as *mut usize;
pub const TIMER4_CTL_R: *mut usize = 0x4003400C as *mut usize;
pub const TIMER4_SYNC_R: *mut usize = 0x40034010 as *mut usize;
pub const TIMER4_IMR_R: *mut usize = 0x40034018 as *mut usize;
pub const TIMER4_RIS_R: *mut usize = 0x4003401C as *mut usize;
pub const TIMER4_MIS_R: *mut usize = 0x40034020 as *mut usize;
pub const TIMER4_ICR_R: *mut usize = 0x40034024 as *mut usize;
pub const TIMER4_TAILR_R: *mut usize = 0x40034028 as *mut usize;
pub const TIMER4_TBILR_R: *mut usize = 0x4003402C as *mut usize;
pub const TIMER4_TAMATCHR_R: *mut usize = 0x40034030 as *mut usize;
pub const TIMER4_TBMATCHR_R: *mut usize = 0x40034034 as *mut usize;
pub const TIMER4_TAPR_R: *mut usize = 0x40034038 as *mut usize;
pub const TIMER4_TBPR_R: *mut usize = 0x4003403C as *mut usize;
pub const TIMER4_TAPMR_R: *mut usize = 0x40034040 as *mut usize;
pub const TIMER4_TBPMR_R: *mut usize = 0x40034044 as *mut usize;
pub const TIMER4_TAR_R: *mut usize = 0x40034048 as *mut usize;
pub const TIMER4_TBR_R: *mut usize = 0x4003404C as *mut usize;
pub const TIMER4_TAV_R: *mut usize = 0x40034050 as *mut usize;
pub const TIMER4_TBV_R: *mut usize = 0x40034054 as *mut usize;
pub const TIMER4_RTCPD_R: *mut usize = 0x40034058 as *mut usize;
pub const TIMER4_TAPS_R: *mut usize = 0x4003405C as *mut usize;
pub const TIMER4_TBPS_R: *mut usize = 0x40034060 as *mut usize;
pub const TIMER4_TAPV_R: *mut usize = 0x40034064 as *mut usize;
pub const TIMER4_TBPV_R: *mut usize = 0x40034068 as *mut usize;
pub const TIMER4_PP_R: *mut usize = 0x40034FC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (TIMER5)
//
// *****************************************************************************
pub const TIMER5_CFG_R: *mut usize = 0x40035000 as *mut usize;
pub const TIMER5_TAMR_R: *mut usize = 0x40035004 as *mut usize;
pub const TIMER5_TBMR_R: *mut usize = 0x40035008 as *mut usize;
pub const TIMER5_CTL_R: *mut usize = 0x4003500C as *mut usize;
pub const TIMER5_SYNC_R: *mut usize = 0x40035010 as *mut usize;
pub const TIMER5_IMR_R: *mut usize = 0x40035018 as *mut usize;
pub const TIMER5_RIS_R: *mut usize = 0x4003501C as *mut usize;
pub const TIMER5_MIS_R: *mut usize = 0x40035020 as *mut usize;
pub const TIMER5_ICR_R: *mut usize = 0x40035024 as *mut usize;
pub const TIMER5_TAILR_R: *mut usize = 0x40035028 as *mut usize;
pub const TIMER5_TBILR_R: *mut usize = 0x4003502C as *mut usize;
pub const TIMER5_TAMATCHR_R: *mut usize = 0x40035030 as *mut usize;
pub const TIMER5_TBMATCHR_R: *mut usize = 0x40035034 as *mut usize;
pub const TIMER5_TAPR_R: *mut usize = 0x40035038 as *mut usize;
pub const TIMER5_TBPR_R: *mut usize = 0x4003503C as *mut usize;
pub const TIMER5_TAPMR_R: *mut usize = 0x40035040 as *mut usize;
pub const TIMER5_TBPMR_R: *mut usize = 0x40035044 as *mut usize;
pub const TIMER5_TAR_R: *mut usize = 0x40035048 as *mut usize;
pub const TIMER5_TBR_R: *mut usize = 0x4003504C as *mut usize;
pub const TIMER5_TAV_R: *mut usize = 0x40035050 as *mut usize;
pub const TIMER5_TBV_R: *mut usize = 0x40035054 as *mut usize;
pub const TIMER5_RTCPD_R: *mut usize = 0x40035058 as *mut usize;
pub const TIMER5_TAPS_R: *mut usize = 0x4003505C as *mut usize;
pub const TIMER5_TBPS_R: *mut usize = 0x40035060 as *mut usize;
pub const TIMER5_TAPV_R: *mut usize = 0x40035064 as *mut usize;
pub const TIMER5_TBPV_R: *mut usize = 0x40035068 as *mut usize;
pub const TIMER5_PP_R: *mut usize = 0x40035FC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (WTIMER0)
//
// *****************************************************************************
pub const WTIMER0_CFG_R: *mut usize = 0x40036000 as *mut usize;
pub const WTIMER0_TAMR_R: *mut usize = 0x40036004 as *mut usize;
pub const WTIMER0_TBMR_R: *mut usize = 0x40036008 as *mut usize;
pub const WTIMER0_CTL_R: *mut usize = 0x4003600C as *mut usize;
pub const WTIMER0_SYNC_R: *mut usize = 0x40036010 as *mut usize;
pub const WTIMER0_IMR_R: *mut usize = 0x40036018 as *mut usize;
pub const WTIMER0_RIS_R: *mut usize = 0x4003601C as *mut usize;
pub const WTIMER0_MIS_R: *mut usize = 0x40036020 as *mut usize;
pub const WTIMER0_ICR_R: *mut usize = 0x40036024 as *mut usize;
pub const WTIMER0_TAILR_R: *mut usize = 0x40036028 as *mut usize;
pub const WTIMER0_TBILR_R: *mut usize = 0x4003602C as *mut usize;
pub const WTIMER0_TAMATCHR_R: *mut usize = 0x40036030 as *mut usize;
pub const WTIMER0_TBMATCHR_R: *mut usize = 0x40036034 as *mut usize;
pub const WTIMER0_TAPR_R: *mut usize = 0x40036038 as *mut usize;
pub const WTIMER0_TBPR_R: *mut usize = 0x4003603C as *mut usize;
pub const WTIMER0_TAPMR_R: *mut usize = 0x40036040 as *mut usize;
pub const WTIMER0_TBPMR_R: *mut usize = 0x40036044 as *mut usize;
pub const WTIMER0_TAR_R: *mut usize = 0x40036048 as *mut usize;
pub const WTIMER0_TBR_R: *mut usize = 0x4003604C as *mut usize;
pub const WTIMER0_TAV_R: *mut usize = 0x40036050 as *mut usize;
pub const WTIMER0_TBV_R: *mut usize = 0x40036054 as *mut usize;
pub const WTIMER0_RTCPD_R: *mut usize = 0x40036058 as *mut usize;
pub const WTIMER0_TAPS_R: *mut usize = 0x4003605C as *mut usize;
pub const WTIMER0_TBPS_R: *mut usize = 0x40036060 as *mut usize;
pub const WTIMER0_TAPV_R: *mut usize = 0x40036064 as *mut usize;
pub const WTIMER0_TBPV_R: *mut usize = 0x40036068 as *mut usize;
pub const WTIMER0_PP_R: *mut usize = 0x40036FC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (WTIMER1)
//
// *****************************************************************************
pub const WTIMER1_CFG_R: *mut usize = 0x40037000 as *mut usize;
pub const WTIMER1_TAMR_R: *mut usize = 0x40037004 as *mut usize;
pub const WTIMER1_TBMR_R: *mut usize = 0x40037008 as *mut usize;
pub const WTIMER1_CTL_R: *mut usize = 0x4003700C as *mut usize;
pub const WTIMER1_SYNC_R: *mut usize = 0x40037010 as *mut usize;
pub const WTIMER1_IMR_R: *mut usize = 0x40037018 as *mut usize;
pub const WTIMER1_RIS_R: *mut usize = 0x4003701C as *mut usize;
pub const WTIMER1_MIS_R: *mut usize = 0x40037020 as *mut usize;
pub const WTIMER1_ICR_R: *mut usize = 0x40037024 as *mut usize;
pub const WTIMER1_TAILR_R: *mut usize = 0x40037028 as *mut usize;
pub const WTIMER1_TBILR_R: *mut usize = 0x4003702C as *mut usize;
pub const WTIMER1_TAMATCHR_R: *mut usize = 0x40037030 as *mut usize;
pub const WTIMER1_TBMATCHR_R: *mut usize = 0x40037034 as *mut usize;
pub const WTIMER1_TAPR_R: *mut usize = 0x40037038 as *mut usize;
pub const WTIMER1_TBPR_R: *mut usize = 0x4003703C as *mut usize;
pub const WTIMER1_TAPMR_R: *mut usize = 0x40037040 as *mut usize;
pub const WTIMER1_TBPMR_R: *mut usize = 0x40037044 as *mut usize;
pub const WTIMER1_TAR_R: *mut usize = 0x40037048 as *mut usize;
pub const WTIMER1_TBR_R: *mut usize = 0x4003704C as *mut usize;
pub const WTIMER1_TAV_R: *mut usize = 0x40037050 as *mut usize;
pub const WTIMER1_TBV_R: *mut usize = 0x40037054 as *mut usize;
pub const WTIMER1_RTCPD_R: *mut usize = 0x40037058 as *mut usize;
pub const WTIMER1_TAPS_R: *mut usize = 0x4003705C as *mut usize;
pub const WTIMER1_TBPS_R: *mut usize = 0x40037060 as *mut usize;
pub const WTIMER1_TAPV_R: *mut usize = 0x40037064 as *mut usize;
pub const WTIMER1_TBPV_R: *mut usize = 0x40037068 as *mut usize;
pub const WTIMER1_PP_R: *mut usize = 0x40037FC0 as *mut usize;

// *****************************************************************************
//
// ADC registers (ADC0)
//
// *****************************************************************************
pub const ADC0_ACTSS_R: *mut usize = 0x40038000 as *mut usize;
pub const ADC0_RIS_R: *mut usize = 0x40038004 as *mut usize;
pub const ADC0_IM_R: *mut usize = 0x40038008 as *mut usize;
pub const ADC0_ISC_R: *mut usize = 0x4003800C as *mut usize;
pub const ADC0_OSTAT_R: *mut usize = 0x40038010 as *mut usize;
pub const ADC0_EMUX_R: *mut usize = 0x40038014 as *mut usize;
pub const ADC0_USTAT_R: *mut usize = 0x40038018 as *mut usize;
pub const ADC0_SSPRI_R: *mut usize = 0x40038020 as *mut usize;
pub const ADC0_SPC_R: *mut usize = 0x40038024 as *mut usize;
pub const ADC0_PSSI_R: *mut usize = 0x40038028 as *mut usize;
pub const ADC0_SAC_R: *mut usize = 0x40038030 as *mut usize;
pub const ADC0_DCISC_R: *mut usize = 0x40038034 as *mut usize;
pub const ADC0_SSMUX0_R: *mut usize = 0x40038040 as *mut usize;
pub const ADC0_SSCTL0_R: *mut usize = 0x40038044 as *mut usize;
pub const ADC0_SSFIFO0_R: *mut usize = 0x40038048 as *mut usize;
pub const ADC0_SSFSTAT0_R: *mut usize = 0x4003804C as *mut usize;
pub const ADC0_SSOP0_R: *mut usize = 0x40038050 as *mut usize;
pub const ADC0_SSDC0_R: *mut usize = 0x40038054 as *mut usize;
pub const ADC0_SSMUX1_R: *mut usize = 0x40038060 as *mut usize;
pub const ADC0_SSCTL1_R: *mut usize = 0x40038064 as *mut usize;
pub const ADC0_SSFIFO1_R: *mut usize = 0x40038068 as *mut usize;
pub const ADC0_SSFSTAT1_R: *mut usize = 0x4003806C as *mut usize;
pub const ADC0_SSOP1_R: *mut usize = 0x40038070 as *mut usize;
pub const ADC0_SSDC1_R: *mut usize = 0x40038074 as *mut usize;
pub const ADC0_SSMUX2_R: *mut usize = 0x40038080 as *mut usize;
pub const ADC0_SSCTL2_R: *mut usize = 0x40038084 as *mut usize;
pub const ADC0_SSFIFO2_R: *mut usize = 0x40038088 as *mut usize;
pub const ADC0_SSFSTAT2_R: *mut usize = 0x4003808C as *mut usize;
pub const ADC0_SSOP2_R: *mut usize = 0x40038090 as *mut usize;
pub const ADC0_SSDC2_R: *mut usize = 0x40038094 as *mut usize;
pub const ADC0_SSMUX3_R: *mut usize = 0x400380A0 as *mut usize;
pub const ADC0_SSCTL3_R: *mut usize = 0x400380A4 as *mut usize;
pub const ADC0_SSFIFO3_R: *mut usize = 0x400380A8 as *mut usize;
pub const ADC0_SSFSTAT3_R: *mut usize = 0x400380AC as *mut usize;
pub const ADC0_SSOP3_R: *mut usize = 0x400380B0 as *mut usize;
pub const ADC0_SSDC3_R: *mut usize = 0x400380B4 as *mut usize;
pub const ADC0_DCRIC_R: *mut usize = 0x40038D00 as *mut usize;
pub const ADC0_DCCTL0_R: *mut usize = 0x40038E00 as *mut usize;
pub const ADC0_DCCTL1_R: *mut usize = 0x40038E04 as *mut usize;
pub const ADC0_DCCTL2_R: *mut usize = 0x40038E08 as *mut usize;
pub const ADC0_DCCTL3_R: *mut usize = 0x40038E0C as *mut usize;
pub const ADC0_DCCTL4_R: *mut usize = 0x40038E10 as *mut usize;
pub const ADC0_DCCTL5_R: *mut usize = 0x40038E14 as *mut usize;
pub const ADC0_DCCTL6_R: *mut usize = 0x40038E18 as *mut usize;
pub const ADC0_DCCTL7_R: *mut usize = 0x40038E1C as *mut usize;
pub const ADC0_DCCMP0_R: *mut usize = 0x40038E40 as *mut usize;
pub const ADC0_DCCMP1_R: *mut usize = 0x40038E44 as *mut usize;
pub const ADC0_DCCMP2_R: *mut usize = 0x40038E48 as *mut usize;
pub const ADC0_DCCMP3_R: *mut usize = 0x40038E4C as *mut usize;
pub const ADC0_DCCMP4_R: *mut usize = 0x40038E50 as *mut usize;
pub const ADC0_DCCMP5_R: *mut usize = 0x40038E54 as *mut usize;
pub const ADC0_DCCMP6_R: *mut usize = 0x40038E58 as *mut usize;
pub const ADC0_DCCMP7_R: *mut usize = 0x40038E5C as *mut usize;
pub const ADC0_PP_R: *mut usize = 0x40038FC0 as *mut usize;
pub const ADC0_PC_R: *mut usize = 0x40038FC4 as *mut usize;
pub const ADC0_CC_R: *mut usize = 0x40038FC8 as *mut usize;

// *****************************************************************************
//
// ADC registers (ADC1)
//
// *****************************************************************************
pub const ADC1_ACTSS_R: *mut usize = 0x40039000 as *mut usize;
pub const ADC1_RIS_R: *mut usize = 0x40039004 as *mut usize;
pub const ADC1_IM_R: *mut usize = 0x40039008 as *mut usize;
pub const ADC1_ISC_R: *mut usize = 0x4003900C as *mut usize;
pub const ADC1_OSTAT_R: *mut usize = 0x40039010 as *mut usize;
pub const ADC1_EMUX_R: *mut usize = 0x40039014 as *mut usize;
pub const ADC1_USTAT_R: *mut usize = 0x40039018 as *mut usize;
pub const ADC1_SSPRI_R: *mut usize = 0x40039020 as *mut usize;
pub const ADC1_SPC_R: *mut usize = 0x40039024 as *mut usize;
pub const ADC1_PSSI_R: *mut usize = 0x40039028 as *mut usize;
pub const ADC1_SAC_R: *mut usize = 0x40039030 as *mut usize;
pub const ADC1_DCISC_R: *mut usize = 0x40039034 as *mut usize;
pub const ADC1_SSMUX0_R: *mut usize = 0x40039040 as *mut usize;
pub const ADC1_SSCTL0_R: *mut usize = 0x40039044 as *mut usize;
pub const ADC1_SSFIFO0_R: *mut usize = 0x40039048 as *mut usize;
pub const ADC1_SSFSTAT0_R: *mut usize = 0x4003904C as *mut usize;
pub const ADC1_SSOP0_R: *mut usize = 0x40039050 as *mut usize;
pub const ADC1_SSDC0_R: *mut usize = 0x40039054 as *mut usize;
pub const ADC1_SSMUX1_R: *mut usize = 0x40039060 as *mut usize;
pub const ADC1_SSCTL1_R: *mut usize = 0x40039064 as *mut usize;
pub const ADC1_SSFIFO1_R: *mut usize = 0x40039068 as *mut usize;
pub const ADC1_SSFSTAT1_R: *mut usize = 0x4003906C as *mut usize;
pub const ADC1_SSOP1_R: *mut usize = 0x40039070 as *mut usize;
pub const ADC1_SSDC1_R: *mut usize = 0x40039074 as *mut usize;
pub const ADC1_SSMUX2_R: *mut usize = 0x40039080 as *mut usize;
pub const ADC1_SSCTL2_R: *mut usize = 0x40039084 as *mut usize;
pub const ADC1_SSFIFO2_R: *mut usize = 0x40039088 as *mut usize;
pub const ADC1_SSFSTAT2_R: *mut usize = 0x4003908C as *mut usize;
pub const ADC1_SSOP2_R: *mut usize = 0x40039090 as *mut usize;
pub const ADC1_SSDC2_R: *mut usize = 0x40039094 as *mut usize;
pub const ADC1_SSMUX3_R: *mut usize = 0x400390A0 as *mut usize;
pub const ADC1_SSCTL3_R: *mut usize = 0x400390A4 as *mut usize;
pub const ADC1_SSFIFO3_R: *mut usize = 0x400390A8 as *mut usize;
pub const ADC1_SSFSTAT3_R: *mut usize = 0x400390AC as *mut usize;
pub const ADC1_SSOP3_R: *mut usize = 0x400390B0 as *mut usize;
pub const ADC1_SSDC3_R: *mut usize = 0x400390B4 as *mut usize;
pub const ADC1_DCRIC_R: *mut usize = 0x40039D00 as *mut usize;
pub const ADC1_DCCTL0_R: *mut usize = 0x40039E00 as *mut usize;
pub const ADC1_DCCTL1_R: *mut usize = 0x40039E04 as *mut usize;
pub const ADC1_DCCTL2_R: *mut usize = 0x40039E08 as *mut usize;
pub const ADC1_DCCTL3_R: *mut usize = 0x40039E0C as *mut usize;
pub const ADC1_DCCTL4_R: *mut usize = 0x40039E10 as *mut usize;
pub const ADC1_DCCTL5_R: *mut usize = 0x40039E14 as *mut usize;
pub const ADC1_DCCTL6_R: *mut usize = 0x40039E18 as *mut usize;
pub const ADC1_DCCTL7_R: *mut usize = 0x40039E1C as *mut usize;
pub const ADC1_DCCMP0_R: *mut usize = 0x40039E40 as *mut usize;
pub const ADC1_DCCMP1_R: *mut usize = 0x40039E44 as *mut usize;
pub const ADC1_DCCMP2_R: *mut usize = 0x40039E48 as *mut usize;
pub const ADC1_DCCMP3_R: *mut usize = 0x40039E4C as *mut usize;
pub const ADC1_DCCMP4_R: *mut usize = 0x40039E50 as *mut usize;
pub const ADC1_DCCMP5_R: *mut usize = 0x40039E54 as *mut usize;
pub const ADC1_DCCMP6_R: *mut usize = 0x40039E58 as *mut usize;
pub const ADC1_DCCMP7_R: *mut usize = 0x40039E5C as *mut usize;
pub const ADC1_PP_R: *mut usize = 0x40039FC0 as *mut usize;
pub const ADC1_PC_R: *mut usize = 0x40039FC4 as *mut usize;
pub const ADC1_CC_R: *mut usize = 0x40039FC8 as *mut usize;

// *****************************************************************************
//
// Comparator registers (COMP)
//
// *****************************************************************************
pub const COMP_ACMIS_R: *mut usize = 0x4003C000 as *mut usize;
pub const COMP_ACRIS_R: *mut usize = 0x4003C004 as *mut usize;
pub const COMP_ACINTEN_R: *mut usize = 0x4003C008 as *mut usize;
pub const COMP_ACREFCTL_R: *mut usize = 0x4003C010 as *mut usize;
pub const COMP_ACSTAT0_R: *mut usize = 0x4003C020 as *mut usize;
pub const COMP_ACCTL0_R: *mut usize = 0x4003C024 as *mut usize;
pub const COMP_ACSTAT1_R: *mut usize = 0x4003C040 as *mut usize;
pub const COMP_ACCTL1_R: *mut usize = 0x4003C044 as *mut usize;
pub const COMP_PP_R: *mut usize = 0x4003CFC0 as *mut usize;

// *****************************************************************************
//
// CAN registers (CAN0)
//
// *****************************************************************************
pub const CAN0_CTL_R: *mut usize = 0x40040000 as *mut usize;
pub const CAN0_STS_R: *mut usize = 0x40040004 as *mut usize;
pub const CAN0_ERR_R: *mut usize = 0x40040008 as *mut usize;
pub const CAN0_BIT_R: *mut usize = 0x4004000C as *mut usize;
pub const CAN0_INT_R: *mut usize = 0x40040010 as *mut usize;
pub const CAN0_TST_R: *mut usize = 0x40040014 as *mut usize;
pub const CAN0_BRPE_R: *mut usize = 0x40040018 as *mut usize;
pub const CAN0_IF1CRQ_R: *mut usize = 0x40040020 as *mut usize;
pub const CAN0_IF1CMSK_R: *mut usize = 0x40040024 as *mut usize;
pub const CAN0_IF1MSK1_R: *mut usize = 0x40040028 as *mut usize;
pub const CAN0_IF1MSK2_R: *mut usize = 0x4004002C as *mut usize;
pub const CAN0_IF1ARB1_R: *mut usize = 0x40040030 as *mut usize;
pub const CAN0_IF1ARB2_R: *mut usize = 0x40040034 as *mut usize;
pub const CAN0_IF1MCTL_R: *mut usize = 0x40040038 as *mut usize;
pub const CAN0_IF1DA1_R: *mut usize = 0x4004003C as *mut usize;
pub const CAN0_IF1DA2_R: *mut usize = 0x40040040 as *mut usize;
pub const CAN0_IF1DB1_R: *mut usize = 0x40040044 as *mut usize;
pub const CAN0_IF1DB2_R: *mut usize = 0x40040048 as *mut usize;
pub const CAN0_IF2CRQ_R: *mut usize = 0x40040080 as *mut usize;
pub const CAN0_IF2CMSK_R: *mut usize = 0x40040084 as *mut usize;
pub const CAN0_IF2MSK1_R: *mut usize = 0x40040088 as *mut usize;
pub const CAN0_IF2MSK2_R: *mut usize = 0x4004008C as *mut usize;
pub const CAN0_IF2ARB1_R: *mut usize = 0x40040090 as *mut usize;
pub const CAN0_IF2ARB2_R: *mut usize = 0x40040094 as *mut usize;
pub const CAN0_IF2MCTL_R: *mut usize = 0x40040098 as *mut usize;
pub const CAN0_IF2DA1_R: *mut usize = 0x4004009C as *mut usize;
pub const CAN0_IF2DA2_R: *mut usize = 0x400400A0 as *mut usize;
pub const CAN0_IF2DB1_R: *mut usize = 0x400400A4 as *mut usize;
pub const CAN0_IF2DB2_R: *mut usize = 0x400400A8 as *mut usize;
pub const CAN0_TXRQ1_R: *mut usize = 0x40040100 as *mut usize;
pub const CAN0_TXRQ2_R: *mut usize = 0x40040104 as *mut usize;
pub const CAN0_NWDA1_R: *mut usize = 0x40040120 as *mut usize;
pub const CAN0_NWDA2_R: *mut usize = 0x40040124 as *mut usize;
pub const CAN0_MSG1INT_R: *mut usize = 0x40040140 as *mut usize;
pub const CAN0_MSG2INT_R: *mut usize = 0x40040144 as *mut usize;
pub const CAN0_MSG1VAL_R: *mut usize = 0x40040160 as *mut usize;
pub const CAN0_MSG2VAL_R: *mut usize = 0x40040164 as *mut usize;

// *****************************************************************************
//
// Timer registers (WTIMER2)
//
// *****************************************************************************
pub const WTIMER2_CFG_R: *mut usize = 0x4004C000 as *mut usize;
pub const WTIMER2_TAMR_R: *mut usize = 0x4004C004 as *mut usize;
pub const WTIMER2_TBMR_R: *mut usize = 0x4004C008 as *mut usize;
pub const WTIMER2_CTL_R: *mut usize = 0x4004C00C as *mut usize;
pub const WTIMER2_SYNC_R: *mut usize = 0x4004C010 as *mut usize;
pub const WTIMER2_IMR_R: *mut usize = 0x4004C018 as *mut usize;
pub const WTIMER2_RIS_R: *mut usize = 0x4004C01C as *mut usize;
pub const WTIMER2_MIS_R: *mut usize = 0x4004C020 as *mut usize;
pub const WTIMER2_ICR_R: *mut usize = 0x4004C024 as *mut usize;
pub const WTIMER2_TAILR_R: *mut usize = 0x4004C028 as *mut usize;
pub const WTIMER2_TBILR_R: *mut usize = 0x4004C02C as *mut usize;
pub const WTIMER2_TAMATCHR_R: *mut usize = 0x4004C030 as *mut usize;
pub const WTIMER2_TBMATCHR_R: *mut usize = 0x4004C034 as *mut usize;
pub const WTIMER2_TAPR_R: *mut usize = 0x4004C038 as *mut usize;
pub const WTIMER2_TBPR_R: *mut usize = 0x4004C03C as *mut usize;
pub const WTIMER2_TAPMR_R: *mut usize = 0x4004C040 as *mut usize;
pub const WTIMER2_TBPMR_R: *mut usize = 0x4004C044 as *mut usize;
pub const WTIMER2_TAR_R: *mut usize = 0x4004C048 as *mut usize;
pub const WTIMER2_TBR_R: *mut usize = 0x4004C04C as *mut usize;
pub const WTIMER2_TAV_R: *mut usize = 0x4004C050 as *mut usize;
pub const WTIMER2_TBV_R: *mut usize = 0x4004C054 as *mut usize;
pub const WTIMER2_RTCPD_R: *mut usize = 0x4004C058 as *mut usize;
pub const WTIMER2_TAPS_R: *mut usize = 0x4004C05C as *mut usize;
pub const WTIMER2_TBPS_R: *mut usize = 0x4004C060 as *mut usize;
pub const WTIMER2_TAPV_R: *mut usize = 0x4004C064 as *mut usize;
pub const WTIMER2_TBPV_R: *mut usize = 0x4004C068 as *mut usize;
pub const WTIMER2_PP_R: *mut usize = 0x4004CFC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (WTIMER3)
//
// *****************************************************************************
pub const WTIMER3_CFG_R: *mut usize = 0x4004D000 as *mut usize;
pub const WTIMER3_TAMR_R: *mut usize = 0x4004D004 as *mut usize;
pub const WTIMER3_TBMR_R: *mut usize = 0x4004D008 as *mut usize;
pub const WTIMER3_CTL_R: *mut usize = 0x4004D00C as *mut usize;
pub const WTIMER3_SYNC_R: *mut usize = 0x4004D010 as *mut usize;
pub const WTIMER3_IMR_R: *mut usize = 0x4004D018 as *mut usize;
pub const WTIMER3_RIS_R: *mut usize = 0x4004D01C as *mut usize;
pub const WTIMER3_MIS_R: *mut usize = 0x4004D020 as *mut usize;
pub const WTIMER3_ICR_R: *mut usize = 0x4004D024 as *mut usize;
pub const WTIMER3_TAILR_R: *mut usize = 0x4004D028 as *mut usize;
pub const WTIMER3_TBILR_R: *mut usize = 0x4004D02C as *mut usize;
pub const WTIMER3_TAMATCHR_R: *mut usize = 0x4004D030 as *mut usize;
pub const WTIMER3_TBMATCHR_R: *mut usize = 0x4004D034 as *mut usize;
pub const WTIMER3_TAPR_R: *mut usize = 0x4004D038 as *mut usize;
pub const WTIMER3_TBPR_R: *mut usize = 0x4004D03C as *mut usize;
pub const WTIMER3_TAPMR_R: *mut usize = 0x4004D040 as *mut usize;
pub const WTIMER3_TBPMR_R: *mut usize = 0x4004D044 as *mut usize;
pub const WTIMER3_TAR_R: *mut usize = 0x4004D048 as *mut usize;
pub const WTIMER3_TBR_R: *mut usize = 0x4004D04C as *mut usize;
pub const WTIMER3_TAV_R: *mut usize = 0x4004D050 as *mut usize;
pub const WTIMER3_TBV_R: *mut usize = 0x4004D054 as *mut usize;
pub const WTIMER3_RTCPD_R: *mut usize = 0x4004D058 as *mut usize;
pub const WTIMER3_TAPS_R: *mut usize = 0x4004D05C as *mut usize;
pub const WTIMER3_TBPS_R: *mut usize = 0x4004D060 as *mut usize;
pub const WTIMER3_TAPV_R: *mut usize = 0x4004D064 as *mut usize;
pub const WTIMER3_TBPV_R: *mut usize = 0x4004D068 as *mut usize;
pub const WTIMER3_PP_R: *mut usize = 0x4004DFC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (WTIMER4)
//
// *****************************************************************************
pub const WTIMER4_CFG_R: *mut usize = 0x4004E000 as *mut usize;
pub const WTIMER4_TAMR_R: *mut usize = 0x4004E004 as *mut usize;
pub const WTIMER4_TBMR_R: *mut usize = 0x4004E008 as *mut usize;
pub const WTIMER4_CTL_R: *mut usize = 0x4004E00C as *mut usize;
pub const WTIMER4_SYNC_R: *mut usize = 0x4004E010 as *mut usize;
pub const WTIMER4_IMR_R: *mut usize = 0x4004E018 as *mut usize;
pub const WTIMER4_RIS_R: *mut usize = 0x4004E01C as *mut usize;
pub const WTIMER4_MIS_R: *mut usize = 0x4004E020 as *mut usize;
pub const WTIMER4_ICR_R: *mut usize = 0x4004E024 as *mut usize;
pub const WTIMER4_TAILR_R: *mut usize = 0x4004E028 as *mut usize;
pub const WTIMER4_TBILR_R: *mut usize = 0x4004E02C as *mut usize;
pub const WTIMER4_TAMATCHR_R: *mut usize = 0x4004E030 as *mut usize;
pub const WTIMER4_TBMATCHR_R: *mut usize = 0x4004E034 as *mut usize;
pub const WTIMER4_TAPR_R: *mut usize = 0x4004E038 as *mut usize;
pub const WTIMER4_TBPR_R: *mut usize = 0x4004E03C as *mut usize;
pub const WTIMER4_TAPMR_R: *mut usize = 0x4004E040 as *mut usize;
pub const WTIMER4_TBPMR_R: *mut usize = 0x4004E044 as *mut usize;
pub const WTIMER4_TAR_R: *mut usize = 0x4004E048 as *mut usize;
pub const WTIMER4_TBR_R: *mut usize = 0x4004E04C as *mut usize;
pub const WTIMER4_TAV_R: *mut usize = 0x4004E050 as *mut usize;
pub const WTIMER4_TBV_R: *mut usize = 0x4004E054 as *mut usize;
pub const WTIMER4_RTCPD_R: *mut usize = 0x4004E058 as *mut usize;
pub const WTIMER4_TAPS_R: *mut usize = 0x4004E05C as *mut usize;
pub const WTIMER4_TBPS_R: *mut usize = 0x4004E060 as *mut usize;
pub const WTIMER4_TAPV_R: *mut usize = 0x4004E064 as *mut usize;
pub const WTIMER4_TBPV_R: *mut usize = 0x4004E068 as *mut usize;
pub const WTIMER4_PP_R: *mut usize = 0x4004EFC0 as *mut usize;

// *****************************************************************************
//
// Timer registers (WTIMER5)
//
// *****************************************************************************
pub const WTIMER5_CFG_R: *mut usize = 0x4004F000 as *mut usize;
pub const WTIMER5_TAMR_R: *mut usize = 0x4004F004 as *mut usize;
pub const WTIMER5_TBMR_R: *mut usize = 0x4004F008 as *mut usize;
pub const WTIMER5_CTL_R: *mut usize = 0x4004F00C as *mut usize;
pub const WTIMER5_SYNC_R: *mut usize = 0x4004F010 as *mut usize;
pub const WTIMER5_IMR_R: *mut usize = 0x4004F018 as *mut usize;
pub const WTIMER5_RIS_R: *mut usize = 0x4004F01C as *mut usize;
pub const WTIMER5_MIS_R: *mut usize = 0x4004F020 as *mut usize;
pub const WTIMER5_ICR_R: *mut usize = 0x4004F024 as *mut usize;
pub const WTIMER5_TAILR_R: *mut usize = 0x4004F028 as *mut usize;
pub const WTIMER5_TBILR_R: *mut usize = 0x4004F02C as *mut usize;
pub const WTIMER5_TAMATCHR_R: *mut usize = 0x4004F030 as *mut usize;
pub const WTIMER5_TBMATCHR_R: *mut usize = 0x4004F034 as *mut usize;
pub const WTIMER5_TAPR_R: *mut usize = 0x4004F038 as *mut usize;
pub const WTIMER5_TBPR_R: *mut usize = 0x4004F03C as *mut usize;
pub const WTIMER5_TAPMR_R: *mut usize = 0x4004F040 as *mut usize;
pub const WTIMER5_TBPMR_R: *mut usize = 0x4004F044 as *mut usize;
pub const WTIMER5_TAR_R: *mut usize = 0x4004F048 as *mut usize;
pub const WTIMER5_TBR_R: *mut usize = 0x4004F04C as *mut usize;
pub const WTIMER5_TAV_R: *mut usize = 0x4004F050 as *mut usize;
pub const WTIMER5_TBV_R: *mut usize = 0x4004F054 as *mut usize;
pub const WTIMER5_RTCPD_R: *mut usize = 0x4004F058 as *mut usize;
pub const WTIMER5_TAPS_R: *mut usize = 0x4004F05C as *mut usize;
pub const WTIMER5_TBPS_R: *mut usize = 0x4004F060 as *mut usize;
pub const WTIMER5_TAPV_R: *mut usize = 0x4004F064 as *mut usize;
pub const WTIMER5_TBPV_R: *mut usize = 0x4004F068 as *mut usize;
pub const WTIMER5_PP_R: *mut usize = 0x4004FFC0 as *mut usize;

// *****************************************************************************
//
// Univeral Serial Bus registers (USB0)
//
// *****************************************************************************

pub const USB0_FADDR_R: *mut u8 = 0x40050000 as *mut u8;

pub const USB0_POWER_R: *mut u8 = 0x40050001 as *mut u8;

pub const USB0_TXIS_R: *mut u16 = 0x40050002 as *mut u16;

pub const USB0_RXIS_R: *mut u16 = 0x40050004 as *mut u16;

pub const USB0_TXIE_R: *mut u16 = 0x40050006 as *mut u16;

pub const USB0_RXIE_R: *mut u16 = 0x40050008 as *mut u16;

pub const USB0_IS_R: *mut u8 = 0x4005000A as *mut u8;

pub const USB0_IE_R: *mut u8 = 0x4005000B as *mut u8;

pub const USB0_FRAME_R: *mut u16 = 0x4005000C as *mut u16;

pub const USB0_EPIDX_R: *mut u8 = 0x4005000E as *mut u8;

pub const USB0_TEST_R: *mut u8 = 0x4005000F as *mut u8;
pub const USB0_FIFO0_R: *mut usize = 0x40050020 as *mut usize;
pub const USB0_FIFO1_R: *mut usize = 0x40050024 as *mut usize;
pub const USB0_FIFO2_R: *mut usize = 0x40050028 as *mut usize;
pub const USB0_FIFO3_R: *mut usize = 0x4005002C as *mut usize;
pub const USB0_FIFO4_R: *mut usize = 0x40050030 as *mut usize;
pub const USB0_FIFO5_R: *mut usize = 0x40050034 as *mut usize;
pub const USB0_FIFO6_R: *mut usize = 0x40050038 as *mut usize;
pub const USB0_FIFO7_R: *mut usize = 0x4005003C as *mut usize;

pub const USB0_TXFIFOSZ_R: *mut u8 = 0x40050062 as *mut u8;

pub const USB0_RXFIFOSZ_R: *mut u8 = 0x40050063 as *mut u8;

pub const USB0_TXFIFOADD_R: *mut u16 = 0x40050064 as *mut u16;

pub const USB0_RXFIFOADD_R: *mut u16 = 0x40050066 as *mut u16;

pub const USB0_CONTIM_R: *mut u8 = 0x4005007A as *mut u8;

pub const USB0_FSEOF_R: *mut u8 = 0x4005007D as *mut u8;

pub const USB0_LSEOF_R: *mut u8 = 0x4005007E as *mut u8;

pub const USB0_CSRL0_R: *mut u8 = 0x40050102 as *mut u8;

pub const USB0_CSRH0_R: *mut u8 = 0x40050103 as *mut u8;

pub const USB0_COUNT0_R: *mut u8 = 0x40050108 as *mut u8;

pub const USB0_TXMAXP1_R: *mut u16 = 0x40050110 as *mut u16;

pub const USB0_TXCSRL1_R: *mut u8 = 0x40050112 as *mut u8;

pub const USB0_TXCSRH1_R: *mut u8 = 0x40050113 as *mut u8;

pub const USB0_RXMAXP1_R: *mut u16 = 0x40050114 as *mut u16;

pub const USB0_RXCSRL1_R: *mut u8 = 0x40050116 as *mut u8;

pub const USB0_RXCSRH1_R: *mut u8 = 0x40050117 as *mut u8;

pub const USB0_RXCOUNT1_R: *mut u16 = 0x40050118 as *mut u16;

pub const USB0_TXMAXP2_R: *mut u16 = 0x40050120 as *mut u16;

pub const USB0_TXCSRL2_R: *mut u8 = 0x40050122 as *mut u8;

pub const USB0_TXCSRH2_R: *mut u8 = 0x40050123 as *mut u8;

pub const USB0_RXMAXP2_R: *mut u16 = 0x40050124 as *mut u16;

pub const USB0_RXCSRL2_R: *mut u8 = 0x40050126 as *mut u8;

pub const USB0_RXCSRH2_R: *mut u8 = 0x40050127 as *mut u8;

pub const USB0_RXCOUNT2_R: *mut u16 = 0x40050128 as *mut u16;

pub const USB0_TXMAXP3_R: *mut u16 = 0x40050130 as *mut u16;

pub const USB0_TXCSRL3_R: *mut u8 = 0x40050132 as *mut u8;

pub const USB0_TXCSRH3_R: *mut u8 = 0x40050133 as *mut u8;

pub const USB0_RXMAXP3_R: *mut u16 = 0x40050134 as *mut u16;

pub const USB0_RXCSRL3_R: *mut u8 = 0x40050136 as *mut u8;

pub const USB0_RXCSRH3_R: *mut u8 = 0x40050137 as *mut u8;

pub const USB0_RXCOUNT3_R: *mut u16 = 0x40050138 as *mut u16;

pub const USB0_TXMAXP4_R: *mut u16 = 0x40050140 as *mut u16;

pub const USB0_TXCSRL4_R: *mut u8 = 0x40050142 as *mut u8;

pub const USB0_TXCSRH4_R: *mut u8 = 0x40050143 as *mut u8;

pub const USB0_RXMAXP4_R: *mut u16 = 0x40050144 as *mut u16;

pub const USB0_RXCSRL4_R: *mut u8 = 0x40050146 as *mut u8;

pub const USB0_RXCSRH4_R: *mut u8 = 0x40050147 as *mut u8;

pub const USB0_RXCOUNT4_R: *mut u16 = 0x40050148 as *mut u16;

pub const USB0_TXMAXP5_R: *mut u16 = 0x40050150 as *mut u16;

pub const USB0_TXCSRL5_R: *mut u8 = 0x40050152 as *mut u8;

pub const USB0_TXCSRH5_R: *mut u8 = 0x40050153 as *mut u8;

pub const USB0_RXMAXP5_R: *mut u16 = 0x40050154 as *mut u16;

pub const USB0_RXCSRL5_R: *mut u8 = 0x40050156 as *mut u8;

pub const USB0_RXCSRH5_R: *mut u8 = 0x40050157 as *mut u8;

pub const USB0_RXCOUNT5_R: *mut u16 = 0x40050158 as *mut u16;

pub const USB0_TXMAXP6_R: *mut u16 = 0x40050160 as *mut u16;

pub const USB0_TXCSRL6_R: *mut u8 = 0x40050162 as *mut u8;

pub const USB0_TXCSRH6_R: *mut u8 = 0x40050163 as *mut u8;

pub const USB0_RXMAXP6_R: *mut u16 = 0x40050164 as *mut u16;

pub const USB0_RXCSRL6_R: *mut u8 = 0x40050166 as *mut u8;

pub const USB0_RXCSRH6_R: *mut u8 = 0x40050167 as *mut u8;

pub const USB0_RXCOUNT6_R: *mut u16 = 0x40050168 as *mut u16;

pub const USB0_TXMAXP7_R: *mut u16 = 0x40050170 as *mut u16;

pub const USB0_TXCSRL7_R: *mut u8 = 0x40050172 as *mut u8;

pub const USB0_TXCSRH7_R: *mut u8 = 0x40050173 as *mut u8;

pub const USB0_RXMAXP7_R: *mut u16 = 0x40050174 as *mut u16;

pub const USB0_RXCSRL7_R: *mut u8 = 0x40050176 as *mut u8;

pub const USB0_RXCSRH7_R: *mut u8 = 0x40050177 as *mut u8;

pub const USB0_RXCOUNT7_R: *mut u16 = 0x40050178 as *mut u16;

pub const USB0_RXDPKTBUFDIS_R: *mut u16 = 0x40050340 as *mut u16;

pub const USB0_TXDPKTBUFDIS_R: *mut u16 = 0x40050342 as *mut u16;
pub const USB0_DRRIS_R: *mut usize = 0x40050410 as *mut usize;
pub const USB0_DRIM_R: *mut usize = 0x40050414 as *mut usize;
pub const USB0_DRISC_R: *mut usize = 0x40050418 as *mut usize;
pub const USB0_DMASEL_R: *mut usize = 0x40050450 as *mut usize;
pub const USB0_PP_R: *mut usize = 0x40050FC0 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTA AHB)
//
// *****************************************************************************
pub const GPIO_PORTA_AHB_DATA_BITS_R: *mut usize = 0x40058000 as *mut usize;
pub const GPIO_PORTA_AHB_DATA_R: *mut usize = 0x400583FC as *mut usize;
pub const GPIO_PORTA_AHB_DIR_R: *mut usize = 0x40058400 as *mut usize;
pub const GPIO_PORTA_AHB_IS_R: *mut usize = 0x40058404 as *mut usize;
pub const GPIO_PORTA_AHB_IBE_R: *mut usize = 0x40058408 as *mut usize;
pub const GPIO_PORTA_AHB_IEV_R: *mut usize = 0x4005840C as *mut usize;
pub const GPIO_PORTA_AHB_IM_R: *mut usize = 0x40058410 as *mut usize;
pub const GPIO_PORTA_AHB_RIS_R: *mut usize = 0x40058414 as *mut usize;
pub const GPIO_PORTA_AHB_MIS_R: *mut usize = 0x40058418 as *mut usize;
pub const GPIO_PORTA_AHB_ICR_R: *mut usize = 0x4005841C as *mut usize;
pub const GPIO_PORTA_AHB_AFSEL_R: *mut usize = 0x40058420 as *mut usize;
pub const GPIO_PORTA_AHB_DR2R_R: *mut usize = 0x40058500 as *mut usize;
pub const GPIO_PORTA_AHB_DR4R_R: *mut usize = 0x40058504 as *mut usize;
pub const GPIO_PORTA_AHB_DR8R_R: *mut usize = 0x40058508 as *mut usize;
pub const GPIO_PORTA_AHB_ODR_R: *mut usize = 0x4005850C as *mut usize;
pub const GPIO_PORTA_AHB_PUR_R: *mut usize = 0x40058510 as *mut usize;
pub const GPIO_PORTA_AHB_PDR_R: *mut usize = 0x40058514 as *mut usize;
pub const GPIO_PORTA_AHB_SLR_R: *mut usize = 0x40058518 as *mut usize;
pub const GPIO_PORTA_AHB_DEN_R: *mut usize = 0x4005851C as *mut usize;
pub const GPIO_PORTA_AHB_LOCK_R: *mut usize = 0x40058520 as *mut usize;
pub const GPIO_PORTA_AHB_CR_R: *mut usize = 0x40058524 as *mut usize;
pub const GPIO_PORTA_AHB_AMSEL_R: *mut usize = 0x40058528 as *mut usize;
pub const GPIO_PORTA_AHB_PCTL_R: *mut usize = 0x4005852C as *mut usize;
pub const GPIO_PORTA_AHB_ADCCTL_R: *mut usize = 0x40058530 as *mut usize;
pub const GPIO_PORTA_AHB_DMACTL_R: *mut usize = 0x40058534 as *mut usize;
pub const GPIO_PORTA_AHB_SI_R: *mut usize = 0x40058538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTB AHB)
//
// *****************************************************************************
pub const GPIO_PORTB_AHB_DATA_BITS_R: *mut usize = 0x40059000 as *mut usize;
pub const GPIO_PORTB_AHB_DATA_R: *mut usize = 0x400593FC as *mut usize;
pub const GPIO_PORTB_AHB_DIR_R: *mut usize = 0x40059400 as *mut usize;
pub const GPIO_PORTB_AHB_IS_R: *mut usize = 0x40059404 as *mut usize;
pub const GPIO_PORTB_AHB_IBE_R: *mut usize = 0x40059408 as *mut usize;
pub const GPIO_PORTB_AHB_IEV_R: *mut usize = 0x4005940C as *mut usize;
pub const GPIO_PORTB_AHB_IM_R: *mut usize = 0x40059410 as *mut usize;
pub const GPIO_PORTB_AHB_RIS_R: *mut usize = 0x40059414 as *mut usize;
pub const GPIO_PORTB_AHB_MIS_R: *mut usize = 0x40059418 as *mut usize;
pub const GPIO_PORTB_AHB_ICR_R: *mut usize = 0x4005941C as *mut usize;
pub const GPIO_PORTB_AHB_AFSEL_R: *mut usize = 0x40059420 as *mut usize;
pub const GPIO_PORTB_AHB_DR2R_R: *mut usize = 0x40059500 as *mut usize;
pub const GPIO_PORTB_AHB_DR4R_R: *mut usize = 0x40059504 as *mut usize;
pub const GPIO_PORTB_AHB_DR8R_R: *mut usize = 0x40059508 as *mut usize;
pub const GPIO_PORTB_AHB_ODR_R: *mut usize = 0x4005950C as *mut usize;
pub const GPIO_PORTB_AHB_PUR_R: *mut usize = 0x40059510 as *mut usize;
pub const GPIO_PORTB_AHB_PDR_R: *mut usize = 0x40059514 as *mut usize;
pub const GPIO_PORTB_AHB_SLR_R: *mut usize = 0x40059518 as *mut usize;
pub const GPIO_PORTB_AHB_DEN_R: *mut usize = 0x4005951C as *mut usize;
pub const GPIO_PORTB_AHB_LOCK_R: *mut usize = 0x40059520 as *mut usize;
pub const GPIO_PORTB_AHB_CR_R: *mut usize = 0x40059524 as *mut usize;
pub const GPIO_PORTB_AHB_AMSEL_R: *mut usize = 0x40059528 as *mut usize;
pub const GPIO_PORTB_AHB_PCTL_R: *mut usize = 0x4005952C as *mut usize;
pub const GPIO_PORTB_AHB_ADCCTL_R: *mut usize = 0x40059530 as *mut usize;
pub const GPIO_PORTB_AHB_DMACTL_R: *mut usize = 0x40059534 as *mut usize;
pub const GPIO_PORTB_AHB_SI_R: *mut usize = 0x40059538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTC AHB)
//
// *****************************************************************************
pub const GPIO_PORTC_AHB_DATA_BITS_R: *mut usize = 0x4005A000 as *mut usize;
pub const GPIO_PORTC_AHB_DATA_R: *mut usize = 0x4005A3FC as *mut usize;
pub const GPIO_PORTC_AHB_DIR_R: *mut usize = 0x4005A400 as *mut usize;
pub const GPIO_PORTC_AHB_IS_R: *mut usize = 0x4005A404 as *mut usize;
pub const GPIO_PORTC_AHB_IBE_R: *mut usize = 0x4005A408 as *mut usize;
pub const GPIO_PORTC_AHB_IEV_R: *mut usize = 0x4005A40C as *mut usize;
pub const GPIO_PORTC_AHB_IM_R: *mut usize = 0x4005A410 as *mut usize;
pub const GPIO_PORTC_AHB_RIS_R: *mut usize = 0x4005A414 as *mut usize;
pub const GPIO_PORTC_AHB_MIS_R: *mut usize = 0x4005A418 as *mut usize;
pub const GPIO_PORTC_AHB_ICR_R: *mut usize = 0x4005A41C as *mut usize;
pub const GPIO_PORTC_AHB_AFSEL_R: *mut usize = 0x4005A420 as *mut usize;
pub const GPIO_PORTC_AHB_DR2R_R: *mut usize = 0x4005A500 as *mut usize;
pub const GPIO_PORTC_AHB_DR4R_R: *mut usize = 0x4005A504 as *mut usize;
pub const GPIO_PORTC_AHB_DR8R_R: *mut usize = 0x4005A508 as *mut usize;
pub const GPIO_PORTC_AHB_ODR_R: *mut usize = 0x4005A50C as *mut usize;
pub const GPIO_PORTC_AHB_PUR_R: *mut usize = 0x4005A510 as *mut usize;
pub const GPIO_PORTC_AHB_PDR_R: *mut usize = 0x4005A514 as *mut usize;
pub const GPIO_PORTC_AHB_SLR_R: *mut usize = 0x4005A518 as *mut usize;
pub const GPIO_PORTC_AHB_DEN_R: *mut usize = 0x4005A51C as *mut usize;
pub const GPIO_PORTC_AHB_LOCK_R: *mut usize = 0x4005A520 as *mut usize;
pub const GPIO_PORTC_AHB_CR_R: *mut usize = 0x4005A524 as *mut usize;
pub const GPIO_PORTC_AHB_AMSEL_R: *mut usize = 0x4005A528 as *mut usize;
pub const GPIO_PORTC_AHB_PCTL_R: *mut usize = 0x4005A52C as *mut usize;
pub const GPIO_PORTC_AHB_ADCCTL_R: *mut usize = 0x4005A530 as *mut usize;
pub const GPIO_PORTC_AHB_DMACTL_R: *mut usize = 0x4005A534 as *mut usize;
pub const GPIO_PORTC_AHB_SI_R: *mut usize = 0x4005A538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTD AHB)
//
// *****************************************************************************
pub const GPIO_PORTD_AHB_DATA_BITS_R: *mut usize = 0x4005B000 as *mut usize;
pub const GPIO_PORTD_AHB_DATA_R: *mut usize = 0x4005B3FC as *mut usize;
pub const GPIO_PORTD_AHB_DIR_R: *mut usize = 0x4005B400 as *mut usize;
pub const GPIO_PORTD_AHB_IS_R: *mut usize = 0x4005B404 as *mut usize;
pub const GPIO_PORTD_AHB_IBE_R: *mut usize = 0x4005B408 as *mut usize;
pub const GPIO_PORTD_AHB_IEV_R: *mut usize = 0x4005B40C as *mut usize;
pub const GPIO_PORTD_AHB_IM_R: *mut usize = 0x4005B410 as *mut usize;
pub const GPIO_PORTD_AHB_RIS_R: *mut usize = 0x4005B414 as *mut usize;
pub const GPIO_PORTD_AHB_MIS_R: *mut usize = 0x4005B418 as *mut usize;
pub const GPIO_PORTD_AHB_ICR_R: *mut usize = 0x4005B41C as *mut usize;
pub const GPIO_PORTD_AHB_AFSEL_R: *mut usize = 0x4005B420 as *mut usize;
pub const GPIO_PORTD_AHB_DR2R_R: *mut usize = 0x4005B500 as *mut usize;
pub const GPIO_PORTD_AHB_DR4R_R: *mut usize = 0x4005B504 as *mut usize;
pub const GPIO_PORTD_AHB_DR8R_R: *mut usize = 0x4005B508 as *mut usize;
pub const GPIO_PORTD_AHB_ODR_R: *mut usize = 0x4005B50C as *mut usize;
pub const GPIO_PORTD_AHB_PUR_R: *mut usize = 0x4005B510 as *mut usize;
pub const GPIO_PORTD_AHB_PDR_R: *mut usize = 0x4005B514 as *mut usize;
pub const GPIO_PORTD_AHB_SLR_R: *mut usize = 0x4005B518 as *mut usize;
pub const GPIO_PORTD_AHB_DEN_R: *mut usize = 0x4005B51C as *mut usize;
pub const GPIO_PORTD_AHB_LOCK_R: *mut usize = 0x4005B520 as *mut usize;
pub const GPIO_PORTD_AHB_CR_R: *mut usize = 0x4005B524 as *mut usize;
pub const GPIO_PORTD_AHB_AMSEL_R: *mut usize = 0x4005B528 as *mut usize;
pub const GPIO_PORTD_AHB_PCTL_R: *mut usize = 0x4005B52C as *mut usize;
pub const GPIO_PORTD_AHB_ADCCTL_R: *mut usize = 0x4005B530 as *mut usize;
pub const GPIO_PORTD_AHB_DMACTL_R: *mut usize = 0x4005B534 as *mut usize;
pub const GPIO_PORTD_AHB_SI_R: *mut usize = 0x4005B538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTE AHB)
//
// *****************************************************************************
pub const GPIO_PORTE_AHB_DATA_BITS_R: *mut usize = 0x4005C000 as *mut usize;
pub const GPIO_PORTE_AHB_DATA_R: *mut usize = 0x4005C3FC as *mut usize;
pub const GPIO_PORTE_AHB_DIR_R: *mut usize = 0x4005C400 as *mut usize;
pub const GPIO_PORTE_AHB_IS_R: *mut usize = 0x4005C404 as *mut usize;
pub const GPIO_PORTE_AHB_IBE_R: *mut usize = 0x4005C408 as *mut usize;
pub const GPIO_PORTE_AHB_IEV_R: *mut usize = 0x4005C40C as *mut usize;
pub const GPIO_PORTE_AHB_IM_R: *mut usize = 0x4005C410 as *mut usize;
pub const GPIO_PORTE_AHB_RIS_R: *mut usize = 0x4005C414 as *mut usize;
pub const GPIO_PORTE_AHB_MIS_R: *mut usize = 0x4005C418 as *mut usize;
pub const GPIO_PORTE_AHB_ICR_R: *mut usize = 0x4005C41C as *mut usize;
pub const GPIO_PORTE_AHB_AFSEL_R: *mut usize = 0x4005C420 as *mut usize;
pub const GPIO_PORTE_AHB_DR2R_R: *mut usize = 0x4005C500 as *mut usize;
pub const GPIO_PORTE_AHB_DR4R_R: *mut usize = 0x4005C504 as *mut usize;
pub const GPIO_PORTE_AHB_DR8R_R: *mut usize = 0x4005C508 as *mut usize;
pub const GPIO_PORTE_AHB_ODR_R: *mut usize = 0x4005C50C as *mut usize;
pub const GPIO_PORTE_AHB_PUR_R: *mut usize = 0x4005C510 as *mut usize;
pub const GPIO_PORTE_AHB_PDR_R: *mut usize = 0x4005C514 as *mut usize;
pub const GPIO_PORTE_AHB_SLR_R: *mut usize = 0x4005C518 as *mut usize;
pub const GPIO_PORTE_AHB_DEN_R: *mut usize = 0x4005C51C as *mut usize;
pub const GPIO_PORTE_AHB_LOCK_R: *mut usize = 0x4005C520 as *mut usize;
pub const GPIO_PORTE_AHB_CR_R: *mut usize = 0x4005C524 as *mut usize;
pub const GPIO_PORTE_AHB_AMSEL_R: *mut usize = 0x4005C528 as *mut usize;
pub const GPIO_PORTE_AHB_PCTL_R: *mut usize = 0x4005C52C as *mut usize;
pub const GPIO_PORTE_AHB_ADCCTL_R: *mut usize = 0x4005C530 as *mut usize;
pub const GPIO_PORTE_AHB_DMACTL_R: *mut usize = 0x4005C534 as *mut usize;
pub const GPIO_PORTE_AHB_SI_R: *mut usize = 0x4005C538 as *mut usize;

// *****************************************************************************
//
// GPIO registers (PORTF AHB)
//
// *****************************************************************************
pub const GPIO_PORTF_AHB_DATA_BITS_R: *mut usize = 0x4005D000 as *mut usize;
pub const GPIO_PORTF_AHB_DATA_R: *mut usize = 0x4005D3FC as *mut usize;
pub const GPIO_PORTF_AHB_DIR_R: *mut usize = 0x4005D400 as *mut usize;
pub const GPIO_PORTF_AHB_IS_R: *mut usize = 0x4005D404 as *mut usize;
pub const GPIO_PORTF_AHB_IBE_R: *mut usize = 0x4005D408 as *mut usize;
pub const GPIO_PORTF_AHB_IEV_R: *mut usize = 0x4005D40C as *mut usize;
pub const GPIO_PORTF_AHB_IM_R: *mut usize = 0x4005D410 as *mut usize;
pub const GPIO_PORTF_AHB_RIS_R: *mut usize = 0x4005D414 as *mut usize;
pub const GPIO_PORTF_AHB_MIS_R: *mut usize = 0x4005D418 as *mut usize;
pub const GPIO_PORTF_AHB_ICR_R: *mut usize = 0x4005D41C as *mut usize;
pub const GPIO_PORTF_AHB_AFSEL_R: *mut usize = 0x4005D420 as *mut usize;
pub const GPIO_PORTF_AHB_DR2R_R: *mut usize = 0x4005D500 as *mut usize;
pub const GPIO_PORTF_AHB_DR4R_R: *mut usize = 0x4005D504 as *mut usize;
pub const GPIO_PORTF_AHB_DR8R_R: *mut usize = 0x4005D508 as *mut usize;
pub const GPIO_PORTF_AHB_ODR_R: *mut usize = 0x4005D50C as *mut usize;
pub const GPIO_PORTF_AHB_PUR_R: *mut usize = 0x4005D510 as *mut usize;
pub const GPIO_PORTF_AHB_PDR_R: *mut usize = 0x4005D514 as *mut usize;
pub const GPIO_PORTF_AHB_SLR_R: *mut usize = 0x4005D518 as *mut usize;
pub const GPIO_PORTF_AHB_DEN_R: *mut usize = 0x4005D51C as *mut usize;
pub const GPIO_PORTF_AHB_LOCK_R: *mut usize = 0x4005D520 as *mut usize;
pub const GPIO_PORTF_AHB_CR_R: *mut usize = 0x4005D524 as *mut usize;
pub const GPIO_PORTF_AHB_AMSEL_R: *mut usize = 0x4005D528 as *mut usize;
pub const GPIO_PORTF_AHB_PCTL_R: *mut usize = 0x4005D52C as *mut usize;
pub const GPIO_PORTF_AHB_ADCCTL_R: *mut usize = 0x4005D530 as *mut usize;
pub const GPIO_PORTF_AHB_DMACTL_R: *mut usize = 0x4005D534 as *mut usize;
pub const GPIO_PORTF_AHB_SI_R: *mut usize = 0x4005D538 as *mut usize;

// *****************************************************************************
//
// EEPROM registers (EEPROM)
//
// *****************************************************************************
pub const EEPROM_EESIZE_R: *mut usize = 0x400AF000 as *mut usize;
pub const EEPROM_EEBLOCK_R: *mut usize = 0x400AF004 as *mut usize;
pub const EEPROM_EEOFFSET_R: *mut usize = 0x400AF008 as *mut usize;
pub const EEPROM_EERDWR_R: *mut usize = 0x400AF010 as *mut usize;
pub const EEPROM_EERDWRINC_R: *mut usize = 0x400AF014 as *mut usize;
pub const EEPROM_EEDONE_R: *mut usize = 0x400AF018 as *mut usize;
pub const EEPROM_EESUPP_R: *mut usize = 0x400AF01C as *mut usize;
pub const EEPROM_EEUNLOCK_R: *mut usize = 0x400AF020 as *mut usize;
pub const EEPROM_EEPROT_R: *mut usize = 0x400AF030 as *mut usize;
pub const EEPROM_EEPASS0_R: *mut usize = 0x400AF034 as *mut usize;
pub const EEPROM_EEPASS1_R: *mut usize = 0x400AF038 as *mut usize;
pub const EEPROM_EEPASS2_R: *mut usize = 0x400AF03C as *mut usize;
pub const EEPROM_EEINT_R: *mut usize = 0x400AF040 as *mut usize;
pub const EEPROM_EEHIDE_R: *mut usize = 0x400AF050 as *mut usize;
pub const EEPROM_EEDBGME_R: *mut usize = 0x400AF080 as *mut usize;
pub const EEPROM_PP_R: *mut usize = 0x400AFFC0 as *mut usize;

// *****************************************************************************
//
// System Exception Module registers (SYSEXC)
//
// *****************************************************************************
pub const SYSEXC_RIS_R: *mut usize = 0x400F9000 as *mut usize;
pub const SYSEXC_IM_R: *mut usize = 0x400F9004 as *mut usize;
pub const SYSEXC_MIS_R: *mut usize = 0x400F9008 as *mut usize;
pub const SYSEXC_IC_R: *mut usize = 0x400F900C as *mut usize;

// *****************************************************************************
//
// Hibernation module registers (HIB)
//
// *****************************************************************************
pub const HIB_RTCC_R: *mut usize = 0x400FC000 as *mut usize;
pub const HIB_RTCM0_R: *mut usize = 0x400FC004 as *mut usize;
pub const HIB_RTCLD_R: *mut usize = 0x400FC00C as *mut usize;
pub const HIB_CTL_R: *mut usize = 0x400FC010 as *mut usize;
pub const HIB_IM_R: *mut usize = 0x400FC014 as *mut usize;
pub const HIB_RIS_R: *mut usize = 0x400FC018 as *mut usize;
pub const HIB_MIS_R: *mut usize = 0x400FC01C as *mut usize;
pub const HIB_IC_R: *mut usize = 0x400FC020 as *mut usize;
pub const HIB_RTCT_R: *mut usize = 0x400FC024 as *mut usize;
pub const HIB_RTCSS_R: *mut usize = 0x400FC028 as *mut usize;
pub const HIB_DATA_R: *mut usize = 0x400FC030 as *mut usize;

// *****************************************************************************
//
// FLASH registers (FLASH CTRL)
//
// *****************************************************************************
pub const FLASH_FMA_R: *mut usize = 0x400FD000 as *mut usize;
pub const FLASH_FMD_R: *mut usize = 0x400FD004 as *mut usize;
pub const FLASH_FMC_R: *mut usize = 0x400FD008 as *mut usize;
pub const FLASH_FCRIS_R: *mut usize = 0x400FD00C as *mut usize;
pub const FLASH_FCIM_R: *mut usize = 0x400FD010 as *mut usize;
pub const FLASH_FCMISC_R: *mut usize = 0x400FD014 as *mut usize;
pub const FLASH_FMC2_R: *mut usize = 0x400FD020 as *mut usize;
pub const FLASH_FWBVAL_R: *mut usize = 0x400FD030 as *mut usize;
pub const FLASH_FWBN_R: *mut usize = 0x400FD100 as *mut usize;
pub const FLASH_FSIZE_R: *mut usize = 0x400FDFC0 as *mut usize;
pub const FLASH_SSIZE_R: *mut usize = 0x400FDFC4 as *mut usize;
pub const FLASH_ROMSWMAP_R: *mut usize = 0x400FDFCC as *mut usize;
pub const FLASH_RMCTL_R: *mut usize = 0x400FE0F0 as *mut usize;
pub const FLASH_BOOTCFG_R: *mut usize = 0x400FE1D0 as *mut usize;
pub const FLASH_USERREG0_R: *mut usize = 0x400FE1E0 as *mut usize;
pub const FLASH_USERREG1_R: *mut usize = 0x400FE1E4 as *mut usize;
pub const FLASH_USERREG2_R: *mut usize = 0x400FE1E8 as *mut usize;
pub const FLASH_USERREG3_R: *mut usize = 0x400FE1EC as *mut usize;
pub const FLASH_FMPRE0_R: *mut usize = 0x400FE200 as *mut usize;
pub const FLASH_FMPRE1_R: *mut usize = 0x400FE204 as *mut usize;
pub const FLASH_FMPRE2_R: *mut usize = 0x400FE208 as *mut usize;
pub const FLASH_FMPRE3_R: *mut usize = 0x400FE20C as *mut usize;
pub const FLASH_FMPPE0_R: *mut usize = 0x400FE400 as *mut usize;
pub const FLASH_FMPPE1_R: *mut usize = 0x400FE404 as *mut usize;
pub const FLASH_FMPPE2_R: *mut usize = 0x400FE408 as *mut usize;
pub const FLASH_FMPPE3_R: *mut usize = 0x400FE40C as *mut usize;

// *****************************************************************************
//
// System Control registers (SYSCTL)
//
// *****************************************************************************
pub const SYSCTL_DID0_R: *mut usize = 0x400FE000 as *mut usize;
pub const SYSCTL_DID1_R: *mut usize = 0x400FE004 as *mut usize;
pub const SYSCTL_DC0_R: *mut usize = 0x400FE008 as *mut usize;
pub const SYSCTL_DC1_R: *mut usize = 0x400FE010 as *mut usize;
pub const SYSCTL_DC2_R: *mut usize = 0x400FE014 as *mut usize;
pub const SYSCTL_DC3_R: *mut usize = 0x400FE018 as *mut usize;
pub const SYSCTL_DC4_R: *mut usize = 0x400FE01C as *mut usize;
pub const SYSCTL_DC5_R: *mut usize = 0x400FE020 as *mut usize;
pub const SYSCTL_DC6_R: *mut usize = 0x400FE024 as *mut usize;
pub const SYSCTL_DC7_R: *mut usize = 0x400FE028 as *mut usize;
pub const SYSCTL_DC8_R: *mut usize = 0x400FE02C as *mut usize;
pub const SYSCTL_PBORCTL_R: *mut usize = 0x400FE030 as *mut usize;
pub const SYSCTL_SRCR0_R: *mut usize = 0x400FE040 as *mut usize;
pub const SYSCTL_SRCR1_R: *mut usize = 0x400FE044 as *mut usize;
pub const SYSCTL_SRCR2_R: *mut usize = 0x400FE048 as *mut usize;
pub const SYSCTL_RIS_R: *mut usize = 0x400FE050 as *mut usize;
pub const SYSCTL_IMC_R: *mut usize = 0x400FE054 as *mut usize;
pub const SYSCTL_MISC_R: *mut usize = 0x400FE058 as *mut usize;
pub const SYSCTL_RESC_R: *mut usize = 0x400FE05C as *mut usize;
pub const SYSCTL_RCC_R: *mut usize = 0x400FE060 as *mut usize;
pub const SYSCTL_GPIOHBCTL_R: *mut usize = 0x400FE06C as *mut usize;
pub const SYSCTL_RCC2_R: *mut usize = 0x400FE070 as *mut usize;
pub const SYSCTL_MOSCCTL_R: *mut usize = 0x400FE07C as *mut usize;
pub const SYSCTL_RCGC0_R: *mut usize = 0x400FE100 as *mut usize;
pub const SYSCTL_RCGC1_R: *mut usize = 0x400FE104 as *mut usize;
pub const SYSCTL_RCGC2_R: *mut usize = 0x400FE108 as *mut usize;
pub const SYSCTL_SCGC0_R: *mut usize = 0x400FE110 as *mut usize;
pub const SYSCTL_SCGC1_R: *mut usize = 0x400FE114 as *mut usize;
pub const SYSCTL_SCGC2_R: *mut usize = 0x400FE118 as *mut usize;
pub const SYSCTL_DCGC0_R: *mut usize = 0x400FE120 as *mut usize;
pub const SYSCTL_DCGC1_R: *mut usize = 0x400FE124 as *mut usize;
pub const SYSCTL_DCGC2_R: *mut usize = 0x400FE128 as *mut usize;
pub const SYSCTL_DSLPCLKCFG_R: *mut usize = 0x400FE144 as *mut usize;
pub const SYSCTL_SYSPROP_R: *mut usize = 0x400FE14C as *mut usize;
pub const SYSCTL_PIOSCCAL_R: *mut usize = 0x400FE150 as *mut usize;
pub const SYSCTL_PIOSCSTAT_R: *mut usize = 0x400FE154 as *mut usize;
pub const SYSCTL_PLLFREQ0_R: *mut usize = 0x400FE160 as *mut usize;
pub const SYSCTL_PLLFREQ1_R: *mut usize = 0x400FE164 as *mut usize;
pub const SYSCTL_PLLSTAT_R: *mut usize = 0x400FE168 as *mut usize;
pub const SYSCTL_DC9_R: *mut usize = 0x400FE190 as *mut usize;
pub const SYSCTL_NVMSTAT_R: *mut usize = 0x400FE1A0 as *mut usize;
pub const SYSCTL_PPWD_R: *mut usize = 0x400FE300 as *mut usize;
pub const SYSCTL_PPTIMER_R: *mut usize = 0x400FE304 as *mut usize;
pub const SYSCTL_PPGPIO_R: *mut usize = 0x400FE308 as *mut usize;
pub const SYSCTL_PPDMA_R: *mut usize = 0x400FE30C as *mut usize;
pub const SYSCTL_PPHIB_R: *mut usize = 0x400FE314 as *mut usize;
pub const SYSCTL_PPUART_R: *mut usize = 0x400FE318 as *mut usize;
pub const SYSCTL_PPSSI_R: *mut usize = 0x400FE31C as *mut usize;
pub const SYSCTL_PPI2C_R: *mut usize = 0x400FE320 as *mut usize;
pub const SYSCTL_PPUSB_R: *mut usize = 0x400FE328 as *mut usize;
pub const SYSCTL_PPCAN_R: *mut usize = 0x400FE334 as *mut usize;
pub const SYSCTL_PPADC_R: *mut usize = 0x400FE338 as *mut usize;
pub const SYSCTL_PPACMP_R: *mut usize = 0x400FE33C as *mut usize;
pub const SYSCTL_PPPWM_R: *mut usize = 0x400FE340 as *mut usize;
pub const SYSCTL_PPQEI_R: *mut usize = 0x400FE344 as *mut usize;
pub const SYSCTL_PPEEPROM_R: *mut usize = 0x400FE358 as *mut usize;
pub const SYSCTL_PPWTIMER_R: *mut usize = 0x400FE35C as *mut usize;
pub const SYSCTL_SRWD_R: *mut usize = 0x400FE500 as *mut usize;
pub const SYSCTL_SRTIMER_R: *mut usize = 0x400FE504 as *mut usize;
pub const SYSCTL_SRGPIO_R: *mut usize = 0x400FE508 as *mut usize;
pub const SYSCTL_SRDMA_R: *mut usize = 0x400FE50C as *mut usize;
pub const SYSCTL_SRHIB_R: *mut usize = 0x400FE514 as *mut usize;
pub const SYSCTL_SRUART_R: *mut usize = 0x400FE518 as *mut usize;
pub const SYSCTL_SRSSI_R: *mut usize = 0x400FE51C as *mut usize;
pub const SYSCTL_SRI2C_R: *mut usize = 0x400FE520 as *mut usize;
pub const SYSCTL_SRUSB_R: *mut usize = 0x400FE528 as *mut usize;
pub const SYSCTL_SRCAN_R: *mut usize = 0x400FE534 as *mut usize;
pub const SYSCTL_SRADC_R: *mut usize = 0x400FE538 as *mut usize;
pub const SYSCTL_SRACMP_R: *mut usize = 0x400FE53C as *mut usize;
pub const SYSCTL_SREEPROM_R: *mut usize = 0x400FE558 as *mut usize;
pub const SYSCTL_SRWTIMER_R: *mut usize = 0x400FE55C as *mut usize;
pub const SYSCTL_RCGCWD_R: *mut usize = 0x400FE600 as *mut usize;
pub const SYSCTL_RCGCTIMER_R: *mut usize = 0x400FE604 as *mut usize;
pub const SYSCTL_RCGCGPIO_R: *mut usize = 0x400FE608 as *mut usize;
pub const SYSCTL_RCGCDMA_R: *mut usize = 0x400FE60C as *mut usize;
pub const SYSCTL_RCGCHIB_R: *mut usize = 0x400FE614 as *mut usize;
pub const SYSCTL_RCGCUART_R: *mut usize = 0x400FE618 as *mut usize;
pub const SYSCTL_RCGCSSI_R: *mut usize = 0x400FE61C as *mut usize;
pub const SYSCTL_RCGCI2C_R: *mut usize = 0x400FE620 as *mut usize;
pub const SYSCTL_RCGCUSB_R: *mut usize = 0x400FE628 as *mut usize;
pub const SYSCTL_RCGCCAN_R: *mut usize = 0x400FE634 as *mut usize;
pub const SYSCTL_RCGCADC_R: *mut usize = 0x400FE638 as *mut usize;
pub const SYSCTL_RCGCACMP_R: *mut usize = 0x400FE63C as *mut usize;
pub const SYSCTL_RCGCEEPROM_R: *mut usize = 0x400FE658 as *mut usize;
pub const SYSCTL_RCGCWTIMER_R: *mut usize = 0x400FE65C as *mut usize;
pub const SYSCTL_SCGCWD_R: *mut usize = 0x400FE700 as *mut usize;
pub const SYSCTL_SCGCTIMER_R: *mut usize = 0x400FE704 as *mut usize;
pub const SYSCTL_SCGCGPIO_R: *mut usize = 0x400FE708 as *mut usize;
pub const SYSCTL_SCGCDMA_R: *mut usize = 0x400FE70C as *mut usize;
pub const SYSCTL_SCGCHIB_R: *mut usize = 0x400FE714 as *mut usize;
pub const SYSCTL_SCGCUART_R: *mut usize = 0x400FE718 as *mut usize;
pub const SYSCTL_SCGCSSI_R: *mut usize = 0x400FE71C as *mut usize;
pub const SYSCTL_SCGCI2C_R: *mut usize = 0x400FE720 as *mut usize;
pub const SYSCTL_SCGCUSB_R: *mut usize = 0x400FE728 as *mut usize;
pub const SYSCTL_SCGCCAN_R: *mut usize = 0x400FE734 as *mut usize;
pub const SYSCTL_SCGCADC_R: *mut usize = 0x400FE738 as *mut usize;
pub const SYSCTL_SCGCACMP_R: *mut usize = 0x400FE73C as *mut usize;
pub const SYSCTL_SCGCEEPROM_R: *mut usize = 0x400FE758 as *mut usize;
pub const SYSCTL_SCGCWTIMER_R: *mut usize = 0x400FE75C as *mut usize;
pub const SYSCTL_DCGCWD_R: *mut usize = 0x400FE800 as *mut usize;
pub const SYSCTL_DCGCTIMER_R: *mut usize = 0x400FE804 as *mut usize;
pub const SYSCTL_DCGCGPIO_R: *mut usize = 0x400FE808 as *mut usize;
pub const SYSCTL_DCGCDMA_R: *mut usize = 0x400FE80C as *mut usize;
pub const SYSCTL_DCGCHIB_R: *mut usize = 0x400FE814 as *mut usize;
pub const SYSCTL_DCGCUART_R: *mut usize = 0x400FE818 as *mut usize;
pub const SYSCTL_DCGCSSI_R: *mut usize = 0x400FE81C as *mut usize;
pub const SYSCTL_DCGCI2C_R: *mut usize = 0x400FE820 as *mut usize;
pub const SYSCTL_DCGCUSB_R: *mut usize = 0x400FE828 as *mut usize;
pub const SYSCTL_DCGCCAN_R: *mut usize = 0x400FE834 as *mut usize;
pub const SYSCTL_DCGCADC_R: *mut usize = 0x400FE838 as *mut usize;
pub const SYSCTL_DCGCACMP_R: *mut usize = 0x400FE83C as *mut usize;
pub const SYSCTL_DCGCEEPROM_R: *mut usize = 0x400FE858 as *mut usize;
pub const SYSCTL_DCGCWTIMER_R: *mut usize = 0x400FE85C as *mut usize;
pub const SYSCTL_PCWD_R: *mut usize = 0x400FE900 as *mut usize;
pub const SYSCTL_PCTIMER_R: *mut usize = 0x400FE904 as *mut usize;
pub const SYSCTL_PCGPIO_R: *mut usize = 0x400FE908 as *mut usize;
pub const SYSCTL_PCDMA_R: *mut usize = 0x400FE90C as *mut usize;
pub const SYSCTL_PCHIB_R: *mut usize = 0x400FE914 as *mut usize;
pub const SYSCTL_PCUART_R: *mut usize = 0x400FE918 as *mut usize;
pub const SYSCTL_PCSSI_R: *mut usize = 0x400FE91C as *mut usize;
pub const SYSCTL_PCI2C_R: *mut usize = 0x400FE920 as *mut usize;
pub const SYSCTL_PCUSB_R: *mut usize = 0x400FE928 as *mut usize;
pub const SYSCTL_PCCAN_R: *mut usize = 0x400FE934 as *mut usize;
pub const SYSCTL_PCADC_R: *mut usize = 0x400FE938 as *mut usize;
pub const SYSCTL_PCACMP_R: *mut usize = 0x400FE93C as *mut usize;
pub const SYSCTL_PCEEPROM_R: *mut usize = 0x400FE958 as *mut usize;
pub const SYSCTL_PCWTIMER_R: *mut usize = 0x400FE95C as *mut usize;
pub const SYSCTL_PRWD_R: *mut usize = 0x400FEA00 as *mut usize;
pub const SYSCTL_PRTIMER_R: *mut usize = 0x400FEA04 as *mut usize;
pub const SYSCTL_PRGPIO_R: *mut usize = 0x400FEA08 as *mut usize;
pub const SYSCTL_PRDMA_R: *mut usize = 0x400FEA0C as *mut usize;
pub const SYSCTL_PRHIB_R: *mut usize = 0x400FEA14 as *mut usize;
pub const SYSCTL_PRUART_R: *mut usize = 0x400FEA18 as *mut usize;
pub const SYSCTL_PRSSI_R: *mut usize = 0x400FEA1C as *mut usize;
pub const SYSCTL_PRI2C_R: *mut usize = 0x400FEA20 as *mut usize;
pub const SYSCTL_PRUSB_R: *mut usize = 0x400FEA28 as *mut usize;
pub const SYSCTL_PRCAN_R: *mut usize = 0x400FEA34 as *mut usize;
pub const SYSCTL_PRADC_R: *mut usize = 0x400FEA38 as *mut usize;
pub const SYSCTL_PRACMP_R: *mut usize = 0x400FEA3C as *mut usize;
pub const SYSCTL_PREEPROM_R: *mut usize = 0x400FEA58 as *mut usize;
pub const SYSCTL_PRWTIMER_R: *mut usize = 0x400FEA5C as *mut usize;

// *****************************************************************************
//
// Micro Direct Memory Access registers (UDMA)
//
// *****************************************************************************
pub const UDMA_STAT_R: *mut usize = 0x400FF000 as *mut usize;
pub const UDMA_CFG_R: *mut usize = 0x400FF004 as *mut usize;
pub const UDMA_CTLBASE_R: *mut usize = 0x400FF008 as *mut usize;
pub const UDMA_ALTBASE_R: *mut usize = 0x400FF00C as *mut usize;
pub const UDMA_WAITSTAT_R: *mut usize = 0x400FF010 as *mut usize;
pub const UDMA_SWREQ_R: *mut usize = 0x400FF014 as *mut usize;
pub const UDMA_USEBURSTSET_R: *mut usize = 0x400FF018 as *mut usize;
pub const UDMA_USEBURSTCLR_R: *mut usize = 0x400FF01C as *mut usize;
pub const UDMA_REQMASKSET_R: *mut usize = 0x400FF020 as *mut usize;
pub const UDMA_REQMASKCLR_R: *mut usize = 0x400FF024 as *mut usize;
pub const UDMA_ENASET_R: *mut usize = 0x400FF028 as *mut usize;
pub const UDMA_ENACLR_R: *mut usize = 0x400FF02C as *mut usize;
pub const UDMA_ALTSET_R: *mut usize = 0x400FF030 as *mut usize;
pub const UDMA_ALTCLR_R: *mut usize = 0x400FF034 as *mut usize;
pub const UDMA_PRIOSET_R: *mut usize = 0x400FF038 as *mut usize;
pub const UDMA_PRIOCLR_R: *mut usize = 0x400FF03C as *mut usize;
pub const UDMA_ERRCLR_R: *mut usize = 0x400FF04C as *mut usize;
pub const UDMA_CHASGN_R: *mut usize = 0x400FF500 as *mut usize;
pub const UDMA_CHIS_R: *mut usize = 0x400FF504 as *mut usize;
pub const UDMA_CHMAP0_R: *mut usize = 0x400FF510 as *mut usize;
pub const UDMA_CHMAP1_R: *mut usize = 0x400FF514 as *mut usize;
pub const UDMA_CHMAP2_R: *mut usize = 0x400FF518 as *mut usize;
pub const UDMA_CHMAP3_R: *mut usize = 0x400FF51C as *mut usize;

// *****************************************************************************
//
// Micro Direct Memory Access (uDMA) offsets (UDMA)
//
// *****************************************************************************
pub const UDMA_SRCENDP: usize = 0x00000000; // DMA Channel Source Address End Pointer
pub const UDMA_DSTENDP: usize = 0x00000004; // DMA Channel Destination Address End Pointer
pub const UDMA_CHCTL: usize = 0x00000008; // DMA Channel Control Word

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOAD register.
//
// *****************************************************************************
pub const WDT_LOAD_M: usize = 0xFFFFFFFF; // Watchdog Load Value
pub const WDT_LOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_VALUE register.
//
// *****************************************************************************
pub const WDT_VALUE_M: usize = 0xFFFFFFFF; // Watchdog Value
pub const WDT_VALUE_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_CTL register.
//
// *****************************************************************************
pub const WDT_CTL_WRC: usize = 0x80000000; // Write Complete
pub const WDT_CTL_INTTYPE: usize = 0x00000004; // Watchdog Interrupt Type
pub const WDT_CTL_RESEN: usize = 0x00000002; // Watchdog Reset Enable
pub const WDT_CTL_INTEN: usize = 0x00000001; // Watchdog Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_ICR register.
//
// *****************************************************************************
pub const WDT_ICR_M: usize = 0xFFFFFFFF; // Watchdog Interrupt Clear
pub const WDT_ICR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_RIS register.
//
// *****************************************************************************
pub const WDT_RIS_WDTRIS: usize = 0x00000001; // Watchdog Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_MIS register.
//
// *****************************************************************************
pub const WDT_MIS_WDTMIS: usize = 0x00000001; // Watchdog Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_TEST register.
//
// *****************************************************************************
pub const WDT_TEST_STALL: usize = 0x00000100; // Watchdog Stall Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the WDT_O_LOCK register.
//
// *****************************************************************************
pub const WDT_LOCK_M: usize = 0xFFFFFFFF; // Watchdog Lock
pub const WDT_LOCK_UNLOCKED: usize = 0x00000000; // Unlocked
pub const WDT_LOCK_LOCKED: usize = 0x00000001; // Locked

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_IM register.
//
// *****************************************************************************
pub const GPIO_IM_GPIO_M: usize = 0x000000FF; // GPIO Interrupt Mask Enable
pub const GPIO_IM_GPIO_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_RIS register.
//
// *****************************************************************************
pub const GPIO_RIS_GPIO_M: usize = 0x000000FF; // GPIO Interrupt Raw Status
pub const GPIO_RIS_GPIO_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_MIS register.
//
// *****************************************************************************
pub const GPIO_MIS_GPIO_M: usize = 0x000000FF; // GPIO Masked Interrupt Status
pub const GPIO_MIS_GPIO_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_ICR register.
//
// *****************************************************************************
pub const GPIO_ICR_GPIO_M: usize = 0x000000FF; // GPIO Interrupt Clear
pub const GPIO_ICR_GPIO_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_LOCK register.
//
// *****************************************************************************
pub const GPIO_LOCK_M: usize = 0xFFFFFFFF; // GPIO Lock
pub const GPIO_LOCK_UNLOCKED: usize = 0x00000000; // The GPIOCR register is unlocked and may be modified
pub const GPIO_LOCK_LOCKED: usize = 0x00000001; // The GPIOCR register is locked and may not be modified
pub const GPIO_LOCK_KEY: usize = 0x4C4F434B; // Unlocks the GPIO_CR register

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_O_SI register.
//
// *****************************************************************************
pub const GPIO_SI_SUM: usize = 0x00000001; // Summary Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port A.
//
// *****************************************************************************
pub const GPIO_PCTL_PA7_M: usize = 0xF0000000; // PA7 mask
pub const GPIO_PCTL_PA7_I2C1SDA: usize = 0x30000000; // I2C1SDA on PA7
pub const GPIO_PCTL_PA6_M: usize = 0x0F000000; // PA6 mask
pub const GPIO_PCTL_PA6_I2C1SCL: usize = 0x03000000; // I2C1SCL on PA6
pub const GPIO_PCTL_PA5_M: usize = 0x00F00000; // PA5 mask
pub const GPIO_PCTL_PA5_SSI0TX: usize = 0x00200000; // SSI0TX on PA5
pub const GPIO_PCTL_PA4_M: usize = 0x000F0000; // PA4 mask
pub const GPIO_PCTL_PA4_SSI0RX: usize = 0x00020000; // SSI0RX on PA4
pub const GPIO_PCTL_PA3_M: usize = 0x0000F000; // PA3 mask
pub const GPIO_PCTL_PA3_SSI0FSS: usize = 0x00002000; // SSI0FSS on PA3
pub const GPIO_PCTL_PA2_M: usize = 0x00000F00; // PA2 mask
pub const GPIO_PCTL_PA2_SSI0CLK: usize = 0x00000200; // SSI0CLK on PA2
pub const GPIO_PCTL_PA1_M: usize = 0x000000F0; // PA1 mask
pub const GPIO_PCTL_PA1_U0TX: usize = 0x00000010; // U0TX on PA1
pub const GPIO_PCTL_PA0_M: usize = 0x0000000F; // PA0 mask
pub const GPIO_PCTL_PA0_U0RX: usize = 0x00000001; // U0RX on PA0

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port B.
//
// *****************************************************************************
pub const GPIO_PCTL_PB7_M: usize = 0xF0000000; // PB7 mask
pub const GPIO_PCTL_PB7_SSI2TX: usize = 0x20000000; // SSI2TX on PB7
pub const GPIO_PCTL_PB7_T0CCP1: usize = 0x70000000; // T0CCP1 on PB7
pub const GPIO_PCTL_PB6_M: usize = 0x0F000000; // PB6 mask
pub const GPIO_PCTL_PB6_SSI2RX: usize = 0x02000000; // SSI2RX on PB6
pub const GPIO_PCTL_PB6_T0CCP0: usize = 0x07000000; // T0CCP0 on PB6
pub const GPIO_PCTL_PB5_M: usize = 0x00F00000; // PB5 mask
pub const GPIO_PCTL_PB5_SSI2FSS: usize = 0x00200000; // SSI2FSS on PB5
pub const GPIO_PCTL_PB5_T1CCP1: usize = 0x00700000; // T1CCP1 on PB5
pub const GPIO_PCTL_PB5_CAN0TX: usize = 0x00800000; // CAN0TX on PB5
pub const GPIO_PCTL_PB4_M: usize = 0x000F0000; // PB4 mask
pub const GPIO_PCTL_PB4_SSI2CLK: usize = 0x00020000; // SSI2CLK on PB4
pub const GPIO_PCTL_PB4_T1CCP0: usize = 0x00070000; // T1CCP0 on PB4
pub const GPIO_PCTL_PB4_CAN0RX: usize = 0x00080000; // CAN0RX on PB4
pub const GPIO_PCTL_PB3_M: usize = 0x0000F000; // PB3 mask
pub const GPIO_PCTL_PB3_I2C0SDA: usize = 0x00003000; // I2C0SDA on PB3
pub const GPIO_PCTL_PB3_T3CCP1: usize = 0x00007000; // T3CCP1 on PB3
pub const GPIO_PCTL_PB2_M: usize = 0x00000F00; // PB2 mask
pub const GPIO_PCTL_PB2_I2C0SCL: usize = 0x00000300; // I2C0SCL on PB2
pub const GPIO_PCTL_PB2_T3CCP0: usize = 0x00000700; // T3CCP0 on PB2
pub const GPIO_PCTL_PB1_M: usize = 0x000000F0; // PB1 mask
pub const GPIO_PCTL_PB1_U1TX: usize = 0x00000010; // U1TX on PB1
pub const GPIO_PCTL_PB1_T2CCP1: usize = 0x00000070; // T2CCP1 on PB1
pub const GPIO_PCTL_PB0_M: usize = 0x0000000F; // PB0 mask
pub const GPIO_PCTL_PB0_U1RX: usize = 0x00000001; // U1RX on PB0
pub const GPIO_PCTL_PB0_T2CCP0: usize = 0x00000007; // T2CCP0 on PB0

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port C.
//
// *****************************************************************************
pub const GPIO_PCTL_PC7_M: usize = 0xF0000000; // PC7 mask
pub const GPIO_PCTL_PC7_U3TX: usize = 0x10000000; // U3TX on PC7
pub const GPIO_PCTL_PC7_WT1CCP1: usize = 0x70000000; // WT1CCP1 on PC7
pub const GPIO_PCTL_PC6_M: usize = 0x0F000000; // PC6 mask
pub const GPIO_PCTL_PC6_U3RX: usize = 0x01000000; // U3RX on PC6
pub const GPIO_PCTL_PC6_WT1CCP0: usize = 0x07000000; // WT1CCP0 on PC6
pub const GPIO_PCTL_PC5_M: usize = 0x00F00000; // PC5 mask
pub const GPIO_PCTL_PC5_U4TX: usize = 0x00100000; // U4TX on PC5
pub const GPIO_PCTL_PC5_U1TX: usize = 0x00200000; // U1TX on PC5
pub const GPIO_PCTL_PC5_WT0CCP1: usize = 0x00700000; // WT0CCP1 on PC5
pub const GPIO_PCTL_PC5_U1CTS: usize = 0x00800000; // U1CTS on PC5
pub const GPIO_PCTL_PC4_M: usize = 0x000F0000; // PC4 mask
pub const GPIO_PCTL_PC4_U4RX: usize = 0x00010000; // U4RX on PC4
pub const GPIO_PCTL_PC4_U1RX: usize = 0x00020000; // U1RX on PC4
pub const GPIO_PCTL_PC4_WT0CCP0: usize = 0x00070000; // WT0CCP0 on PC4
pub const GPIO_PCTL_PC4_U1RTS: usize = 0x00080000; // U1RTS on PC4
pub const GPIO_PCTL_PC3_M: usize = 0x0000F000; // PC3 mask
pub const GPIO_PCTL_PC3_TDO: usize = 0x00001000; // TDO on PC3
pub const GPIO_PCTL_PC3_T5CCP1: usize = 0x00007000; // T5CCP1 on PC3
pub const GPIO_PCTL_PC2_M: usize = 0x00000F00; // PC2 mask
pub const GPIO_PCTL_PC2_TDI: usize = 0x00000100; // TDI on PC2
pub const GPIO_PCTL_PC2_T5CCP0: usize = 0x00000700; // T5CCP0 on PC2
pub const GPIO_PCTL_PC1_M: usize = 0x000000F0; // PC1 mask
pub const GPIO_PCTL_PC1_TMS: usize = 0x00000010; // TMS on PC1
pub const GPIO_PCTL_PC1_T4CCP1: usize = 0x00000070; // T4CCP1 on PC1
pub const GPIO_PCTL_PC0_M: usize = 0x0000000F; // PC0 mask
pub const GPIO_PCTL_PC0_TCK: usize = 0x00000001; // TCK on PC0
pub const GPIO_PCTL_PC0_T4CCP0: usize = 0x00000007; // T4CCP0 on PC0

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port D.
//
// *****************************************************************************
pub const GPIO_PCTL_PD7_M: usize = 0xF0000000; // PD7 mask
pub const GPIO_PCTL_PD7_U2TX: usize = 0x10000000; // U2TX on PD7
pub const GPIO_PCTL_PD7_WT5CCP1: usize = 0x70000000; // WT5CCP1 on PD7
pub const GPIO_PCTL_PD7_NMI: usize = 0x80000000; // NMI on PD7
pub const GPIO_PCTL_PD6_M: usize = 0x0F000000; // PD6 mask
pub const GPIO_PCTL_PD6_U2RX: usize = 0x01000000; // U2RX on PD6
pub const GPIO_PCTL_PD6_WT5CCP0: usize = 0x07000000; // WT5CCP0 on PD6
pub const GPIO_PCTL_PD5_M: usize = 0x00F00000; // PD5 mask
pub const GPIO_PCTL_PD5_U6TX: usize = 0x00100000; // U6TX on PD5
pub const GPIO_PCTL_PD5_WT4CCP1: usize = 0x00700000; // WT4CCP1 on PD5
pub const GPIO_PCTL_PD4_M: usize = 0x000F0000; // PD4 mask
pub const GPIO_PCTL_PD4_U6RX: usize = 0x00010000; // U6RX on PD4
pub const GPIO_PCTL_PD4_WT4CCP0: usize = 0x00070000; // WT4CCP0 on PD4
pub const GPIO_PCTL_PD3_M: usize = 0x0000F000; // PD3 mask
pub const GPIO_PCTL_PD3_SSI3TX: usize = 0x00001000; // SSI3TX on PD3
pub const GPIO_PCTL_PD3_SSI1TX: usize = 0x00002000; // SSI1TX on PD3
pub const GPIO_PCTL_PD3_WT3CCP1: usize = 0x00007000; // WT3CCP1 on PD3
pub const GPIO_PCTL_PD2_M: usize = 0x00000F00; // PD2 mask
pub const GPIO_PCTL_PD2_SSI3RX: usize = 0x00000100; // SSI3RX on PD2
pub const GPIO_PCTL_PD2_SSI1RX: usize = 0x00000200; // SSI1RX on PD2
pub const GPIO_PCTL_PD2_WT3CCP0: usize = 0x00000700; // WT3CCP0 on PD2
pub const GPIO_PCTL_PD1_M: usize = 0x000000F0; // PD1 mask
pub const GPIO_PCTL_PD1_SSI3FSS: usize = 0x00000010; // SSI3FSS on PD1
pub const GPIO_PCTL_PD1_SSI1FSS: usize = 0x00000020; // SSI1FSS on PD1
pub const GPIO_PCTL_PD1_I2C3SDA: usize = 0x00000030; // I2C3SDA on PD1
pub const GPIO_PCTL_PD1_WT2CCP1: usize = 0x00000070; // WT2CCP1 on PD1
pub const GPIO_PCTL_PD0_M: usize = 0x0000000F; // PD0 mask
pub const GPIO_PCTL_PD0_SSI3CLK: usize = 0x00000001; // SSI3CLK on PD0
pub const GPIO_PCTL_PD0_SSI1CLK: usize = 0x00000002; // SSI1CLK on PD0
pub const GPIO_PCTL_PD0_I2C3SCL: usize = 0x00000003; // I2C3SCL on PD0
pub const GPIO_PCTL_PD0_WT2CCP0: usize = 0x00000007; // WT2CCP0 on PD0

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port E.
//
// *****************************************************************************
pub const GPIO_PCTL_PE5_M: usize = 0x00F00000; // PE5 mask
pub const GPIO_PCTL_PE5_U5TX: usize = 0x00100000; // U5TX on PE5
pub const GPIO_PCTL_PE5_I2C2SDA: usize = 0x00300000; // I2C2SDA on PE5
pub const GPIO_PCTL_PE5_CAN0TX: usize = 0x00800000; // CAN0TX on PE5
pub const GPIO_PCTL_PE4_M: usize = 0x000F0000; // PE4 mask
pub const GPIO_PCTL_PE4_U5RX: usize = 0x00010000; // U5RX on PE4
pub const GPIO_PCTL_PE4_I2C2SCL: usize = 0x00030000; // I2C2SCL on PE4
pub const GPIO_PCTL_PE4_CAN0RX: usize = 0x00080000; // CAN0RX on PE4
pub const GPIO_PCTL_PE3_M: usize = 0x0000F000; // PE3 mask
pub const GPIO_PCTL_PE2_M: usize = 0x00000F00; // PE2 mask
pub const GPIO_PCTL_PE1_M: usize = 0x000000F0; // PE1 mask
pub const GPIO_PCTL_PE1_U7TX: usize = 0x00000010; // U7TX on PE1
pub const GPIO_PCTL_PE0_M: usize = 0x0000000F; // PE0 mask
pub const GPIO_PCTL_PE0_U7RX: usize = 0x00000001; // U7RX on PE0

// *****************************************************************************
//
// The following are defines for the bit fields in the GPIO_PCTL register for
// port F.
//
// *****************************************************************************
pub const GPIO_PCTL_PF4_M: usize = 0x000F0000; // PF4 mask
pub const GPIO_PCTL_PF4_T2CCP0: usize = 0x00070000; // T2CCP0 on PF4
pub const GPIO_PCTL_PF3_M: usize = 0x0000F000; // PF3 mask
pub const GPIO_PCTL_PF3_SSI1FSS: usize = 0x00002000; // SSI1FSS on PF3
pub const GPIO_PCTL_PF3_CAN0TX: usize = 0x00003000; // CAN0TX on PF3
pub const GPIO_PCTL_PF3_T1CCP1: usize = 0x00007000; // T1CCP1 on PF3
pub const GPIO_PCTL_PF3_TRCLK: usize = 0x0000E000; // TRCLK on PF3
pub const GPIO_PCTL_PF2_M: usize = 0x00000F00; // PF2 mask
pub const GPIO_PCTL_PF2_SSI1CLK: usize = 0x00000200; // SSI1CLK on PF2
pub const GPIO_PCTL_PF2_T1CCP0: usize = 0x00000700; // T1CCP0 on PF2
pub const GPIO_PCTL_PF2_TRD0: usize = 0x00000E00; // TRD0 on PF2
pub const GPIO_PCTL_PF1_M: usize = 0x000000F0; // PF1 mask
pub const GPIO_PCTL_PF1_U1CTS: usize = 0x00000010; // U1CTS on PF1
pub const GPIO_PCTL_PF1_SSI1TX: usize = 0x00000020; // SSI1TX on PF1
pub const GPIO_PCTL_PF1_T0CCP1: usize = 0x00000070; // T0CCP1 on PF1
pub const GPIO_PCTL_PF1_C1O: usize = 0x00000090; // C1O on PF1
pub const GPIO_PCTL_PF1_TRD1: usize = 0x000000E0; // TRD1 on PF1
pub const GPIO_PCTL_PF0_M: usize = 0x0000000F; // PF0 mask
pub const GPIO_PCTL_PF0_U1RTS: usize = 0x00000001; // U1RTS on PF0
pub const GPIO_PCTL_PF0_SSI1RX: usize = 0x00000002; // SSI1RX on PF0
pub const GPIO_PCTL_PF0_CAN0RX: usize = 0x00000003; // CAN0RX on PF0
pub const GPIO_PCTL_PF0_T0CCP0: usize = 0x00000007; // T0CCP0 on PF0
pub const GPIO_PCTL_PF0_NMI: usize = 0x00000008; // NMI on PF0
pub const GPIO_PCTL_PF0_C0O: usize = 0x00000009; // C0O on PF0
pub const GPIO_PCTL_PF0_TRD2: usize = 0x0000000E; // TRD2 on PF0

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR0 register.
//
// *****************************************************************************
pub const SSI_CR0_SCR_M: usize = 0x0000FF00; // SSI Serial Clock Rate
pub const SSI_CR0_SPH: usize = 0x00000080; // SSI Serial Clock Phase
pub const SSI_CR0_SPO: usize = 0x00000040; // SSI Serial Clock Polarity
pub const SSI_CR0_FRF_M: usize = 0x00000030; // SSI Frame Format Select
pub const SSI_CR0_FRF_MOTO: usize = 0x00000000; // Freescale SPI Frame Format
pub const SSI_CR0_FRF_TI: usize = 0x00000010; // Texas Instruments Synchronous Serial Frame Format
pub const SSI_CR0_FRF_NMW: usize = 0x00000020; // MICROWIRE Frame Format
pub const SSI_CR0_DSS_M: usize = 0x0000000F; // SSI Data Size Select
pub const SSI_CR0_DSS_4: usize = 0x00000003; // 4-bit data
pub const SSI_CR0_DSS_5: usize = 0x00000004; // 5-bit data
pub const SSI_CR0_DSS_6: usize = 0x00000005; // 6-bit data
pub const SSI_CR0_DSS_7: usize = 0x00000006; // 7-bit data
pub const SSI_CR0_DSS_8: usize = 0x00000007; // 8-bit data
pub const SSI_CR0_DSS_9: usize = 0x00000008; // 9-bit data
pub const SSI_CR0_DSS_10: usize = 0x00000009; // 10-bit data
pub const SSI_CR0_DSS_11: usize = 0x0000000A; // 11-bit data
pub const SSI_CR0_DSS_12: usize = 0x0000000B; // 12-bit data
pub const SSI_CR0_DSS_13: usize = 0x0000000C; // 13-bit data
pub const SSI_CR0_DSS_14: usize = 0x0000000D; // 14-bit data
pub const SSI_CR0_DSS_15: usize = 0x0000000E; // 15-bit data
pub const SSI_CR0_DSS_16: usize = 0x0000000F; // 16-bit data
pub const SSI_CR0_SCR_S: usize = 8;

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CR1 register.
//
// *****************************************************************************
pub const SSI_CR1_EOT: usize = 0x00000010; // End of Transmission
pub const SSI_CR1_SOD: usize = 0x00000008; // SSI Slave Mode Output Disable
pub const SSI_CR1_MS: usize = 0x00000004; // SSI Master/Slave Select
pub const SSI_CR1_SSE: usize = 0x00000002; // SSI Synchronous Serial Port Enable
pub const SSI_CR1_LBM: usize = 0x00000001; // SSI Loopback Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DR register.
//
// *****************************************************************************
pub const SSI_DR_DATA_M: usize = 0x0000FFFF; // SSI Receive/Transmit Data
pub const SSI_DR_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_SR register.
//
// *****************************************************************************
pub const SSI_SR_BSY: usize = 0x00000010; // SSI Busy Bit
pub const SSI_SR_RFF: usize = 0x00000008; // SSI Receive FIFO Full
pub const SSI_SR_RNE: usize = 0x00000004; // SSI Receive FIFO Not Empty
pub const SSI_SR_TNF: usize = 0x00000002; // SSI Transmit FIFO Not Full
pub const SSI_SR_TFE: usize = 0x00000001; // SSI Transmit FIFO Empty

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CPSR register.
//
// *****************************************************************************
pub const SSI_CPSR_CPSDVSR_M: usize = 0x000000FF; // SSI Clock Prescale Divisor
pub const SSI_CPSR_CPSDVSR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_IM register.
//
// *****************************************************************************
pub const SSI_IM_TXIM: usize = 0x00000008; // SSI Transmit FIFO Interrupt Mask
pub const SSI_IM_RXIM: usize = 0x00000004; // SSI Receive FIFO Interrupt Mask
pub const SSI_IM_RTIM: usize = 0x00000002; // SSI Receive Time-Out Interrupt Mask
pub const SSI_IM_RORIM: usize = 0x00000001; // SSI Receive Overrun Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_RIS register.
//
// *****************************************************************************
pub const SSI_RIS_TXRIS: usize = 0x00000008; // SSI Transmit FIFO Raw Interrupt Status
pub const SSI_RIS_RXRIS: usize = 0x00000004; // SSI Receive FIFO Raw Interrupt Status
pub const SSI_RIS_RTRIS: usize = 0x00000002; // SSI Receive Time-Out Raw Interrupt Status
pub const SSI_RIS_RORRIS: usize = 0x00000001; // SSI Receive Overrun Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_MIS register.
//
// *****************************************************************************
pub const SSI_MIS_TXMIS: usize = 0x00000008; // SSI Transmit FIFO Masked Interrupt Status
pub const SSI_MIS_RXMIS: usize = 0x00000004; // SSI Receive FIFO Masked Interrupt Status
pub const SSI_MIS_RTMIS: usize = 0x00000002; // SSI Receive Time-Out Masked Interrupt Status
pub const SSI_MIS_RORMIS: usize = 0x00000001; // SSI Receive Overrun Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_ICR register.
//
// *****************************************************************************
pub const SSI_ICR_RTIC: usize = 0x00000002; // SSI Receive Time-Out Interrupt Clear
pub const SSI_ICR_RORIC: usize = 0x00000001; // SSI Receive Overrun Interrupt Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_DMACTL register.
//
// *****************************************************************************
pub const SSI_DMACTL_TXDMAE: usize = 0x00000002; // Transmit DMA Enable
pub const SSI_DMACTL_RXDMAE: usize = 0x00000001; // Receive DMA Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the SSI_O_CC register.
//
// *****************************************************************************
pub const SSI_CC_CS_M: usize = 0x0000000F; // SSI Baud Clock Source
pub const SSI_CC_CS_SYSPLL: usize = 0x00000000; // Either the system clock (if the PLL bypass is in effect) or the PLL output (default)
pub const SSI_CC_CS_PIOSC: usize = 0x00000005; // PIOSC

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DR register.
//
// *****************************************************************************
pub const UART_DR_OE: usize = 0x00000800; // UART Overrun Error
pub const UART_DR_BE: usize = 0x00000400; // UART Break Error
pub const UART_DR_PE: usize = 0x00000200; // UART Parity Error
pub const UART_DR_FE: usize = 0x00000100; // UART Framing Error
pub const UART_DR_DATA_M: usize = 0x000000FF; // Data Transmitted or Received
pub const UART_DR_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RSR register.
//
// *****************************************************************************
pub const UART_RSR_OE: usize = 0x00000008; // UART Overrun Error
pub const UART_RSR_BE: usize = 0x00000004; // UART Break Error
pub const UART_RSR_PE: usize = 0x00000002; // UART Parity Error
pub const UART_RSR_FE: usize = 0x00000001; // UART Framing Error

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
// *****************************************************************************
pub const UART_ECR_DATA_M: usize = 0x000000FF; // Error Clear
pub const UART_ECR_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FR register.
//
// *****************************************************************************
pub const UART_FR_TXFE: usize = 0x00000080; // UART Transmit FIFO Empty
pub const UART_FR_RXFF: usize = 0x00000040; // UART Receive FIFO Full
pub const UART_FR_TXFF: usize = 0x00000020; // UART Transmit FIFO Full
pub const UART_FR_RXFE: usize = 0x00000010; // UART Receive FIFO Empty
pub const UART_FR_BUSY: usize = 0x00000008; // UART Busy
pub const UART_FR_CTS: usize = 0x00000001; // Clear To Send

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
// *****************************************************************************
pub const UART_ILPR_ILPDVSR_M: usize = 0x000000FF; // IrDA Low-Power Divisor
pub const UART_ILPR_ILPDVSR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IBRD register.
//
// *****************************************************************************
pub const UART_IBRD_DIVINT_M: usize = 0x0000FFFF; // Integer Baud-Rate Divisor
pub const UART_IBRD_DIVINT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FBRD register.
//
// *****************************************************************************
pub const UART_FBRD_DIVFRAC_M: usize = 0x0000003F; // Fractional Baud-Rate Divisor
pub const UART_FBRD_DIVFRAC_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
// *****************************************************************************
pub const UART_LCRH_SPS: usize = 0x00000080; // UART Stick Parity Select
pub const UART_LCRH_WLEN_M: usize = 0x00000060; // UART Word Length
pub const UART_LCRH_WLEN_5: usize = 0x00000000; // 5 bits (default)
pub const UART_LCRH_WLEN_6: usize = 0x00000020; // 6 bits
pub const UART_LCRH_WLEN_7: usize = 0x00000040; // 7 bits
pub const UART_LCRH_WLEN_8: usize = 0x00000060; // 8 bits
pub const UART_LCRH_FEN: usize = 0x00000010; // UART Enable FIFOs
pub const UART_LCRH_STP2: usize = 0x00000008; // UART Two Stop Bits Select
pub const UART_LCRH_EPS: usize = 0x00000004; // UART Even Parity Select
pub const UART_LCRH_PEN: usize = 0x00000002; // UART Parity Enable
pub const UART_LCRH_BRK: usize = 0x00000001; // UART Send Break

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CTL register.
//
// *****************************************************************************
pub const UART_CTL_RXE: usize = 0x00000200; // UART Receive Enable
pub const UART_CTL_TXE: usize = 0x00000100; // UART Transmit Enable
pub const UART_CTL_LBE: usize = 0x00000080; // UART Loop Back Enable
pub const UART_CTL_LIN: usize = 0x00000040; // LIN Mode Enable
pub const UART_CTL_HSE: usize = 0x00000020; // High-Speed Enable
pub const UART_CTL_EOT: usize = 0x00000010; // End of Transmission
pub const UART_CTL_SMART: usize = 0x00000008; // ISO 7816 Smart Card Support
pub const UART_CTL_SIRLP: usize = 0x00000004; // UART SIR Low-Power Mode
pub const UART_CTL_SIREN: usize = 0x00000002; // UART SIR Enable
pub const UART_CTL_UARTEN: usize = 0x00000001; // UART Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IFLS register.
//
// *****************************************************************************
pub const UART_IFLS_RX_M: usize = 0x00000038; // UART Receive Interrupt FIFO Level Select
pub const UART_IFLS_RX1_8: usize = 0x00000000; // RX FIFO >= 1/8 full
pub const UART_IFLS_RX2_8: usize = 0x00000008; // RX FIFO >= 1/4 full
pub const UART_IFLS_RX4_8: usize = 0x00000010; // RX FIFO >= 1/2 full (default)
pub const UART_IFLS_RX6_8: usize = 0x00000018; // RX FIFO >= 3/4 full
pub const UART_IFLS_RX7_8: usize = 0x00000020; // RX FIFO >= 7/8 full
pub const UART_IFLS_TX_M: usize = 0x00000007; // UART Transmit Interrupt FIFO Level Select
pub const UART_IFLS_TX1_8: usize = 0x00000000; // TX FIFO <= 1/8 full
pub const UART_IFLS_TX2_8: usize = 0x00000001; // TX FIFO <= 1/4 full
pub const UART_IFLS_TX4_8: usize = 0x00000002; // TX FIFO <= 1/2 full (default)
pub const UART_IFLS_TX6_8: usize = 0x00000003; // TX FIFO <= 3/4 full
pub const UART_IFLS_TX7_8: usize = 0x00000004; // TX FIFO <= 7/8 full

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IM register.
//
// *****************************************************************************
pub const UART_IM_LME5IM: usize = 0x00008000; // LIN Mode Edge 5 Interrupt Mask
pub const UART_IM_LME1IM: usize = 0x00004000; // LIN Mode Edge 1 Interrupt Mask
pub const UART_IM_LMSBIM: usize = 0x00002000; // LIN Mode Sync Break Interrupt Mask
pub const UART_IM_9BITIM: usize = 0x00001000; // 9-Bit Mode Interrupt Mask
pub const UART_IM_OEIM: usize = 0x00000400; // UART Overrun Error Interrupt Mask
pub const UART_IM_BEIM: usize = 0x00000200; // UART Break Error Interrupt Mask
pub const UART_IM_PEIM: usize = 0x00000100; // UART Parity Error Interrupt Mask
pub const UART_IM_FEIM: usize = 0x00000080; // UART Framing Error Interrupt Mask
pub const UART_IM_RTIM: usize = 0x00000040; // UART Receive Time-Out Interrupt Mask
pub const UART_IM_TXIM: usize = 0x00000020; // UART Transmit Interrupt Mask
pub const UART_IM_RXIM: usize = 0x00000010; // UART Receive Interrupt Mask
pub const UART_IM_CTSMIM: usize = 0x00000002; // UART Clear to Send Modem Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
// *****************************************************************************
pub const UART_RIS_LME5RIS: usize = 0x00008000; // LIN Mode Edge 5 Raw Interrupt Status
pub const UART_RIS_LME1RIS: usize = 0x00004000; // LIN Mode Edge 1 Raw Interrupt Status
pub const UART_RIS_LMSBRIS: usize = 0x00002000; // LIN Mode Sync Break Raw Interrupt Status
pub const UART_RIS_9BITRIS: usize = 0x00001000; // 9-Bit Mode Raw Interrupt Status
pub const UART_RIS_OERIS: usize = 0x00000400; // UART Overrun Error Raw Interrupt Status
pub const UART_RIS_BERIS: usize = 0x00000200; // UART Break Error Raw Interrupt Status
pub const UART_RIS_PERIS: usize = 0x00000100; // UART Parity Error Raw Interrupt Status
pub const UART_RIS_FERIS: usize = 0x00000080; // UART Framing Error Raw Interrupt Status
pub const UART_RIS_RTRIS: usize = 0x00000040; // UART Receive Time-Out Raw Interrupt Status
pub const UART_RIS_TXRIS: usize = 0x00000020; // UART Transmit Raw Interrupt Status
pub const UART_RIS_RXRIS: usize = 0x00000010; // UART Receive Raw Interrupt Status
pub const UART_RIS_CTSRIS: usize = 0x00000002; // UART Clear to Send Modem Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
// *****************************************************************************
pub const UART_MIS_LME5MIS: usize = 0x00008000; // LIN Mode Edge 5 Masked Interrupt Status
pub const UART_MIS_LME1MIS: usize = 0x00004000; // LIN Mode Edge 1 Masked Interrupt Status
pub const UART_MIS_LMSBMIS: usize = 0x00002000; // LIN Mode Sync Break Masked Interrupt Status
pub const UART_MIS_9BITMIS: usize = 0x00001000; // 9-Bit Mode Masked Interrupt Status
pub const UART_MIS_OEMIS: usize = 0x00000400; // UART Overrun Error Masked Interrupt Status
pub const UART_MIS_BEMIS: usize = 0x00000200; // UART Break Error Masked Interrupt Status
pub const UART_MIS_PEMIS: usize = 0x00000100; // UART Parity Error Masked Interrupt Status
pub const UART_MIS_FEMIS: usize = 0x00000080; // UART Framing Error Masked Interrupt Status
pub const UART_MIS_RTMIS: usize = 0x00000040; // UART Receive Time-Out Masked Interrupt Status
pub const UART_MIS_TXMIS: usize = 0x00000020; // UART Transmit Masked Interrupt Status
pub const UART_MIS_RXMIS: usize = 0x00000010; // UART Receive Masked Interrupt Status
pub const UART_MIS_CTSMIS: usize = 0x00000002; // UART Clear to Send Modem Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
// *****************************************************************************
pub const UART_ICR_LME5IC: usize = 0x00008000; // LIN Mode Edge 5 Interrupt Clear
pub const UART_ICR_LME1IC: usize = 0x00004000; // LIN Mode Edge 1 Interrupt Clear
pub const UART_ICR_LMSBIC: usize = 0x00002000; // LIN Mode Sync Break Interrupt Clear
pub const UART_ICR_9BITIC: usize = 0x00001000; // 9-Bit Mode Interrupt Clear
pub const UART_ICR_OEIC: usize = 0x00000400; // Overrun Error Interrupt Clear
pub const UART_ICR_BEIC: usize = 0x00000200; // Break Error Interrupt Clear
pub const UART_ICR_PEIC: usize = 0x00000100; // Parity Error Interrupt Clear
pub const UART_ICR_FEIC: usize = 0x00000080; // Framing Error Interrupt Clear
pub const UART_ICR_RTIC: usize = 0x00000040; // Receive Time-Out Interrupt Clear
pub const UART_ICR_TXIC: usize = 0x00000020; // Transmit Interrupt Clear
pub const UART_ICR_RXIC: usize = 0x00000010; // Receive Interrupt Clear
pub const UART_ICR_CTSMIC: usize = 0x00000002; // UART Clear to Send Modem Interrupt Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
// *****************************************************************************
pub const UART_DMACTL_DMAERR: usize = 0x00000004; // DMA on Error
pub const UART_DMACTL_TXDMAE: usize = 0x00000002; // Transmit DMA Enable
pub const UART_DMACTL_RXDMAE: usize = 0x00000001; // Receive DMA Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCTL register.
//
// *****************************************************************************
pub const UART_LCTL_BLEN_M: usize = 0x00000030; // Sync Break Length
pub const UART_LCTL_BLEN_13T: usize = 0x00000000; // Sync break length is 13T bits (default)
pub const UART_LCTL_BLEN_14T: usize = 0x00000010; // Sync break length is 14T bits
pub const UART_LCTL_BLEN_15T: usize = 0x00000020; // Sync break length is 15T bits
pub const UART_LCTL_BLEN_16T: usize = 0x00000030; // Sync break length is 16T bits
pub const UART_LCTL_MASTER: usize = 0x00000001; // LIN Master Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LSS register.
//
// *****************************************************************************
pub const UART_LSS_TSS_M: usize = 0x0000FFFF; // Timer Snap Shot
pub const UART_LSS_TSS_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LTIM register.
//
// *****************************************************************************
pub const UART_LTIM_TIMER_M: usize = 0x0000FFFF; // Timer Value
pub const UART_LTIM_TIMER_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITADDR
// register.
//
// *****************************************************************************
pub const UART_9BITADDR_9BITEN: usize = 0x00008000; // Enable 9-Bit Mode
pub const UART_9BITADDR_ADDR_M: usize = 0x000000FF; // Self Address for 9-Bit Mode
pub const UART_9BITADDR_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITAMASK
// register.
//
// *****************************************************************************
pub const UART_9BITAMASK_MASK_M: usize = 0x000000FF; // Self Address Mask for 9-Bit Mode
pub const UART_9BITAMASK_MASK_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_PP register.
//
// *****************************************************************************
pub const UART_PP_NB: usize = 0x00000002; // 9-Bit Support
pub const UART_PP_SC: usize = 0x00000001; // Smart Card Support

// *****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CC register.
//
// *****************************************************************************
pub const UART_CC_CS_M: usize = 0x0000000F; // UART Baud Clock Source
pub const UART_CC_CS_SYSCLK: usize = 0x00000000; // The system clock (default)
pub const UART_CC_CS_PIOSC: usize = 0x00000005; // PIOSC

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MSA register.
//
// *****************************************************************************
pub const I2C_MSA_SA_M: usize = 0x000000FE; // I2C Slave Address
pub const I2C_MSA_RS: usize = 0x00000001; // Receive not send
pub const I2C_MSA_SA_S: usize = 1;

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR register.
//
// *****************************************************************************
pub const I2C_SOAR_OAR_M: usize = 0x0000007F; // I2C Slave Own Address
pub const I2C_SOAR_OAR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SCSR register.
//
// *****************************************************************************
pub const I2C_SCSR_OAR2SEL: usize = 0x00000008; // OAR2 Address Matched
pub const I2C_SCSR_FBR: usize = 0x00000004; // First Byte Received
pub const I2C_SCSR_TREQ: usize = 0x00000002; // Transmit Request
pub const I2C_SCSR_DA: usize = 0x00000001; // Device Active
pub const I2C_SCSR_RREQ: usize = 0x00000001; // Receive Request

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCS register.
//
// *****************************************************************************
pub const I2C_MCS_CLKTO: usize = 0x00000080; // Clock Timeout Error
pub const I2C_MCS_BUSBSY: usize = 0x00000040; // Bus Busy
pub const I2C_MCS_IDLE: usize = 0x00000020; // I2C Idle
pub const I2C_MCS_ARBLST: usize = 0x00000010; // Arbitration Lost
pub const I2C_MCS_HS: usize = 0x00000010; // High-Speed Enable
pub const I2C_MCS_ACK: usize = 0x00000008; // Data Acknowledge Enable
pub const I2C_MCS_DATACK: usize = 0x00000008; // Acknowledge Data
pub const I2C_MCS_ADRACK: usize = 0x00000004; // Acknowledge Address
pub const I2C_MCS_STOP: usize = 0x00000004; // Generate STOP
pub const I2C_MCS_ERROR: usize = 0x00000002; // Error
pub const I2C_MCS_START: usize = 0x00000002; // Generate START
pub const I2C_MCS_RUN: usize = 0x00000001; // I2C Master Enable
pub const I2C_MCS_BUSY: usize = 0x00000001; // I2C Busy

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SDR register.
//
// *****************************************************************************
pub const I2C_SDR_DATA_M: usize = 0x000000FF; // Data for Transfer
pub const I2C_SDR_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MDR register.
//
// *****************************************************************************
pub const I2C_MDR_DATA_M: usize = 0x000000FF; // Data Transferred
pub const I2C_MDR_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MTPR register.
//
// *****************************************************************************
pub const I2C_MTPR_HS: usize = 0x00000080; // High-Speed Enable
pub const I2C_MTPR_TPR_M: usize = 0x0000007F; // SCL Clock Period
pub const I2C_MTPR_TPR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SIMR register.
//
// *****************************************************************************
pub const I2C_SIMR_STOPIM: usize = 0x00000004; // Stop Condition Interrupt Mask
pub const I2C_SIMR_STARTIM: usize = 0x00000002; // Start Condition Interrupt Mask
pub const I2C_SIMR_DATAIM: usize = 0x00000001; // Data Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SRIS register.
//
// *****************************************************************************
pub const I2C_SRIS_STOPRIS: usize = 0x00000004; // Stop Condition Raw Interrupt Status
pub const I2C_SRIS_STARTRIS: usize = 0x00000002; // Start Condition Raw Interrupt Status
pub const I2C_SRIS_DATARIS: usize = 0x00000001; // Data Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MIMR register.
//
// *****************************************************************************
pub const I2C_MIMR_CLKIM: usize = 0x00000002; // Clock Timeout Interrupt Mask
pub const I2C_MIMR_IM: usize = 0x00000001; // Master Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MRIS register.
//
// *****************************************************************************
pub const I2C_MRIS_CLKRIS: usize = 0x00000002; // Clock Timeout Raw Interrupt Status
pub const I2C_MRIS_RIS: usize = 0x00000001; // Master Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SMIS register.
//
// *****************************************************************************
pub const I2C_SMIS_STOPMIS: usize = 0x00000004; // Stop Condition Masked Interrupt Status
pub const I2C_SMIS_STARTMIS: usize = 0x00000002; // Start Condition Masked Interrupt Status
pub const I2C_SMIS_DATAMIS: usize = 0x00000001; // Data Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SICR register.
//
// *****************************************************************************
pub const I2C_SICR_STOPIC: usize = 0x00000004; // Stop Condition Interrupt Clear
pub const I2C_SICR_STARTIC: usize = 0x00000002; // Start Condition Interrupt Clear
pub const I2C_SICR_DATAIC: usize = 0x00000001; // Data Interrupt Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MMIS register.
//
// *****************************************************************************
pub const I2C_MMIS_CLKMIS: usize = 0x00000002; // Clock Timeout Masked Interrupt Status
pub const I2C_MMIS_MIS: usize = 0x00000001; // Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MICR register.
//
// *****************************************************************************
pub const I2C_MICR_CLKIC: usize = 0x00000002; // Clock Timeout Interrupt Clear
pub const I2C_MICR_IC: usize = 0x00000001; // Master Interrupt Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SOAR2 register.
//
// *****************************************************************************
pub const I2C_SOAR2_OAR2EN: usize = 0x00000080; // I2C Slave Own Address 2 Enable
pub const I2C_SOAR2_OAR2_M: usize = 0x0000007F; // I2C Slave Own Address 2
pub const I2C_SOAR2_OAR2_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCR register.
//
// *****************************************************************************
pub const I2C_MCR_SFE: usize = 0x00000020; // I2C Slave Function Enable
pub const I2C_MCR_MFE: usize = 0x00000010; // I2C Master Function Enable
pub const I2C_MCR_LPBK: usize = 0x00000001; // I2C Loopback

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_SACKCTL register.
//
// *****************************************************************************
pub const I2C_SACKCTL_ACKOVAL: usize = 0x00000002; // I2C Slave ACK Override Value
pub const I2C_SACKCTL_ACKOEN: usize = 0x00000001; // I2C Slave ACK Override Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MCLKOCNT register.
//
// *****************************************************************************
pub const I2C_MCLKOCNT_CNTL_M: usize = 0x000000FF; // I2C Master Count
pub const I2C_MCLKOCNT_CNTL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_MBMON register.
//
// *****************************************************************************
pub const I2C_MBMON_SDA: usize = 0x00000002; // I2C SDA Status
pub const I2C_MBMON_SCL: usize = 0x00000001; // I2C SCL Status

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PP register.
//
// *****************************************************************************
pub const I2C_PP_HS: usize = 0x00000001; // High-Speed Capable

// *****************************************************************************
//
// The following are defines for the bit fields in the I2C_O_PC register.
//
// *****************************************************************************
pub const I2C_PC_HS: usize = 0x00000001; // High-Speed Capable

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CFG register.
//
// *****************************************************************************
pub const TIMER_CFG_M: usize = 0x00000007; // GPTM Configuration
pub const TIMER_CFG_32_BIT_TIMER: usize = 0x00000000; // 32-bit timer configuration
pub const TIMER_CFG_32_BIT_RTC: usize = 0x00000001; // 32-bit real-time clock (RTC) counter configuration
pub const TIMER_CFG_16_BIT: usize = 0x00000004; // 16-bit timer configuration. The function is controlled by bits 1:0 of GPTMTAMR and GPTMTBMR

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMR register.
//
// *****************************************************************************
pub const TIMER_TAMR_TAPLO: usize = 0x00000800; // GPTM Timer A PWM Legacy Operation
pub const TIMER_TAMR_TAMRSU: usize = 0x00000400; // GPTM Timer A Match Register Update
pub const TIMER_TAMR_TAPWMIE: usize = 0x00000200; // GPTM Timer A PWM Interrupt Enable
pub const TIMER_TAMR_TAILD: usize = 0x00000100; // GPTM Timer A Interval Load Write
pub const TIMER_TAMR_TASNAPS: usize = 0x00000080; // GPTM Timer A Snap-Shot Mode
pub const TIMER_TAMR_TAWOT: usize = 0x00000040; // GPTM Timer A Wait-on-Trigger
pub const TIMER_TAMR_TAMIE: usize = 0x00000020; // GPTM Timer A Match Interrupt Enable
pub const TIMER_TAMR_TACDIR: usize = 0x00000010; // GPTM Timer A Count Direction
pub const TIMER_TAMR_TAAMS: usize = 0x00000008; // GPTM Timer A Alternate Mode Select
pub const TIMER_TAMR_TACMR: usize = 0x00000004; // GPTM Timer A Capture Mode
pub const TIMER_TAMR_TAMR_M: usize = 0x00000003; // GPTM Timer A Mode
pub const TIMER_TAMR_TAMR_1_SHOT: usize = 0x00000001; // One-Shot Timer mode
pub const TIMER_TAMR_TAMR_PERIOD: usize = 0x00000002; // Periodic Timer mode
pub const TIMER_TAMR_TAMR_CAP: usize = 0x00000003; // Capture mode

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMR register.
//
// *****************************************************************************
pub const TIMER_TBMR_TBPLO: usize = 0x00000800; // GPTM Timer B PWM Legacy Operation
pub const TIMER_TBMR_TBMRSU: usize = 0x00000400; // GPTM Timer B Match Register Update
pub const TIMER_TBMR_TBPWMIE: usize = 0x00000200; // GPTM Timer B PWM Interrupt Enable
pub const TIMER_TBMR_TBILD: usize = 0x00000100; // GPTM Timer B Interval Load Write
pub const TIMER_TBMR_TBSNAPS: usize = 0x00000080; // GPTM Timer B Snap-Shot Mode
pub const TIMER_TBMR_TBWOT: usize = 0x00000040; // GPTM Timer B Wait-on-Trigger
pub const TIMER_TBMR_TBMIE: usize = 0x00000020; // GPTM Timer B Match Interrupt Enable
pub const TIMER_TBMR_TBCDIR: usize = 0x00000010; // GPTM Timer B Count Direction
pub const TIMER_TBMR_TBAMS: usize = 0x00000008; // GPTM Timer B Alternate Mode Select
pub const TIMER_TBMR_TBCMR: usize = 0x00000004; // GPTM Timer B Capture Mode
pub const TIMER_TBMR_TBMR_M: usize = 0x00000003; // GPTM Timer B Mode
pub const TIMER_TBMR_TBMR_1_SHOT: usize = 0x00000001; // One-Shot Timer mode
pub const TIMER_TBMR_TBMR_PERIOD: usize = 0x00000002; // Periodic Timer mode
pub const TIMER_TBMR_TBMR_CAP: usize = 0x00000003; // Capture mode

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_CTL register.
//
// *****************************************************************************
pub const TIMER_CTL_TBPWML: usize = 0x00004000; // GPTM Timer B PWM Output Level
pub const TIMER_CTL_TBOTE: usize = 0x00002000; // GPTM Timer B Output Trigger Enable
pub const TIMER_CTL_TBEVENT_M: usize = 0x00000C00; // GPTM Timer B Event Mode
pub const TIMER_CTL_TBEVENT_POS: usize = 0x00000000; // Positive edge
pub const TIMER_CTL_TBEVENT_NEG: usize = 0x00000400; // Negative edge
pub const TIMER_CTL_TBEVENT_BOTH: usize = 0x00000C00; // Both edges
pub const TIMER_CTL_TBSTALL: usize = 0x00000200; // GPTM Timer B Stall Enable
pub const TIMER_CTL_TBEN: usize = 0x00000100; // GPTM Timer B Enable
pub const TIMER_CTL_TAPWML: usize = 0x00000040; // GPTM Timer A PWM Output Level
pub const TIMER_CTL_TAOTE: usize = 0x00000020; // GPTM Timer A Output Trigger Enable
pub const TIMER_CTL_RTCEN: usize = 0x00000010; // GPTM RTC Stall Enable
pub const TIMER_CTL_TAEVENT_M: usize = 0x0000000C; // GPTM Timer A Event Mode
pub const TIMER_CTL_TAEVENT_POS: usize = 0x00000000; // Positive edge
pub const TIMER_CTL_TAEVENT_NEG: usize = 0x00000004; // Negative edge
pub const TIMER_CTL_TAEVENT_BOTH: usize = 0x0000000C; // Both edges
pub const TIMER_CTL_TASTALL: usize = 0x00000002; // GPTM Timer A Stall Enable
pub const TIMER_CTL_TAEN: usize = 0x00000001; // GPTM Timer A Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_SYNC register.
//
// *****************************************************************************
pub const TIMER_SYNC_SYNCWT5_M: usize = 0x00C00000; // Synchronize GPTM 32/64-Bit Timer 5
pub const TIMER_SYNC_SYNCWT5_NONE: usize = 0x00000000; // GPTM 32/64-Bit Timer 5 is not affected
pub const TIMER_SYNC_SYNCWT5_TA: usize = 0x00400000; // A timeout event for Timer A of GPTM 32/64-Bit Timer 5 is triggered
pub const TIMER_SYNC_SYNCWT5_TB: usize = 0x00800000; // A timeout event for Timer B of GPTM 32/64-Bit Timer 5 is triggered
pub const TIMER_SYNC_SYNCWT5_TATB: usize = 0x00C00000; // A timeout event for both Timer A and Timer B of GPTM 32/64-Bit Timer 5 is triggered
pub const TIMER_SYNC_SYNCWT4_M: usize = 0x00300000; // Synchronize GPTM 32/64-Bit Timer 4
pub const TIMER_SYNC_SYNCWT4_NONE: usize = 0x00000000; // GPTM 32/64-Bit Timer 4 is not affected
pub const TIMER_SYNC_SYNCWT4_TA: usize = 0x00100000; // A timeout event for Timer A of GPTM 32/64-Bit Timer 4 is triggered
pub const TIMER_SYNC_SYNCWT4_TB: usize = 0x00200000; // A timeout event for Timer B of GPTM 32/64-Bit Timer 4 is triggered
pub const TIMER_SYNC_SYNCWT4_TATB: usize = 0x00300000; // A timeout event for both Timer A and Timer B of GPTM 32/64-Bit Timer 4 is triggered
pub const TIMER_SYNC_SYNCWT3_M: usize = 0x000C0000; // Synchronize GPTM 32/64-Bit Timer 3
pub const TIMER_SYNC_SYNCWT3_NONE: usize = 0x00000000; // GPTM 32/64-Bit Timer 3 is not affected
pub const TIMER_SYNC_SYNCWT3_TA: usize = 0x00040000; // A timeout event for Timer A of GPTM 32/64-Bit Timer 3 is triggered
pub const TIMER_SYNC_SYNCWT3_TB: usize = 0x00080000; // A timeout event for Timer B of GPTM 32/64-Bit Timer 3 is triggered
pub const TIMER_SYNC_SYNCWT3_TATB: usize = 0x000C0000; // A timeout event for both Timer A and Timer B of GPTM 32/64-Bit Timer 3 is triggered
pub const TIMER_SYNC_SYNCWT2_M: usize = 0x00030000; // Synchronize GPTM 32/64-Bit Timer 2
pub const TIMER_SYNC_SYNCWT2_NONE: usize = 0x00000000; // GPTM 32/64-Bit Timer 2 is not affected
pub const TIMER_SYNC_SYNCWT2_TA: usize = 0x00010000; // A timeout event for Timer A of GPTM 32/64-Bit Timer 2 is triggered
pub const TIMER_SYNC_SYNCWT2_TB: usize = 0x00020000; // A timeout event for Timer B of GPTM 32/64-Bit Timer 2 is triggered
pub const TIMER_SYNC_SYNCWT2_TATB: usize = 0x00030000; // A timeout event for both Timer A and Timer B of GPTM 32/64-Bit Timer 2 is triggered
pub const TIMER_SYNC_SYNCWT1_M: usize = 0x0000C000; // Synchronize GPTM 32/64-Bit Timer 1
pub const TIMER_SYNC_SYNCWT1_NONE: usize = 0x00000000; // GPTM 32/64-Bit Timer 1 is not affected
pub const TIMER_SYNC_SYNCWT1_TA: usize = 0x00004000; // A timeout event for Timer A of GPTM 32/64-Bit Timer 1 is triggered
pub const TIMER_SYNC_SYNCWT1_TB: usize = 0x00008000; // A timeout event for Timer B of GPTM 32/64-Bit Timer 1 is triggered
pub const TIMER_SYNC_SYNCWT1_TATB: usize = 0x0000C000; // A timeout event for both Timer A and Timer B of GPTM 32/64-Bit Timer 1 is triggered
pub const TIMER_SYNC_SYNCWT0_M: usize = 0x00003000; // Synchronize GPTM 32/64-Bit Timer 0
pub const TIMER_SYNC_SYNCWT0_NONE: usize = 0x00000000; // GPTM 32/64-Bit Timer 0 is not affected
pub const TIMER_SYNC_SYNCWT0_TA: usize = 0x00001000; // A timeout event for Timer A of GPTM 32/64-Bit Timer 0 is triggered
pub const TIMER_SYNC_SYNCWT0_TB: usize = 0x00002000; // A timeout event for Timer B of GPTM 32/64-Bit Timer 0 is triggered
pub const TIMER_SYNC_SYNCWT0_TATB: usize = 0x00003000; // A timeout event for both Timer A and Timer B of GPTM 32/64-Bit Timer 0 is triggered
pub const TIMER_SYNC_SYNCT5_M: usize = 0x00000C00; // Synchronize GPTM 16/32-Bit Timer 5
pub const TIMER_SYNC_SYNCT5_NONE: usize = 0x00000000; // GPTM 16/32-Bit Timer 5 is not affected
pub const TIMER_SYNC_SYNCT5_TA: usize = 0x00000400; // A timeout event for Timer A of GPTM 16/32-Bit Timer 5 is triggered
pub const TIMER_SYNC_SYNCT5_TB: usize = 0x00000800; // A timeout event for Timer B of GPTM 16/32-Bit Timer 5 is triggered
pub const TIMER_SYNC_SYNCT5_TATB: usize = 0x00000C00; // A timeout event for both Timer A and Timer B of GPTM 16/32-Bit Timer 5 is triggered
pub const TIMER_SYNC_SYNCT4_M: usize = 0x00000300; // Synchronize GPTM 16/32-Bit Timer 4
pub const TIMER_SYNC_SYNCT4_NONE: usize = 0x00000000; // GPTM 16/32-Bit Timer 4 is not affected
pub const TIMER_SYNC_SYNCT4_TA: usize = 0x00000100; // A timeout event for Timer A of GPTM 16/32-Bit Timer 4 is triggered
pub const TIMER_SYNC_SYNCT4_TB: usize = 0x00000200; // A timeout event for Timer B of GPTM 16/32-Bit Timer 4 is triggered
pub const TIMER_SYNC_SYNCT4_TATB: usize = 0x00000300; // A timeout event for both Timer A and Timer B of GPTM 16/32-Bit Timer 4 is triggered
pub const TIMER_SYNC_SYNCT3_M: usize = 0x000000C0; // Synchronize GPTM 16/32-Bit Timer 3
pub const TIMER_SYNC_SYNCT3_NONE: usize = 0x00000000; // GPTM 16/32-Bit Timer 3 is not affected
pub const TIMER_SYNC_SYNCT3_TA: usize = 0x00000040; // A timeout event for Timer A of GPTM 16/32-Bit Timer 3 is triggered
pub const TIMER_SYNC_SYNCT3_TB: usize = 0x00000080; // A timeout event for Timer B of GPTM 16/32-Bit Timer 3 is triggered
pub const TIMER_SYNC_SYNCT3_TATB: usize = 0x000000C0; // A timeout event for both Timer A and Timer B of GPTM 16/32-Bit Timer 3 is triggered
pub const TIMER_SYNC_SYNCT2_M: usize = 0x00000030; // Synchronize GPTM 16/32-Bit Timer 2
pub const TIMER_SYNC_SYNCT2_NONE: usize = 0x00000000; // GPTM 16/32-Bit Timer 2 is not affected
pub const TIMER_SYNC_SYNCT2_TA: usize = 0x00000010; // A timeout event for Timer A of GPTM 16/32-Bit Timer 2 is triggered
pub const TIMER_SYNC_SYNCT2_TB: usize = 0x00000020; // A timeout event for Timer B of GPTM 16/32-Bit Timer 2 is triggered
pub const TIMER_SYNC_SYNCT2_TATB: usize = 0x00000030; // A timeout event for both Timer A and Timer B of GPTM 16/32-Bit Timer 2 is triggered
pub const TIMER_SYNC_SYNCT1_M: usize = 0x0000000C; // Synchronize GPTM 16/32-Bit Timer 1
pub const TIMER_SYNC_SYNCT1_NONE: usize = 0x00000000; // GPTM 16/32-Bit Timer 1 is not affected
pub const TIMER_SYNC_SYNCT1_TA: usize = 0x00000004; // A timeout event for Timer A of GPTM 16/32-Bit Timer 1 is triggered
pub const TIMER_SYNC_SYNCT1_TB: usize = 0x00000008; // A timeout event for Timer B of GPTM 16/32-Bit Timer 1 is triggered
pub const TIMER_SYNC_SYNCT1_TATB: usize = 0x0000000C; // A timeout event for both Timer A and Timer B of GPTM 16/32-Bit Timer 1 is triggered
pub const TIMER_SYNC_SYNCT0_M: usize = 0x00000003; // Synchronize GPTM 16/32-Bit Timer 0
pub const TIMER_SYNC_SYNCT0_NONE: usize = 0x00000000; // GPTM 16/32-Bit Timer 0 is not affected
pub const TIMER_SYNC_SYNCT0_TA: usize = 0x00000001; // A timeout event for Timer A of GPTM 16/32-Bit Timer 0 is triggered
pub const TIMER_SYNC_SYNCT0_TB: usize = 0x00000002; // A timeout event for Timer B of GPTM 16/32-Bit Timer 0 is triggered
pub const TIMER_SYNC_SYNCT0_TATB: usize = 0x00000003; // A timeout event for both Timer A and Timer B of GPTM 16/32-Bit Timer 0 is triggered

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_IMR register.
//
// *****************************************************************************
pub const TIMER_IMR_WUEIM: usize = 0x00010000; // GPTM Write Update Error Interrupt Mask
pub const TIMER_IMR_TBMIM: usize = 0x00000800; // GPTM Timer B Match Interrupt Mask
pub const TIMER_IMR_CBEIM: usize = 0x00000400; // GPTM Timer B Capture Mode Event Interrupt Mask
pub const TIMER_IMR_CBMIM: usize = 0x00000200; // GPTM Timer B Capture Mode Match Interrupt Mask
pub const TIMER_IMR_TBTOIM: usize = 0x00000100; // GPTM Timer B Time-Out Interrupt Mask
pub const TIMER_IMR_TAMIM: usize = 0x00000010; // GPTM Timer A Match Interrupt Mask
pub const TIMER_IMR_RTCIM: usize = 0x00000008; // GPTM RTC Interrupt Mask
pub const TIMER_IMR_CAEIM: usize = 0x00000004; // GPTM Timer A Capture Mode Event Interrupt Mask
pub const TIMER_IMR_CAMIM: usize = 0x00000002; // GPTM Timer A Capture Mode Match Interrupt Mask
pub const TIMER_IMR_TATOIM: usize = 0x00000001; // GPTM Timer A Time-Out Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RIS register.
//
// *****************************************************************************
pub const TIMER_RIS_WUERIS: usize = 0x00010000; // GPTM Write Update Error Raw Interrupt
pub const TIMER_RIS_TBMRIS: usize = 0x00000800; // GPTM Timer B Match Raw Interrupt
pub const TIMER_RIS_CBERIS: usize = 0x00000400; // GPTM Timer B Capture Mode Event Raw Interrupt
pub const TIMER_RIS_CBMRIS: usize = 0x00000200; // GPTM Timer B Capture Mode Match Raw Interrupt
pub const TIMER_RIS_TBTORIS: usize = 0x00000100; // GPTM Timer B Time-Out Raw Interrupt
pub const TIMER_RIS_TAMRIS: usize = 0x00000010; // GPTM Timer A Match Raw Interrupt
pub const TIMER_RIS_RTCRIS: usize = 0x00000008; // GPTM RTC Raw Interrupt
pub const TIMER_RIS_CAERIS: usize = 0x00000004; // GPTM Timer A Capture Mode Event Raw Interrupt
pub const TIMER_RIS_CAMRIS: usize = 0x00000002; // GPTM Timer A Capture Mode Match Raw Interrupt
pub const TIMER_RIS_TATORIS: usize = 0x00000001; // GPTM Timer A Time-Out Raw Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_MIS register.
//
// *****************************************************************************
pub const TIMER_MIS_WUEMIS: usize = 0x00010000; // GPTM Write Update Error Masked Interrupt
pub const TIMER_MIS_TBMMIS: usize = 0x00000800; // GPTM Timer B Match Masked Interrupt
pub const TIMER_MIS_CBEMIS: usize = 0x00000400; // GPTM Timer B Capture Mode Event Masked Interrupt
pub const TIMER_MIS_CBMMIS: usize = 0x00000200; // GPTM Timer B Capture Mode Match Masked Interrupt
pub const TIMER_MIS_TBTOMIS: usize = 0x00000100; // GPTM Timer B Time-Out Masked Interrupt
pub const TIMER_MIS_TAMMIS: usize = 0x00000010; // GPTM Timer A Match Masked Interrupt
pub const TIMER_MIS_RTCMIS: usize = 0x00000008; // GPTM RTC Masked Interrupt
pub const TIMER_MIS_CAEMIS: usize = 0x00000004; // GPTM Timer A Capture Mode Event Masked Interrupt
pub const TIMER_MIS_CAMMIS: usize = 0x00000002; // GPTM Timer A Capture Mode Match Masked Interrupt
pub const TIMER_MIS_TATOMIS: usize = 0x00000001; // GPTM Timer A Time-Out Masked Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_ICR register.
//
// *****************************************************************************
pub const TIMER_ICR_WUECINT: usize = 0x00010000; // 32/64-Bit GPTM Write Update Error Interrupt Clear
pub const TIMER_ICR_TBMCINT: usize = 0x00000800; // GPTM Timer B Match Interrupt Clear
pub const TIMER_ICR_CBECINT: usize = 0x00000400; // GPTM Timer B Capture Mode Event Interrupt Clear
pub const TIMER_ICR_CBMCINT: usize = 0x00000200; // GPTM Timer B Capture Mode Match Interrupt Clear
pub const TIMER_ICR_TBTOCINT: usize = 0x00000100; // GPTM Timer B Time-Out Interrupt Clear
pub const TIMER_ICR_TAMCINT: usize = 0x00000010; // GPTM Timer A Match Interrupt Clear
pub const TIMER_ICR_RTCCINT: usize = 0x00000008; // GPTM RTC Interrupt Clear
pub const TIMER_ICR_CAECINT: usize = 0x00000004; // GPTM Timer A Capture Mode Event Interrupt Clear
pub const TIMER_ICR_CAMCINT: usize = 0x00000002; // GPTM Timer A Capture Mode Match Interrupt Clear
pub const TIMER_ICR_TATOCINT: usize = 0x00000001; // GPTM Timer A Time-Out Raw Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAILR register.
//
// *****************************************************************************
pub const TIMER_TAILR_M: usize = 0xFFFFFFFF; // GPTM Timer A Interval Load Register
pub const TIMER_TAILR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBILR register.
//
// *****************************************************************************
pub const TIMER_TBILR_M: usize = 0xFFFFFFFF; // GPTM Timer B Interval Load Register
pub const TIMER_TBILR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAMATCHR
// register.
//
// *****************************************************************************
pub const TIMER_TAMATCHR_TAMR_M: usize = 0xFFFFFFFF; // GPTM Timer A Match Register
pub const TIMER_TAMATCHR_TAMR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBMATCHR
// register.
//
// *****************************************************************************
pub const TIMER_TBMATCHR_TBMR_M: usize = 0xFFFFFFFF; // GPTM Timer B Match Register
pub const TIMER_TBMATCHR_TBMR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPR register.
//
// *****************************************************************************
pub const TIMER_TAPR_TAPSRH_M: usize = 0x0000FF00; // GPTM Timer A Prescale High Byte
pub const TIMER_TAPR_TAPSR_M: usize = 0x000000FF; // GPTM Timer A Prescale
pub const TIMER_TAPR_TAPSRH_S: usize = 8;
pub const TIMER_TAPR_TAPSR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPR register.
//
// *****************************************************************************
pub const TIMER_TBPR_TBPSRH_M: usize = 0x0000FF00; // GPTM Timer B Prescale High Byte
pub const TIMER_TBPR_TBPSR_M: usize = 0x000000FF; // GPTM Timer B Prescale
pub const TIMER_TBPR_TBPSRH_S: usize = 8;
pub const TIMER_TBPR_TBPSR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPMR register.
//
// *****************************************************************************
pub const TIMER_TAPMR_TAPSMRH_M: usize = 0x0000FF00; // GPTM Timer A Prescale Match High Byte
pub const TIMER_TAPMR_TAPSMR_M: usize = 0x000000FF; // GPTM TimerA Prescale Match
pub const TIMER_TAPMR_TAPSMRH_S: usize = 8;
pub const TIMER_TAPMR_TAPSMR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPMR register.
//
// *****************************************************************************
pub const TIMER_TBPMR_TBPSMRH_M: usize = 0x0000FF00; // GPTM Timer B Prescale Match High Byte
pub const TIMER_TBPMR_TBPSMR_M: usize = 0x000000FF; // GPTM TimerB Prescale Match
pub const TIMER_TBPMR_TBPSMRH_S: usize = 8;
pub const TIMER_TBPMR_TBPSMR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAR register.
//
// *****************************************************************************
pub const TIMER_TAR_M: usize = 0xFFFFFFFF; // GPTM Timer A Register
pub const TIMER_TAR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBR register.
//
// *****************************************************************************
pub const TIMER_TBR_M: usize = 0xFFFFFFFF; // GPTM Timer B Register
pub const TIMER_TBR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAV register.
//
// *****************************************************************************
pub const TIMER_TAV_M: usize = 0xFFFFFFFF; // GPTM Timer A Value
pub const TIMER_TAV_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBV register.
//
// *****************************************************************************
pub const TIMER_TBV_M: usize = 0xFFFFFFFF; // GPTM Timer B Value
pub const TIMER_TBV_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_RTCPD register.
//
// *****************************************************************************
pub const TIMER_RTCPD_RTCPD_M: usize = 0x0000FFFF; // RTC Predivide Counter Value
pub const TIMER_RTCPD_RTCPD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPS register.
//
// *****************************************************************************
pub const TIMER_TAPS_PSS_M: usize = 0x0000FFFF; // GPTM Timer A Prescaler Snapshot
pub const TIMER_TAPS_PSS_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPS register.
//
// *****************************************************************************
pub const TIMER_TBPS_PSS_M: usize = 0x0000FFFF; // GPTM Timer A Prescaler Value
pub const TIMER_TBPS_PSS_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TAPV register.
//
// *****************************************************************************
pub const TIMER_TAPV_PSV_M: usize = 0x0000FFFF; // GPTM Timer A Prescaler Value
pub const TIMER_TAPV_PSV_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_TBPV register.
//
// *****************************************************************************
pub const TIMER_TBPV_PSV_M: usize = 0x0000FFFF; // GPTM Timer B Prescaler Value
pub const TIMER_TBPV_PSV_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the TIMER_O_PP register.
//
// *****************************************************************************
pub const TIMER_PP_SIZE_M: usize = 0x0000000F; // Count Size
pub const TIMER_PP_SIZE_16: usize = 0x00000000; // Timer A and Timer B counters are 16 bits each with an 8-bit prescale counter
pub const TIMER_PP_SIZE_32: usize = 0x00000001; // Timer A and Timer B counters are 32 bits each with a 16-bit prescale counter

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ACTSS register.
//
// *****************************************************************************
pub const ADC_ACTSS_ASEN3: usize = 0x00000008; // ADC SS3 Enable
pub const ADC_ACTSS_ASEN2: usize = 0x00000004; // ADC SS2 Enable
pub const ADC_ACTSS_ASEN1: usize = 0x00000002; // ADC SS1 Enable
pub const ADC_ACTSS_ASEN0: usize = 0x00000001; // ADC SS0 Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_RIS register.
//
// *****************************************************************************
pub const ADC_RIS_INRDC: usize = 0x00010000; // Digital Comparator Raw Interrupt Status
pub const ADC_RIS_INR3: usize = 0x00000008; // SS3 Raw Interrupt Status
pub const ADC_RIS_INR2: usize = 0x00000004; // SS2 Raw Interrupt Status
pub const ADC_RIS_INR1: usize = 0x00000002; // SS1 Raw Interrupt Status
pub const ADC_RIS_INR0: usize = 0x00000001; // SS0 Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_IM register.
//
// *****************************************************************************
pub const ADC_IM_DCONSS3: usize = 0x00080000; // Digital Comparator Interrupt on SS3
pub const ADC_IM_DCONSS2: usize = 0x00040000; // Digital Comparator Interrupt on SS2
pub const ADC_IM_DCONSS1: usize = 0x00020000; // Digital Comparator Interrupt on SS1
pub const ADC_IM_DCONSS0: usize = 0x00010000; // Digital Comparator Interrupt on SS0
pub const ADC_IM_MASK3: usize = 0x00000008; // SS3 Interrupt Mask
pub const ADC_IM_MASK2: usize = 0x00000004; // SS2 Interrupt Mask
pub const ADC_IM_MASK1: usize = 0x00000002; // SS1 Interrupt Mask
pub const ADC_IM_MASK0: usize = 0x00000001; // SS0 Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_ISC register.
//
// *****************************************************************************
pub const ADC_ISC_DCINSS3: usize = 0x00080000; // Digital Comparator Interrupt Status on SS3
pub const ADC_ISC_DCINSS2: usize = 0x00040000; // Digital Comparator Interrupt Status on SS2
pub const ADC_ISC_DCINSS1: usize = 0x00020000; // Digital Comparator Interrupt Status on SS1
pub const ADC_ISC_DCINSS0: usize = 0x00010000; // Digital Comparator Interrupt Status on SS0
pub const ADC_ISC_IN3: usize = 0x00000008; // SS3 Interrupt Status and Clear
pub const ADC_ISC_IN2: usize = 0x00000004; // SS2 Interrupt Status and Clear
pub const ADC_ISC_IN1: usize = 0x00000002; // SS1 Interrupt Status and Clear
pub const ADC_ISC_IN0: usize = 0x00000001; // SS0 Interrupt Status and Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_OSTAT register.
//
// *****************************************************************************
pub const ADC_OSTAT_OV3: usize = 0x00000008; // SS3 FIFO Overflow
pub const ADC_OSTAT_OV2: usize = 0x00000004; // SS2 FIFO Overflow
pub const ADC_OSTAT_OV1: usize = 0x00000002; // SS1 FIFO Overflow
pub const ADC_OSTAT_OV0: usize = 0x00000001; // SS0 FIFO Overflow

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_EMUX register.
//
// *****************************************************************************
pub const ADC_EMUX_EM3_M: usize = 0x0000F000; // SS3 Trigger Select
pub const ADC_EMUX_EM3_PROCESSOR: usize = 0x00000000; // Processor (default)
pub const ADC_EMUX_EM3_COMP0: usize = 0x00001000; // Analog Comparator 0
pub const ADC_EMUX_EM3_COMP1: usize = 0x00002000; // Analog Comparator 1
pub const ADC_EMUX_EM3_EXTERNAL: usize = 0x00004000; // External (GPIO PB4)
pub const ADC_EMUX_EM3_TIMER: usize = 0x00005000; // Timer
pub const ADC_EMUX_EM3_ALWAYS: usize = 0x0000F000; // Always (continuously sample)
pub const ADC_EMUX_EM2_M: usize = 0x00000F00; // SS2 Trigger Select
pub const ADC_EMUX_EM2_PROCESSOR: usize = 0x00000000; // Processor (default)
pub const ADC_EMUX_EM2_COMP0: usize = 0x00000100; // Analog Comparator 0
pub const ADC_EMUX_EM2_COMP1: usize = 0x00000200; // Analog Comparator 1
pub const ADC_EMUX_EM2_EXTERNAL: usize = 0x00000400; // External (GPIO PB4)
pub const ADC_EMUX_EM2_TIMER: usize = 0x00000500; // Timer
pub const ADC_EMUX_EM2_ALWAYS: usize = 0x00000F00; // Always (continuously sample)
pub const ADC_EMUX_EM1_M: usize = 0x000000F0; // SS1 Trigger Select
pub const ADC_EMUX_EM1_PROCESSOR: usize = 0x00000000; // Processor (default)
pub const ADC_EMUX_EM1_COMP0: usize = 0x00000010; // Analog Comparator 0
pub const ADC_EMUX_EM1_COMP1: usize = 0x00000020; // Analog Comparator 1
pub const ADC_EMUX_EM1_EXTERNAL: usize = 0x00000040; // External (GPIO PB4)
pub const ADC_EMUX_EM1_TIMER: usize = 0x00000050; // Timer
pub const ADC_EMUX_EM1_ALWAYS: usize = 0x000000F0; // Always (continuously sample)
pub const ADC_EMUX_EM0_M: usize = 0x0000000F; // SS0 Trigger Select
pub const ADC_EMUX_EM0_PROCESSOR: usize = 0x00000000; // Processor (default)
pub const ADC_EMUX_EM0_COMP0: usize = 0x00000001; // Analog Comparator 0
pub const ADC_EMUX_EM0_COMP1: usize = 0x00000002; // Analog Comparator 1
pub const ADC_EMUX_EM0_EXTERNAL: usize = 0x00000004; // External (GPIO PB4)
pub const ADC_EMUX_EM0_TIMER: usize = 0x00000005; // Timer
pub const ADC_EMUX_EM0_ALWAYS: usize = 0x0000000F; // Always (continuously sample)

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_USTAT register.
//
// *****************************************************************************
pub const ADC_USTAT_UV3: usize = 0x00000008; // SS3 FIFO Underflow
pub const ADC_USTAT_UV2: usize = 0x00000004; // SS2 FIFO Underflow
pub const ADC_USTAT_UV1: usize = 0x00000002; // SS1 FIFO Underflow
pub const ADC_USTAT_UV0: usize = 0x00000001; // SS0 FIFO Underflow

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSPRI register.
//
// *****************************************************************************
pub const ADC_SSPRI_SS3_M: usize = 0x00003000; // SS3 Priority
pub const ADC_SSPRI_SS3_1ST: usize = 0x00000000; // First priority
pub const ADC_SSPRI_SS3_2ND: usize = 0x00001000; // Second priority
pub const ADC_SSPRI_SS3_3RD: usize = 0x00002000; // Third priority
pub const ADC_SSPRI_SS3_4TH: usize = 0x00003000; // Fourth priority
pub const ADC_SSPRI_SS2_M: usize = 0x00000300; // SS2 Priority
pub const ADC_SSPRI_SS2_1ST: usize = 0x00000000; // First priority
pub const ADC_SSPRI_SS2_2ND: usize = 0x00000100; // Second priority
pub const ADC_SSPRI_SS2_3RD: usize = 0x00000200; // Third priority
pub const ADC_SSPRI_SS2_4TH: usize = 0x00000300; // Fourth priority
pub const ADC_SSPRI_SS1_M: usize = 0x00000030; // SS1 Priority
pub const ADC_SSPRI_SS1_1ST: usize = 0x00000000; // First priority
pub const ADC_SSPRI_SS1_2ND: usize = 0x00000010; // Second priority
pub const ADC_SSPRI_SS1_3RD: usize = 0x00000020; // Third priority
pub const ADC_SSPRI_SS1_4TH: usize = 0x00000030; // Fourth priority
pub const ADC_SSPRI_SS0_M: usize = 0x00000003; // SS0 Priority
pub const ADC_SSPRI_SS0_1ST: usize = 0x00000000; // First priority
pub const ADC_SSPRI_SS0_2ND: usize = 0x00000001; // Second priority
pub const ADC_SSPRI_SS0_3RD: usize = 0x00000002; // Third priority
pub const ADC_SSPRI_SS0_4TH: usize = 0x00000003; // Fourth priority

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SPC register.
//
// *****************************************************************************
pub const ADC_SPC_PHASE_M: usize = 0x0000000F; // Phase Difference
pub const ADC_SPC_PHASE_0: usize = 0x00000000; // ADC sample lags by 0.0
pub const ADC_SPC_PHASE_22_5: usize = 0x00000001; // ADC sample lags by 22.5
pub const ADC_SPC_PHASE_45: usize = 0x00000002; // ADC sample lags by 45.0
pub const ADC_SPC_PHASE_67_5: usize = 0x00000003; // ADC sample lags by 67.5
pub const ADC_SPC_PHASE_90: usize = 0x00000004; // ADC sample lags by 90.0
pub const ADC_SPC_PHASE_112_5: usize = 0x00000005; // ADC sample lags by 112.5
pub const ADC_SPC_PHASE_135: usize = 0x00000006; // ADC sample lags by 135.0
pub const ADC_SPC_PHASE_157_5: usize = 0x00000007; // ADC sample lags by 157.5
pub const ADC_SPC_PHASE_180: usize = 0x00000008; // ADC sample lags by 180.0
pub const ADC_SPC_PHASE_202_5: usize = 0x00000009; // ADC sample lags by 202.5
pub const ADC_SPC_PHASE_225: usize = 0x0000000A; // ADC sample lags by 225.0
pub const ADC_SPC_PHASE_247_5: usize = 0x0000000B; // ADC sample lags by 247.5
pub const ADC_SPC_PHASE_270: usize = 0x0000000C; // ADC sample lags by 270.0
pub const ADC_SPC_PHASE_292_5: usize = 0x0000000D; // ADC sample lags by 292.5
pub const ADC_SPC_PHASE_315: usize = 0x0000000E; // ADC sample lags by 315.0
pub const ADC_SPC_PHASE_337_5: usize = 0x0000000F; // ADC sample lags by 337.5

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PSSI register.
//
// *****************************************************************************
pub const ADC_PSSI_GSYNC: usize = 0x80000000; // Global Synchronize
pub const ADC_PSSI_SYNCWAIT: usize = 0x08000000; // Synchronize Wait
pub const ADC_PSSI_SS3: usize = 0x00000008; // SS3 Initiate
pub const ADC_PSSI_SS2: usize = 0x00000004; // SS2 Initiate
pub const ADC_PSSI_SS1: usize = 0x00000002; // SS1 Initiate
pub const ADC_PSSI_SS0: usize = 0x00000001; // SS0 Initiate

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SAC register.
//
// *****************************************************************************
pub const ADC_SAC_AVG_M: usize = 0x00000007; // Hardware Averaging Control
pub const ADC_SAC_AVG_OFF: usize = 0x00000000; // No hardware oversampling
pub const ADC_SAC_AVG_2X: usize = 0x00000001; // 2x hardware oversampling
pub const ADC_SAC_AVG_4X: usize = 0x00000002; // 4x hardware oversampling
pub const ADC_SAC_AVG_8X: usize = 0x00000003; // 8x hardware oversampling
pub const ADC_SAC_AVG_16X: usize = 0x00000004; // 16x hardware oversampling
pub const ADC_SAC_AVG_32X: usize = 0x00000005; // 32x hardware oversampling
pub const ADC_SAC_AVG_64X: usize = 0x00000006; // 64x hardware oversampling

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCISC register.
//
// *****************************************************************************
pub const ADC_DCISC_DCINT7: usize = 0x00000080; // Digital Comparator 7 Interrupt Status and Clear
pub const ADC_DCISC_DCINT6: usize = 0x00000040; // Digital Comparator 6 Interrupt Status and Clear
pub const ADC_DCISC_DCINT5: usize = 0x00000020; // Digital Comparator 5 Interrupt Status and Clear
pub const ADC_DCISC_DCINT4: usize = 0x00000010; // Digital Comparator 4 Interrupt Status and Clear
pub const ADC_DCISC_DCINT3: usize = 0x00000008; // Digital Comparator 3 Interrupt Status and Clear
pub const ADC_DCISC_DCINT2: usize = 0x00000004; // Digital Comparator 2 Interrupt Status and Clear
pub const ADC_DCISC_DCINT1: usize = 0x00000002; // Digital Comparator 1 Interrupt Status and Clear
pub const ADC_DCISC_DCINT0: usize = 0x00000001; // Digital Comparator 0 Interrupt Status and Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX0 register.
//
// *****************************************************************************
pub const ADC_SSMUX0_MUX7_M: usize = 0xF0000000; // 8th Sample Input Select
pub const ADC_SSMUX0_MUX6_M: usize = 0x0F000000; // 7th Sample Input Select
pub const ADC_SSMUX0_MUX5_M: usize = 0x00F00000; // 6th Sample Input Select
pub const ADC_SSMUX0_MUX4_M: usize = 0x000F0000; // 5th Sample Input Select
pub const ADC_SSMUX0_MUX3_M: usize = 0x0000F000; // 4th Sample Input Select
pub const ADC_SSMUX0_MUX2_M: usize = 0x00000F00; // 3rd Sample Input Select
pub const ADC_SSMUX0_MUX1_M: usize = 0x000000F0; // 2nd Sample Input Select
pub const ADC_SSMUX0_MUX0_M: usize = 0x0000000F; // 1st Sample Input Select
pub const ADC_SSMUX0_MUX7_S: usize = 28;
pub const ADC_SSMUX0_MUX6_S: usize = 24;
pub const ADC_SSMUX0_MUX5_S: usize = 20;
pub const ADC_SSMUX0_MUX4_S: usize = 16;
pub const ADC_SSMUX0_MUX3_S: usize = 12;
pub const ADC_SSMUX0_MUX2_S: usize = 8;
pub const ADC_SSMUX0_MUX1_S: usize = 4;
pub const ADC_SSMUX0_MUX0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL0 register.
//
// *****************************************************************************
pub const ADC_SSCTL0_TS7: usize = 0x80000000; // 8th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE7: usize = 0x40000000; // 8th Sample Interrupt Enable
pub const ADC_SSCTL0_END7: usize = 0x20000000; // 8th Sample is End of Sequence
pub const ADC_SSCTL0_D7: usize = 0x10000000; // 8th Sample Diff Input Select
pub const ADC_SSCTL0_TS6: usize = 0x08000000; // 7th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE6: usize = 0x04000000; // 7th Sample Interrupt Enable
pub const ADC_SSCTL0_END6: usize = 0x02000000; // 7th Sample is End of Sequence
pub const ADC_SSCTL0_D6: usize = 0x01000000; // 7th Sample Diff Input Select
pub const ADC_SSCTL0_TS5: usize = 0x00800000; // 6th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE5: usize = 0x00400000; // 6th Sample Interrupt Enable
pub const ADC_SSCTL0_END5: usize = 0x00200000; // 6th Sample is End of Sequence
pub const ADC_SSCTL0_D5: usize = 0x00100000; // 6th Sample Diff Input Select
pub const ADC_SSCTL0_TS4: usize = 0x00080000; // 5th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE4: usize = 0x00040000; // 5th Sample Interrupt Enable
pub const ADC_SSCTL0_END4: usize = 0x00020000; // 5th Sample is End of Sequence
pub const ADC_SSCTL0_D4: usize = 0x00010000; // 5th Sample Diff Input Select
pub const ADC_SSCTL0_TS3: usize = 0x00008000; // 4th Sample Temp Sensor Select
pub const ADC_SSCTL0_IE3: usize = 0x00004000; // 4th Sample Interrupt Enable
pub const ADC_SSCTL0_END3: usize = 0x00002000; // 4th Sample is End of Sequence
pub const ADC_SSCTL0_D3: usize = 0x00001000; // 4th Sample Diff Input Select
pub const ADC_SSCTL0_TS2: usize = 0x00000800; // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL0_IE2: usize = 0x00000400; // 3rd Sample Interrupt Enable
pub const ADC_SSCTL0_END2: usize = 0x00000200; // 3rd Sample is End of Sequence
pub const ADC_SSCTL0_D2: usize = 0x00000100; // 3rd Sample Diff Input Select
pub const ADC_SSCTL0_TS1: usize = 0x00000080; // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL0_IE1: usize = 0x00000040; // 2nd Sample Interrupt Enable
pub const ADC_SSCTL0_END1: usize = 0x00000020; // 2nd Sample is End of Sequence
pub const ADC_SSCTL0_D1: usize = 0x00000010; // 2nd Sample Diff Input Select
pub const ADC_SSCTL0_TS0: usize = 0x00000008; // 1st Sample Temp Sensor Select
pub const ADC_SSCTL0_IE0: usize = 0x00000004; // 1st Sample Interrupt Enable
pub const ADC_SSCTL0_END0: usize = 0x00000002; // 1st Sample is End of Sequence
pub const ADC_SSCTL0_D0: usize = 0x00000001; // 1st Sample Diff Input Select

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO0 register.
//
// *****************************************************************************
pub const ADC_SSFIFO0_DATA_M: usize = 0x00000FFF; // Conversion Result Data
pub const ADC_SSFIFO0_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT0 register.
//
// *****************************************************************************
pub const ADC_SSFSTAT0_FULL: usize = 0x00001000; // FIFO Full
pub const ADC_SSFSTAT0_EMPTY: usize = 0x00000100; // FIFO Empty
pub const ADC_SSFSTAT0_HPTR_M: usize = 0x000000F0; // FIFO Head Pointer
pub const ADC_SSFSTAT0_TPTR_M: usize = 0x0000000F; // FIFO Tail Pointer
pub const ADC_SSFSTAT0_HPTR_S: usize = 4;
pub const ADC_SSFSTAT0_TPTR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP0 register.
//
// *****************************************************************************
pub const ADC_SSOP0_S7DCOP: usize = 0x10000000; // Sample 7 Digital Comparator Operation
pub const ADC_SSOP0_S6DCOP: usize = 0x01000000; // Sample 6 Digital Comparator Operation
pub const ADC_SSOP0_S5DCOP: usize = 0x00100000; // Sample 5 Digital Comparator Operation
pub const ADC_SSOP0_S4DCOP: usize = 0x00010000; // Sample 4 Digital Comparator Operation
pub const ADC_SSOP0_S3DCOP: usize = 0x00001000; // Sample 3 Digital Comparator Operation
pub const ADC_SSOP0_S2DCOP: usize = 0x00000100; // Sample 2 Digital Comparator Operation
pub const ADC_SSOP0_S1DCOP: usize = 0x00000010; // Sample 1 Digital Comparator Operation
pub const ADC_SSOP0_S0DCOP: usize = 0x00000001; // Sample 0 Digital Comparator Operation

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC0 register.
//
// *****************************************************************************
pub const ADC_SSDC0_S7DCSEL_M: usize = 0xF0000000; // Sample 7 Digital Comparator Select
pub const ADC_SSDC0_S6DCSEL_M: usize = 0x0F000000; // Sample 6 Digital Comparator Select
pub const ADC_SSDC0_S5DCSEL_M: usize = 0x00F00000; // Sample 5 Digital Comparator Select
pub const ADC_SSDC0_S4DCSEL_M: usize = 0x000F0000; // Sample 4 Digital Comparator Select
pub const ADC_SSDC0_S3DCSEL_M: usize = 0x0000F000; // Sample 3 Digital Comparator Select
pub const ADC_SSDC0_S2DCSEL_M: usize = 0x00000F00; // Sample 2 Digital Comparator Select
pub const ADC_SSDC0_S1DCSEL_M: usize = 0x000000F0; // Sample 1 Digital Comparator Select
pub const ADC_SSDC0_S0DCSEL_M: usize = 0x0000000F; // Sample 0 Digital Comparator Select
pub const ADC_SSDC0_S6DCSEL_S: usize = 24;
pub const ADC_SSDC0_S5DCSEL_S: usize = 20;
pub const ADC_SSDC0_S4DCSEL_S: usize = 16;
pub const ADC_SSDC0_S3DCSEL_S: usize = 12;
pub const ADC_SSDC0_S2DCSEL_S: usize = 8;
pub const ADC_SSDC0_S1DCSEL_S: usize = 4;
pub const ADC_SSDC0_S0DCSEL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX1 register.
//
// *****************************************************************************
pub const ADC_SSMUX1_MUX3_M: usize = 0x0000F000; // 4th Sample Input Select
pub const ADC_SSMUX1_MUX2_M: usize = 0x00000F00; // 3rd Sample Input Select
pub const ADC_SSMUX1_MUX1_M: usize = 0x000000F0; // 2nd Sample Input Select
pub const ADC_SSMUX1_MUX0_M: usize = 0x0000000F; // 1st Sample Input Select
pub const ADC_SSMUX1_MUX3_S: usize = 12;
pub const ADC_SSMUX1_MUX2_S: usize = 8;
pub const ADC_SSMUX1_MUX1_S: usize = 4;
pub const ADC_SSMUX1_MUX0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL1 register.
//
// *****************************************************************************
pub const ADC_SSCTL1_TS3: usize = 0x00008000; // 4th Sample Temp Sensor Select
pub const ADC_SSCTL1_IE3: usize = 0x00004000; // 4th Sample Interrupt Enable
pub const ADC_SSCTL1_END3: usize = 0x00002000; // 4th Sample is End of Sequence
pub const ADC_SSCTL1_D3: usize = 0x00001000; // 4th Sample Diff Input Select
pub const ADC_SSCTL1_TS2: usize = 0x00000800; // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL1_IE2: usize = 0x00000400; // 3rd Sample Interrupt Enable
pub const ADC_SSCTL1_END2: usize = 0x00000200; // 3rd Sample is End of Sequence
pub const ADC_SSCTL1_D2: usize = 0x00000100; // 3rd Sample Diff Input Select
pub const ADC_SSCTL1_TS1: usize = 0x00000080; // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL1_IE1: usize = 0x00000040; // 2nd Sample Interrupt Enable
pub const ADC_SSCTL1_END1: usize = 0x00000020; // 2nd Sample is End of Sequence
pub const ADC_SSCTL1_D1: usize = 0x00000010; // 2nd Sample Diff Input Select
pub const ADC_SSCTL1_TS0: usize = 0x00000008; // 1st Sample Temp Sensor Select
pub const ADC_SSCTL1_IE0: usize = 0x00000004; // 1st Sample Interrupt Enable
pub const ADC_SSCTL1_END0: usize = 0x00000002; // 1st Sample is End of Sequence
pub const ADC_SSCTL1_D0: usize = 0x00000001; // 1st Sample Diff Input Select

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO1 register.
//
// *****************************************************************************
pub const ADC_SSFIFO1_DATA_M: usize = 0x00000FFF; // Conversion Result Data
pub const ADC_SSFIFO1_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT1 register.
//
// *****************************************************************************
pub const ADC_SSFSTAT1_FULL: usize = 0x00001000; // FIFO Full
pub const ADC_SSFSTAT1_EMPTY: usize = 0x00000100; // FIFO Empty
pub const ADC_SSFSTAT1_HPTR_M: usize = 0x000000F0; // FIFO Head Pointer
pub const ADC_SSFSTAT1_TPTR_M: usize = 0x0000000F; // FIFO Tail Pointer
pub const ADC_SSFSTAT1_HPTR_S: usize = 4;
pub const ADC_SSFSTAT1_TPTR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP1 register.
//
// *****************************************************************************
pub const ADC_SSOP1_S3DCOP: usize = 0x00001000; // Sample 3 Digital Comparator Operation
pub const ADC_SSOP1_S2DCOP: usize = 0x00000100; // Sample 2 Digital Comparator Operation
pub const ADC_SSOP1_S1DCOP: usize = 0x00000010; // Sample 1 Digital Comparator Operation
pub const ADC_SSOP1_S0DCOP: usize = 0x00000001; // Sample 0 Digital Comparator Operation

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC1 register.
//
// *****************************************************************************
pub const ADC_SSDC1_S3DCSEL_M: usize = 0x0000F000; // Sample 3 Digital Comparator Select
pub const ADC_SSDC1_S2DCSEL_M: usize = 0x00000F00; // Sample 2 Digital Comparator Select
pub const ADC_SSDC1_S1DCSEL_M: usize = 0x000000F0; // Sample 1 Digital Comparator Select
pub const ADC_SSDC1_S0DCSEL_M: usize = 0x0000000F; // Sample 0 Digital Comparator Select
pub const ADC_SSDC1_S2DCSEL_S: usize = 8;
pub const ADC_SSDC1_S1DCSEL_S: usize = 4;
pub const ADC_SSDC1_S0DCSEL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX2 register.
//
// *****************************************************************************
pub const ADC_SSMUX2_MUX3_M: usize = 0x0000F000; // 4th Sample Input Select
pub const ADC_SSMUX2_MUX2_M: usize = 0x00000F00; // 3rd Sample Input Select
pub const ADC_SSMUX2_MUX1_M: usize = 0x000000F0; // 2nd Sample Input Select
pub const ADC_SSMUX2_MUX0_M: usize = 0x0000000F; // 1st Sample Input Select
pub const ADC_SSMUX2_MUX3_S: usize = 12;
pub const ADC_SSMUX2_MUX2_S: usize = 8;
pub const ADC_SSMUX2_MUX1_S: usize = 4;
pub const ADC_SSMUX2_MUX0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL2 register.
//
// *****************************************************************************
pub const ADC_SSCTL2_TS3: usize = 0x00008000; // 4th Sample Temp Sensor Select
pub const ADC_SSCTL2_IE3: usize = 0x00004000; // 4th Sample Interrupt Enable
pub const ADC_SSCTL2_END3: usize = 0x00002000; // 4th Sample is End of Sequence
pub const ADC_SSCTL2_D3: usize = 0x00001000; // 4th Sample Diff Input Select
pub const ADC_SSCTL2_TS2: usize = 0x00000800; // 3rd Sample Temp Sensor Select
pub const ADC_SSCTL2_IE2: usize = 0x00000400; // 3rd Sample Interrupt Enable
pub const ADC_SSCTL2_END2: usize = 0x00000200; // 3rd Sample is End of Sequence
pub const ADC_SSCTL2_D2: usize = 0x00000100; // 3rd Sample Diff Input Select
pub const ADC_SSCTL2_TS1: usize = 0x00000080; // 2nd Sample Temp Sensor Select
pub const ADC_SSCTL2_IE1: usize = 0x00000040; // 2nd Sample Interrupt Enable
pub const ADC_SSCTL2_END1: usize = 0x00000020; // 2nd Sample is End of Sequence
pub const ADC_SSCTL2_D1: usize = 0x00000010; // 2nd Sample Diff Input Select
pub const ADC_SSCTL2_TS0: usize = 0x00000008; // 1st Sample Temp Sensor Select
pub const ADC_SSCTL2_IE0: usize = 0x00000004; // 1st Sample Interrupt Enable
pub const ADC_SSCTL2_END0: usize = 0x00000002; // 1st Sample is End of Sequence
pub const ADC_SSCTL2_D0: usize = 0x00000001; // 1st Sample Diff Input Select

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO2 register.
//
// *****************************************************************************
pub const ADC_SSFIFO2_DATA_M: usize = 0x00000FFF; // Conversion Result Data
pub const ADC_SSFIFO2_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT2 register.
//
// *****************************************************************************
pub const ADC_SSFSTAT2_FULL: usize = 0x00001000; // FIFO Full
pub const ADC_SSFSTAT2_EMPTY: usize = 0x00000100; // FIFO Empty
pub const ADC_SSFSTAT2_HPTR_M: usize = 0x000000F0; // FIFO Head Pointer
pub const ADC_SSFSTAT2_TPTR_M: usize = 0x0000000F; // FIFO Tail Pointer
pub const ADC_SSFSTAT2_HPTR_S: usize = 4;
pub const ADC_SSFSTAT2_TPTR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP2 register.
//
// *****************************************************************************
pub const ADC_SSOP2_S3DCOP: usize = 0x00001000; // Sample 3 Digital Comparator Operation
pub const ADC_SSOP2_S2DCOP: usize = 0x00000100; // Sample 2 Digital Comparator Operation
pub const ADC_SSOP2_S1DCOP: usize = 0x00000010; // Sample 1 Digital Comparator Operation
pub const ADC_SSOP2_S0DCOP: usize = 0x00000001; // Sample 0 Digital Comparator Operation

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC2 register.
//
// *****************************************************************************
pub const ADC_SSDC2_S3DCSEL_M: usize = 0x0000F000; // Sample 3 Digital Comparator Select
pub const ADC_SSDC2_S2DCSEL_M: usize = 0x00000F00; // Sample 2 Digital Comparator Select
pub const ADC_SSDC2_S1DCSEL_M: usize = 0x000000F0; // Sample 1 Digital Comparator Select
pub const ADC_SSDC2_S0DCSEL_M: usize = 0x0000000F; // Sample 0 Digital Comparator Select
pub const ADC_SSDC2_S2DCSEL_S: usize = 8;
pub const ADC_SSDC2_S1DCSEL_S: usize = 4;
pub const ADC_SSDC2_S0DCSEL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSMUX3 register.
//
// *****************************************************************************
pub const ADC_SSMUX3_MUX0_M: usize = 0x0000000F; // 1st Sample Input Select
pub const ADC_SSMUX3_MUX0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSCTL3 register.
//
// *****************************************************************************
pub const ADC_SSCTL3_TS0: usize = 0x00000008; // 1st Sample Temp Sensor Select
pub const ADC_SSCTL3_IE0: usize = 0x00000004; // 1st Sample Interrupt Enable
pub const ADC_SSCTL3_END0: usize = 0x00000002; // 1st Sample is End of Sequence
pub const ADC_SSCTL3_D0: usize = 0x00000001; // 1st Sample Diff Input Select

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFIFO3 register.
//
// *****************************************************************************
pub const ADC_SSFIFO3_DATA_M: usize = 0x00000FFF; // Conversion Result Data
pub const ADC_SSFIFO3_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSFSTAT3 register.
//
// *****************************************************************************
pub const ADC_SSFSTAT3_FULL: usize = 0x00001000; // FIFO Full
pub const ADC_SSFSTAT3_EMPTY: usize = 0x00000100; // FIFO Empty
pub const ADC_SSFSTAT3_HPTR_M: usize = 0x000000F0; // FIFO Head Pointer
pub const ADC_SSFSTAT3_TPTR_M: usize = 0x0000000F; // FIFO Tail Pointer
pub const ADC_SSFSTAT3_HPTR_S: usize = 4;
pub const ADC_SSFSTAT3_TPTR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSOP3 register.
//
// *****************************************************************************
pub const ADC_SSOP3_S0DCOP: usize = 0x00000001; // Sample 0 Digital Comparator Operation

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_SSDC3 register.
//
// *****************************************************************************
pub const ADC_SSDC3_S0DCSEL_M: usize = 0x0000000F; // Sample 0 Digital Comparator Select

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCRIC register.
//
// *****************************************************************************
pub const ADC_DCRIC_DCTRIG7: usize = 0x00800000; // Digital Comparator Trigger 7
pub const ADC_DCRIC_DCTRIG6: usize = 0x00400000; // Digital Comparator Trigger 6
pub const ADC_DCRIC_DCTRIG5: usize = 0x00200000; // Digital Comparator Trigger 5
pub const ADC_DCRIC_DCTRIG4: usize = 0x00100000; // Digital Comparator Trigger 4
pub const ADC_DCRIC_DCTRIG3: usize = 0x00080000; // Digital Comparator Trigger 3
pub const ADC_DCRIC_DCTRIG2: usize = 0x00040000; // Digital Comparator Trigger 2
pub const ADC_DCRIC_DCTRIG1: usize = 0x00020000; // Digital Comparator Trigger 1
pub const ADC_DCRIC_DCTRIG0: usize = 0x00010000; // Digital Comparator Trigger 0
pub const ADC_DCRIC_DCINT7: usize = 0x00000080; // Digital Comparator Interrupt 7
pub const ADC_DCRIC_DCINT6: usize = 0x00000040; // Digital Comparator Interrupt 6
pub const ADC_DCRIC_DCINT5: usize = 0x00000020; // Digital Comparator Interrupt 5
pub const ADC_DCRIC_DCINT4: usize = 0x00000010; // Digital Comparator Interrupt 4
pub const ADC_DCRIC_DCINT3: usize = 0x00000008; // Digital Comparator Interrupt 3
pub const ADC_DCRIC_DCINT2: usize = 0x00000004; // Digital Comparator Interrupt 2
pub const ADC_DCRIC_DCINT1: usize = 0x00000002; // Digital Comparator Interrupt 1
pub const ADC_DCRIC_DCINT0: usize = 0x00000001; // Digital Comparator Interrupt 0

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL0 register.
//
// *****************************************************************************
pub const ADC_DCCTL0_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL0_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL0_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL0_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL0_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL0_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL0_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL0_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL0_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL0_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL1 register.
//
// *****************************************************************************
pub const ADC_DCCTL1_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL1_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL1_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL1_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL1_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL1_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL1_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL1_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL1_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL1_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL2 register.
//
// *****************************************************************************
pub const ADC_DCCTL2_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL2_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL2_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL2_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL2_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL2_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL2_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL2_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL2_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL2_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL3 register.
//
// *****************************************************************************
pub const ADC_DCCTL3_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL3_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL3_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL3_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL3_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL3_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL3_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL3_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL3_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL3_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL4 register.
//
// *****************************************************************************
pub const ADC_DCCTL4_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL4_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL4_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL4_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL4_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL4_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL4_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL4_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL4_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL4_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL5 register.
//
// *****************************************************************************
pub const ADC_DCCTL5_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL5_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL5_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL5_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL5_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL5_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL5_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL5_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL5_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL5_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL6 register.
//
// *****************************************************************************
pub const ADC_DCCTL6_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL6_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL6_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL6_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL6_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL6_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL6_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL6_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL6_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL6_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCTL7 register.
//
// *****************************************************************************
pub const ADC_DCCTL7_CIE: usize = 0x00000010; // Comparison Interrupt Enable
pub const ADC_DCCTL7_CIC_M: usize = 0x0000000C; // Comparison Interrupt Condition
pub const ADC_DCCTL7_CIC_LOW: usize = 0x00000000; // Low Band
pub const ADC_DCCTL7_CIC_MID: usize = 0x00000004; // Mid Band
pub const ADC_DCCTL7_CIC_HIGH: usize = 0x0000000C; // High Band
pub const ADC_DCCTL7_CIM_M: usize = 0x00000003; // Comparison Interrupt Mode
pub const ADC_DCCTL7_CIM_ALWAYS: usize = 0x00000000; // Always
pub const ADC_DCCTL7_CIM_ONCE: usize = 0x00000001; // Once
pub const ADC_DCCTL7_CIM_HALWAYS: usize = 0x00000002; // Hysteresis Always
pub const ADC_DCCTL7_CIM_HONCE: usize = 0x00000003; // Hysteresis Once

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP0 register.
//
// *****************************************************************************
pub const ADC_DCCMP0_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP0_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP0_COMP1_S: usize = 16;
pub const ADC_DCCMP0_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP1 register.
//
// *****************************************************************************
pub const ADC_DCCMP1_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP1_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP1_COMP1_S: usize = 16;
pub const ADC_DCCMP1_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP2 register.
//
// *****************************************************************************
pub const ADC_DCCMP2_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP2_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP2_COMP1_S: usize = 16;
pub const ADC_DCCMP2_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP3 register.
//
// *****************************************************************************
pub const ADC_DCCMP3_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP3_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP3_COMP1_S: usize = 16;
pub const ADC_DCCMP3_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP4 register.
//
// *****************************************************************************
pub const ADC_DCCMP4_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP4_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP4_COMP1_S: usize = 16;
pub const ADC_DCCMP4_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP5 register.
//
// *****************************************************************************
pub const ADC_DCCMP5_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP5_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP5_COMP1_S: usize = 16;
pub const ADC_DCCMP5_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP6 register.
//
// *****************************************************************************
pub const ADC_DCCMP6_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP6_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP6_COMP1_S: usize = 16;
pub const ADC_DCCMP6_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_DCCMP7 register.
//
// *****************************************************************************
pub const ADC_DCCMP7_COMP1_M: usize = 0x0FFF0000; // Compare 1
pub const ADC_DCCMP7_COMP0_M: usize = 0x00000FFF; // Compare 0
pub const ADC_DCCMP7_COMP1_S: usize = 16;
pub const ADC_DCCMP7_COMP0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PP register.
//
// *****************************************************************************
pub const ADC_PP_TS: usize = 0x00800000; // Temperature Sensor
pub const ADC_PP_RSL_M: usize = 0x007C0000; // Resolution
pub const ADC_PP_TYPE_M: usize = 0x00030000; // ADC Architecture
pub const ADC_PP_TYPE_SAR: usize = 0x00000000; // SAR
pub const ADC_PP_DC_M: usize = 0x0000FC00; // Digital Comparator Count
pub const ADC_PP_CH_M: usize = 0x000003F0; // ADC Channel Count
pub const ADC_PP_MSR_M: usize = 0x0000000F; // Maximum ADC Sample Rate
pub const ADC_PP_MSR_125K: usize = 0x00000001; // 125 ksps
pub const ADC_PP_MSR_250K: usize = 0x00000003; // 250 ksps
pub const ADC_PP_MSR_500K: usize = 0x00000005; // 500 ksps
pub const ADC_PP_MSR_1M: usize = 0x00000007; // 1 Msps
pub const ADC_PP_RSL_S: usize = 18;
pub const ADC_PP_DC_S: usize = 10;
pub const ADC_PP_CH_S: usize = 4;

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_PC register.
//
// *****************************************************************************
pub const ADC_PC_SR_M: usize = 0x0000000F; // ADC Sample Rate
pub const ADC_PC_SR_125K: usize = 0x00000001; // 125 ksps
pub const ADC_PC_SR_250K: usize = 0x00000003; // 250 ksps
pub const ADC_PC_SR_500K: usize = 0x00000005; // 500 ksps
pub const ADC_PC_SR_1M: usize = 0x00000007; // 1 Msps

// *****************************************************************************
//
// The following are defines for the bit fields in the ADC_O_CC register.
//
// *****************************************************************************
pub const ADC_CC_CS_M: usize = 0x0000000F; // ADC Clock Source
pub const ADC_CC_CS_SYSPLL: usize = 0x00000000; // Either the system clock (if the PLL bypass is in effect) or the 16 MHz clock derived from PLL / 25 (default)
pub const ADC_CC_CS_PIOSC: usize = 0x00000001; // PIOSC

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACMIS register.
//
// *****************************************************************************
pub const COMP_ACMIS_IN1: usize = 0x00000002; // Comparator 1 Masked Interrupt Status
pub const COMP_ACMIS_IN0: usize = 0x00000001; // Comparator 0 Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACRIS register.
//
// *****************************************************************************
pub const COMP_ACRIS_IN1: usize = 0x00000002; // Comparator 1 Interrupt Status
pub const COMP_ACRIS_IN0: usize = 0x00000001; // Comparator 0 Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACINTEN register.
//
// *****************************************************************************
pub const COMP_ACINTEN_IN1: usize = 0x00000002; // Comparator 1 Interrupt Enable
pub const COMP_ACINTEN_IN0: usize = 0x00000001; // Comparator 0 Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACREFCTL
// register.
//
// *****************************************************************************
pub const COMP_ACREFCTL_EN: usize = 0x00000200; // Resistor Ladder Enable
pub const COMP_ACREFCTL_RNG: usize = 0x00000100; // Resistor Ladder Range
pub const COMP_ACREFCTL_VREF_M: usize = 0x0000000F; // Resistor Ladder Voltage Ref
pub const COMP_ACREFCTL_VREF_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT0 register.
//
// *****************************************************************************
pub const COMP_ACSTAT0_OVAL: usize = 0x00000002; // Comparator Output Value

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL0 register.
//
// *****************************************************************************
pub const COMP_ACCTL0_TOEN: usize = 0x00000800; // Trigger Output Enable
pub const COMP_ACCTL0_ASRCP_M: usize = 0x00000600; // Analog Source Positive
pub const COMP_ACCTL0_ASRCP_PIN: usize = 0x00000000; // Pin value of Cn+
pub const COMP_ACCTL0_ASRCP_PIN0: usize = 0x00000200; // Pin value of C0+
pub const COMP_ACCTL0_ASRCP_REF: usize = 0x00000400; // Internal voltage reference
pub const COMP_ACCTL0_TSLVAL: usize = 0x00000080; // Trigger Sense Level Value
pub const COMP_ACCTL0_TSEN_M: usize = 0x00000060; // Trigger Sense
pub const COMP_ACCTL0_TSEN_LEVEL: usize = 0x00000000; // Level sense, see TSLVAL
pub const COMP_ACCTL0_TSEN_FALL: usize = 0x00000020; // Falling edge
pub const COMP_ACCTL0_TSEN_RISE: usize = 0x00000040; // Rising edge
pub const COMP_ACCTL0_TSEN_BOTH: usize = 0x00000060; // Either edge
pub const COMP_ACCTL0_ISLVAL: usize = 0x00000010; // Interrupt Sense Level Value
pub const COMP_ACCTL0_ISEN_M: usize = 0x0000000C; // Interrupt Sense
pub const COMP_ACCTL0_ISEN_LEVEL: usize = 0x00000000; // Level sense, see ISLVAL
pub const COMP_ACCTL0_ISEN_FALL: usize = 0x00000004; // Falling edge
pub const COMP_ACCTL0_ISEN_RISE: usize = 0x00000008; // Rising edge
pub const COMP_ACCTL0_ISEN_BOTH: usize = 0x0000000C; // Either edge
pub const COMP_ACCTL0_CINV: usize = 0x00000002; // Comparator Output Invert

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACSTAT1 register.
//
// *****************************************************************************
pub const COMP_ACSTAT1_OVAL: usize = 0x00000002; // Comparator Output Value

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_ACCTL1 register.
//
// *****************************************************************************
pub const COMP_ACCTL1_TOEN: usize = 0x00000800; // Trigger Output Enable
pub const COMP_ACCTL1_ASRCP_M: usize = 0x00000600; // Analog Source Positive
pub const COMP_ACCTL1_ASRCP_PIN: usize = 0x00000000; // Pin value of Cn+
pub const COMP_ACCTL1_ASRCP_PIN0: usize = 0x00000200; // Pin value of C0+
pub const COMP_ACCTL1_ASRCP_REF: usize = 0x00000400; // Internal voltage reference (VIREF)
pub const COMP_ACCTL1_TSLVAL: usize = 0x00000080; // Trigger Sense Level Value
pub const COMP_ACCTL1_TSEN_M: usize = 0x00000060; // Trigger Sense
pub const COMP_ACCTL1_TSEN_LEVEL: usize = 0x00000000; // Level sense, see TSLVAL
pub const COMP_ACCTL1_TSEN_FALL: usize = 0x00000020; // Falling edge
pub const COMP_ACCTL1_TSEN_RISE: usize = 0x00000040; // Rising edge
pub const COMP_ACCTL1_TSEN_BOTH: usize = 0x00000060; // Either edge
pub const COMP_ACCTL1_ISLVAL: usize = 0x00000010; // Interrupt Sense Level Value
pub const COMP_ACCTL1_ISEN_M: usize = 0x0000000C; // Interrupt Sense
pub const COMP_ACCTL1_ISEN_LEVEL: usize = 0x00000000; // Level sense, see ISLVAL
pub const COMP_ACCTL1_ISEN_FALL: usize = 0x00000004; // Falling edge
pub const COMP_ACCTL1_ISEN_RISE: usize = 0x00000008; // Rising edge
pub const COMP_ACCTL1_ISEN_BOTH: usize = 0x0000000C; // Either edge
pub const COMP_ACCTL1_CINV: usize = 0x00000002; // Comparator Output Invert

// *****************************************************************************
//
// The following are defines for the bit fields in the COMP_O_PP register.
//
// *****************************************************************************
pub const COMP_PP_C2O: usize = 0x00040000; // Comparator Output 2 Present
pub const COMP_PP_C1O: usize = 0x00020000; // Comparator Output 1 Present
pub const COMP_PP_C0O: usize = 0x00010000; // Comparator Output 0 Present
pub const COMP_PP_CMP2: usize = 0x00000004; // Comparator 2 Present
pub const COMP_PP_CMP1: usize = 0x00000002; // Comparator 1 Present
pub const COMP_PP_CMP0: usize = 0x00000001; // Comparator 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_CTL register.
//
// *****************************************************************************
pub const CAN_CTL_TEST: usize = 0x00000080; // Test Mode Enable
pub const CAN_CTL_CCE: usize = 0x00000040; // Configuration Change Enable
pub const CAN_CTL_DAR: usize = 0x00000020; // Disable Automatic-Retransmission
pub const CAN_CTL_EIE: usize = 0x00000008; // Error Interrupt Enable
pub const CAN_CTL_SIE: usize = 0x00000004; // Status Interrupt Enable
pub const CAN_CTL_IE: usize = 0x00000002; // CAN Interrupt Enable
pub const CAN_CTL_INIT: usize = 0x00000001; // Initialization

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_STS register.
//
// *****************************************************************************
pub const CAN_STS_BOFF: usize = 0x00000080; // Bus-Off Status
pub const CAN_STS_EWARN: usize = 0x00000040; // Warning Status
pub const CAN_STS_EPASS: usize = 0x00000020; // Error Passive
pub const CAN_STS_RXOK: usize = 0x00000010; // Received a Message Successfully
pub const CAN_STS_TXOK: usize = 0x00000008; // Transmitted a Message Successfully
pub const CAN_STS_LEC_M: usize = 0x00000007; // Last Error Code
pub const CAN_STS_LEC_NONE: usize = 0x00000000; // No Error
pub const CAN_STS_LEC_STUFF: usize = 0x00000001; // Stuff Error
pub const CAN_STS_LEC_FORM: usize = 0x00000002; // Format Error
pub const CAN_STS_LEC_ACK: usize = 0x00000003; // ACK Error
pub const CAN_STS_LEC_BIT1: usize = 0x00000004; // Bit 1 Error
pub const CAN_STS_LEC_BIT0: usize = 0x00000005; // Bit 0 Error
pub const CAN_STS_LEC_CRC: usize = 0x00000006; // CRC Error
pub const CAN_STS_LEC_NOEVENT: usize = 0x00000007; // No Event

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_ERR register.
//
// *****************************************************************************
pub const CAN_ERR_RP: usize = 0x00008000; // Received Error Passive
pub const CAN_ERR_REC_M: usize = 0x00007F00; // Receive Error Counter
pub const CAN_ERR_TEC_M: usize = 0x000000FF; // Transmit Error Counter
pub const CAN_ERR_REC_S: usize = 8;
pub const CAN_ERR_TEC_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BIT register.
//
// *****************************************************************************
pub const CAN_BIT_TSEG2_M: usize = 0x00007000; // Time Segment after Sample Point
pub const CAN_BIT_TSEG1_M: usize = 0x00000F00; // Time Segment Before Sample Point
pub const CAN_BIT_SJW_M: usize = 0x000000C0; // (Re)Synchronization Jump Width
pub const CAN_BIT_BRP_M: usize = 0x0000003F; // Baud Rate Prescaler
pub const CAN_BIT_TSEG2_S: usize = 12;
pub const CAN_BIT_TSEG1_S: usize = 8;
pub const CAN_BIT_SJW_S: usize = 6;
pub const CAN_BIT_BRP_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_INT register.
//
// *****************************************************************************
pub const CAN_INT_INTID_M: usize = 0x0000FFFF; // Interrupt Identifier
pub const CAN_INT_INTID_NONE: usize = 0x00000000; // No interrupt pending
pub const CAN_INT_INTID_STATUS: usize = 0x00008000; // Status Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TST register.
//
// *****************************************************************************
pub const CAN_TST_RX: usize = 0x00000080; // Receive Observation
pub const CAN_TST_TX_M: usize = 0x00000060; // Transmit Control
pub const CAN_TST_TX_CANCTL: usize = 0x00000000; // CAN Module Control
pub const CAN_TST_TX_SAMPLE: usize = 0x00000020; // Sample Point
pub const CAN_TST_TX_DOMINANT: usize = 0x00000040; // Driven Low
pub const CAN_TST_TX_RECESSIVE: usize = 0x00000060; // Driven High
pub const CAN_TST_LBACK: usize = 0x00000010; // Loopback Mode
pub const CAN_TST_SILENT: usize = 0x00000008; // Silent Mode
pub const CAN_TST_BASIC: usize = 0x00000004; // Basic Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_BRPE register.
//
// *****************************************************************************
pub const CAN_BRPE_BRPE_M: usize = 0x0000000F; // Baud Rate Prescaler Extension
pub const CAN_BRPE_BRPE_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CRQ register.
//
// *****************************************************************************
pub const CAN_IF1CRQ_BUSY: usize = 0x00008000; // Busy Flag
pub const CAN_IF1CRQ_MNUM_M: usize = 0x0000003F; // Message Number
pub const CAN_IF1CRQ_MNUM_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1CMSK register.
//
// *****************************************************************************
pub const CAN_IF1CMSK_WRNRD: usize = 0x00000080; // Write, Not Read
pub const CAN_IF1CMSK_MASK: usize = 0x00000040; // Access Mask Bits
pub const CAN_IF1CMSK_ARB: usize = 0x00000020; // Access Arbitration Bits
pub const CAN_IF1CMSK_CONTROL: usize = 0x00000010; // Access Control Bits
pub const CAN_IF1CMSK_CLRINTPND: usize = 0x00000008; // Clear Interrupt Pending Bit
pub const CAN_IF1CMSK_NEWDAT: usize = 0x00000004; // Access New Data
pub const CAN_IF1CMSK_TXRQST: usize = 0x00000004; // Access Transmission Request
pub const CAN_IF1CMSK_DATAA: usize = 0x00000002; // Access Data Byte 0 to 3
pub const CAN_IF1CMSK_DATAB: usize = 0x00000001; // Access Data Byte 4 to 7

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK1 register.
//
// *****************************************************************************
pub const CAN_IF1MSK1_IDMSK_M: usize = 0x0000FFFF; // Identifier Mask
pub const CAN_IF1MSK1_IDMSK_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MSK2 register.
//
// *****************************************************************************
pub const CAN_IF1MSK2_MXTD: usize = 0x00008000; // Mask Extended Identifier
pub const CAN_IF1MSK2_MDIR: usize = 0x00004000; // Mask Message Direction
pub const CAN_IF1MSK2_IDMSK_M: usize = 0x00001FFF; // Identifier Mask
pub const CAN_IF1MSK2_IDMSK_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB1 register.
//
// *****************************************************************************
pub const CAN_IF1ARB1_ID_M: usize = 0x0000FFFF; // Message Identifier
pub const CAN_IF1ARB1_ID_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1ARB2 register.
//
// *****************************************************************************
pub const CAN_IF1ARB2_MSGVAL: usize = 0x00008000; // Message Valid
pub const CAN_IF1ARB2_XTD: usize = 0x00004000; // Extended Identifier
pub const CAN_IF1ARB2_DIR: usize = 0x00002000; // Message Direction
pub const CAN_IF1ARB2_ID_M: usize = 0x00001FFF; // Message Identifier
pub const CAN_IF1ARB2_ID_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1MCTL register.
//
// *****************************************************************************
pub const CAN_IF1MCTL_NEWDAT: usize = 0x00008000; // New Data
pub const CAN_IF1MCTL_MSGLST: usize = 0x00004000; // Message Lost
pub const CAN_IF1MCTL_INTPND: usize = 0x00002000; // Interrupt Pending
pub const CAN_IF1MCTL_UMASK: usize = 0x00001000; // Use Acceptance Mask
pub const CAN_IF1MCTL_TXIE: usize = 0x00000800; // Transmit Interrupt Enable
pub const CAN_IF1MCTL_RXIE: usize = 0x00000400; // Receive Interrupt Enable
pub const CAN_IF1MCTL_RMTEN: usize = 0x00000200; // Remote Enable
pub const CAN_IF1MCTL_TXRQST: usize = 0x00000100; // Transmit Request
pub const CAN_IF1MCTL_EOB: usize = 0x00000080; // End of Buffer
pub const CAN_IF1MCTL_DLC_M: usize = 0x0000000F; // Data Length Code
pub const CAN_IF1MCTL_DLC_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA1 register.
//
// *****************************************************************************
pub const CAN_IF1DA1_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF1DA1_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DA2 register.
//
// *****************************************************************************
pub const CAN_IF1DA2_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF1DA2_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB1 register.
//
// *****************************************************************************
pub const CAN_IF1DB1_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF1DB1_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF1DB2 register.
//
// *****************************************************************************
pub const CAN_IF1DB2_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF1DB2_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CRQ register.
//
// *****************************************************************************
pub const CAN_IF2CRQ_BUSY: usize = 0x00008000; // Busy Flag
pub const CAN_IF2CRQ_MNUM_M: usize = 0x0000003F; // Message Number
pub const CAN_IF2CRQ_MNUM_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2CMSK register.
//
// *****************************************************************************
pub const CAN_IF2CMSK_WRNRD: usize = 0x00000080; // Write, Not Read
pub const CAN_IF2CMSK_MASK: usize = 0x00000040; // Access Mask Bits
pub const CAN_IF2CMSK_ARB: usize = 0x00000020; // Access Arbitration Bits
pub const CAN_IF2CMSK_CONTROL: usize = 0x00000010; // Access Control Bits
pub const CAN_IF2CMSK_CLRINTPND: usize = 0x00000008; // Clear Interrupt Pending Bit
pub const CAN_IF2CMSK_NEWDAT: usize = 0x00000004; // Access New Data
pub const CAN_IF2CMSK_TXRQST: usize = 0x00000004; // Access Transmission Request
pub const CAN_IF2CMSK_DATAA: usize = 0x00000002; // Access Data Byte 0 to 3
pub const CAN_IF2CMSK_DATAB: usize = 0x00000001; // Access Data Byte 4 to 7

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK1 register.
//
// *****************************************************************************
pub const CAN_IF2MSK1_IDMSK_M: usize = 0x0000FFFF; // Identifier Mask
pub const CAN_IF2MSK1_IDMSK_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MSK2 register.
//
// *****************************************************************************
pub const CAN_IF2MSK2_MXTD: usize = 0x00008000; // Mask Extended Identifier
pub const CAN_IF2MSK2_MDIR: usize = 0x00004000; // Mask Message Direction
pub const CAN_IF2MSK2_IDMSK_M: usize = 0x00001FFF; // Identifier Mask
pub const CAN_IF2MSK2_IDMSK_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB1 register.
//
// *****************************************************************************
pub const CAN_IF2ARB1_ID_M: usize = 0x0000FFFF; // Message Identifier
pub const CAN_IF2ARB1_ID_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2ARB2 register.
//
// *****************************************************************************
pub const CAN_IF2ARB2_MSGVAL: usize = 0x00008000; // Message Valid
pub const CAN_IF2ARB2_XTD: usize = 0x00004000; // Extended Identifier
pub const CAN_IF2ARB2_DIR: usize = 0x00002000; // Message Direction
pub const CAN_IF2ARB2_ID_M: usize = 0x00001FFF; // Message Identifier
pub const CAN_IF2ARB2_ID_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2MCTL register.
//
// *****************************************************************************
pub const CAN_IF2MCTL_NEWDAT: usize = 0x00008000; // New Data
pub const CAN_IF2MCTL_MSGLST: usize = 0x00004000; // Message Lost
pub const CAN_IF2MCTL_INTPND: usize = 0x00002000; // Interrupt Pending
pub const CAN_IF2MCTL_UMASK: usize = 0x00001000; // Use Acceptance Mask
pub const CAN_IF2MCTL_TXIE: usize = 0x00000800; // Transmit Interrupt Enable
pub const CAN_IF2MCTL_RXIE: usize = 0x00000400; // Receive Interrupt Enable
pub const CAN_IF2MCTL_RMTEN: usize = 0x00000200; // Remote Enable
pub const CAN_IF2MCTL_TXRQST: usize = 0x00000100; // Transmit Request
pub const CAN_IF2MCTL_EOB: usize = 0x00000080; // End of Buffer
pub const CAN_IF2MCTL_DLC_M: usize = 0x0000000F; // Data Length Code
pub const CAN_IF2MCTL_DLC_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA1 register.
//
// *****************************************************************************
pub const CAN_IF2DA1_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF2DA1_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DA2 register.
//
// *****************************************************************************
pub const CAN_IF2DA2_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF2DA2_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB1 register.
//
// *****************************************************************************
pub const CAN_IF2DB1_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF2DB1_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_IF2DB2 register.
//
// *****************************************************************************
pub const CAN_IF2DB2_DATA_M: usize = 0x0000FFFF; // Data
pub const CAN_IF2DB2_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ1 register.
//
// *****************************************************************************
pub const CAN_TXRQ1_TXRQST_M: usize = 0x0000FFFF; // Transmission Request Bits
pub const CAN_TXRQ1_TXRQST_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_TXRQ2 register.
//
// *****************************************************************************
pub const CAN_TXRQ2_TXRQST_M: usize = 0x0000FFFF; // Transmission Request Bits
pub const CAN_TXRQ2_TXRQST_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA1 register.
//
// *****************************************************************************
pub const CAN_NWDA1_NEWDAT_M: usize = 0x0000FFFF; // New Data Bits
pub const CAN_NWDA1_NEWDAT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_NWDA2 register.
//
// *****************************************************************************
pub const CAN_NWDA2_NEWDAT_M: usize = 0x0000FFFF; // New Data Bits
pub const CAN_NWDA2_NEWDAT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1INT register.
//
// *****************************************************************************
pub const CAN_MSG1INT_INTPND_M: usize = 0x0000FFFF; // Interrupt Pending Bits
pub const CAN_MSG1INT_INTPND_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2INT register.
//
// *****************************************************************************
pub const CAN_MSG2INT_INTPND_M: usize = 0x0000FFFF; // Interrupt Pending Bits
pub const CAN_MSG2INT_INTPND_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG1VAL register.
//
// *****************************************************************************
pub const CAN_MSG1VAL_MSGVAL_M: usize = 0x0000FFFF; // Message Valid Bits
pub const CAN_MSG1VAL_MSGVAL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the CAN_O_MSG2VAL register.
//
// *****************************************************************************
pub const CAN_MSG2VAL_MSGVAL_M: usize = 0x0000FFFF; // Message Valid Bits
pub const CAN_MSG2VAL_MSGVAL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FADDR register.
//
// *****************************************************************************
pub const USB_FADDR_M: usize = 0x0000007F; // Function Address
pub const USB_FADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_POWER register.
//
// *****************************************************************************
pub const USB_POWER_ISOUP: usize = 0x00000080; // Isochronous Update
pub const USB_POWER_SOFTCONN: usize = 0x00000040; // Soft Connect/Disconnect
pub const USB_POWER_RESET: usize = 0x00000008; // RESET Signaling
pub const USB_POWER_RESUME: usize = 0x00000004; // RESUME Signaling
pub const USB_POWER_SUSPEND: usize = 0x00000002; // SUSPEND Mode
pub const USB_POWER_PWRDNPHY: usize = 0x00000001; // Power Down PHY

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIS register.
//
// *****************************************************************************
pub const USB_TXIS_EP7: usize = 0x00000080; // TX Endpoint 7 Interrupt
pub const USB_TXIS_EP6: usize = 0x00000040; // TX Endpoint 6 Interrupt
pub const USB_TXIS_EP5: usize = 0x00000020; // TX Endpoint 5 Interrupt
pub const USB_TXIS_EP4: usize = 0x00000010; // TX Endpoint 4 Interrupt
pub const USB_TXIS_EP3: usize = 0x00000008; // TX Endpoint 3 Interrupt
pub const USB_TXIS_EP2: usize = 0x00000004; // TX Endpoint 2 Interrupt
pub const USB_TXIS_EP1: usize = 0x00000002; // TX Endpoint 1 Interrupt
pub const USB_TXIS_EP0: usize = 0x00000001; // TX and RX Endpoint 0 Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIS register.
//
// *****************************************************************************
pub const USB_RXIS_EP7: usize = 0x00000080; // RX Endpoint 7 Interrupt
pub const USB_RXIS_EP6: usize = 0x00000040; // RX Endpoint 6 Interrupt
pub const USB_RXIS_EP5: usize = 0x00000020; // RX Endpoint 5 Interrupt
pub const USB_RXIS_EP4: usize = 0x00000010; // RX Endpoint 4 Interrupt
pub const USB_RXIS_EP3: usize = 0x00000008; // RX Endpoint 3 Interrupt
pub const USB_RXIS_EP2: usize = 0x00000004; // RX Endpoint 2 Interrupt
pub const USB_RXIS_EP1: usize = 0x00000002; // RX Endpoint 1 Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXIE register.
//
// *****************************************************************************
pub const USB_TXIE_EP7: usize = 0x00000080; // TX Endpoint 7 Interrupt Enable
pub const USB_TXIE_EP6: usize = 0x00000040; // TX Endpoint 6 Interrupt Enable
pub const USB_TXIE_EP5: usize = 0x00000020; // TX Endpoint 5 Interrupt Enable
pub const USB_TXIE_EP4: usize = 0x00000010; // TX Endpoint 4 Interrupt Enable
pub const USB_TXIE_EP3: usize = 0x00000008; // TX Endpoint 3 Interrupt Enable
pub const USB_TXIE_EP2: usize = 0x00000004; // TX Endpoint 2 Interrupt Enable
pub const USB_TXIE_EP1: usize = 0x00000002; // TX Endpoint 1 Interrupt Enable
pub const USB_TXIE_EP0: usize = 0x00000001; // TX and RX Endpoint 0 Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXIE register.
//
// *****************************************************************************
pub const USB_RXIE_EP7: usize = 0x00000080; // RX Endpoint 7 Interrupt Enable
pub const USB_RXIE_EP6: usize = 0x00000040; // RX Endpoint 6 Interrupt Enable
pub const USB_RXIE_EP5: usize = 0x00000020; // RX Endpoint 5 Interrupt Enable
pub const USB_RXIE_EP4: usize = 0x00000010; // RX Endpoint 4 Interrupt Enable
pub const USB_RXIE_EP3: usize = 0x00000008; // RX Endpoint 3 Interrupt Enable
pub const USB_RXIE_EP2: usize = 0x00000004; // RX Endpoint 2 Interrupt Enable
pub const USB_RXIE_EP1: usize = 0x00000002; // RX Endpoint 1 Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IS register.
//
// *****************************************************************************
pub const USB_IS_DISCON: usize = 0x00000020; // Session Disconnect
pub const USB_IS_SOF: usize = 0x00000008; // Start of Frame
pub const USB_IS_RESET: usize = 0x00000004; // RESET Signaling Detected
pub const USB_IS_RESUME: usize = 0x00000002; // RESUME Signaling Detected
pub const USB_IS_SUSPEND: usize = 0x00000001; // SUSPEND Signaling Detected

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_IE register.
//
// *****************************************************************************
pub const USB_IE_DISCON: usize = 0x00000020; // Enable Disconnect Interrupt
pub const USB_IE_SOF: usize = 0x00000008; // Enable Start-of-Frame Interrupt
pub const USB_IE_RESET: usize = 0x00000004; // Enable RESET Interrupt
pub const USB_IE_RESUME: usize = 0x00000002; // Enable RESUME Interrupt
pub const USB_IE_SUSPND: usize = 0x00000001; // Enable SUSPEND Interrupt

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FRAME register.
//
// *****************************************************************************
pub const USB_FRAME_M: usize = 0x000007FF; // Frame Number
pub const USB_FRAME_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_EPIDX register.
//
// *****************************************************************************
pub const USB_EPIDX_EPIDX_M: usize = 0x0000000F; // Endpoint Index
pub const USB_EPIDX_EPIDX_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TEST register.
//
// *****************************************************************************
pub const USB_TEST_FIFOACC: usize = 0x00000040; // FIFO Access
pub const USB_TEST_FORCEFS: usize = 0x00000020; // Force Full-Speed Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO0 register.
//
// *****************************************************************************
pub const USB_FIFO0_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO0_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO1 register.
//
// *****************************************************************************
pub const USB_FIFO1_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO1_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO2 register.
//
// *****************************************************************************
pub const USB_FIFO2_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO2_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO3 register.
//
// *****************************************************************************
pub const USB_FIFO3_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO3_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO4 register.
//
// *****************************************************************************
pub const USB_FIFO4_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO4_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO5 register.
//
// *****************************************************************************
pub const USB_FIFO5_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO5_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO6 register.
//
// *****************************************************************************
pub const USB_FIFO6_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO6_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FIFO7 register.
//
// *****************************************************************************
pub const USB_FIFO7_EPDATA_M: usize = 0xFFFFFFFF; // Endpoint Data
pub const USB_FIFO7_EPDATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOSZ register.
//
// *****************************************************************************
pub const USB_TXFIFOSZ_DPB: usize = 0x00000010; // Double Packet Buffer Support
pub const USB_TXFIFOSZ_SIZE_M: usize = 0x0000000F; // Max Packet Size
pub const USB_TXFIFOSZ_SIZE_8: usize = 0x00000000; // 8
pub const USB_TXFIFOSZ_SIZE_16: usize = 0x00000001; // 16
pub const USB_TXFIFOSZ_SIZE_32: usize = 0x00000002; // 32
pub const USB_TXFIFOSZ_SIZE_64: usize = 0x00000003; // 64
pub const USB_TXFIFOSZ_SIZE_128: usize = 0x00000004; // 128
pub const USB_TXFIFOSZ_SIZE_256: usize = 0x00000005; // 256
pub const USB_TXFIFOSZ_SIZE_512: usize = 0x00000006; // 512
pub const USB_TXFIFOSZ_SIZE_1024: usize = 0x00000007; // 1024
pub const USB_TXFIFOSZ_SIZE_2048: usize = 0x00000008; // 2048

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOSZ register.
//
// *****************************************************************************
pub const USB_RXFIFOSZ_DPB: usize = 0x00000010; // Double Packet Buffer Support
pub const USB_RXFIFOSZ_SIZE_M: usize = 0x0000000F; // Max Packet Size
pub const USB_RXFIFOSZ_SIZE_8: usize = 0x00000000; // 8
pub const USB_RXFIFOSZ_SIZE_16: usize = 0x00000001; // 16
pub const USB_RXFIFOSZ_SIZE_32: usize = 0x00000002; // 32
pub const USB_RXFIFOSZ_SIZE_64: usize = 0x00000003; // 64
pub const USB_RXFIFOSZ_SIZE_128: usize = 0x00000004; // 128
pub const USB_RXFIFOSZ_SIZE_256: usize = 0x00000005; // 256
pub const USB_RXFIFOSZ_SIZE_512: usize = 0x00000006; // 512
pub const USB_RXFIFOSZ_SIZE_1024: usize = 0x00000007; // 1024
pub const USB_RXFIFOSZ_SIZE_2048: usize = 0x00000008; // 2048

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXFIFOADD
// register.
//
// *****************************************************************************
pub const USB_TXFIFOADD_ADDR_M: usize = 0x000001FF; // Transmit/Receive Start Address
pub const USB_TXFIFOADD_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXFIFOADD
// register.
//
// *****************************************************************************
pub const USB_RXFIFOADD_ADDR_M: usize = 0x000001FF; // Transmit/Receive Start Address
pub const USB_RXFIFOADD_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CONTIM register.
//
// *****************************************************************************
pub const USB_CONTIM_WTCON_M: usize = 0x000000F0; // Connect Wait
pub const USB_CONTIM_WTID_M: usize = 0x0000000F; // Wait ID
pub const USB_CONTIM_WTCON_S: usize = 4;
pub const USB_CONTIM_WTID_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_FSEOF register.
//
// *****************************************************************************
pub const USB_FSEOF_FSEOFG_M: usize = 0x000000FF; // Full-Speed End-of-Frame Gap
pub const USB_FSEOF_FSEOFG_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_LSEOF register.
//
// *****************************************************************************
pub const USB_LSEOF_LSEOFG_M: usize = 0x000000FF; // Low-Speed End-of-Frame Gap
pub const USB_LSEOF_LSEOFG_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRL0 register.
//
// *****************************************************************************
pub const USB_CSRL0_SETENDC: usize = 0x00000080; // Setup End Clear
pub const USB_CSRL0_RXRDYC: usize = 0x00000040; // RXRDY Clear
pub const USB_CSRL0_STALL: usize = 0x00000020; // Send Stall
pub const USB_CSRL0_SETEND: usize = 0x00000010; // Setup End
pub const USB_CSRL0_DATAEND: usize = 0x00000008; // Data End
pub const USB_CSRL0_STALLED: usize = 0x00000004; // Endpoint Stalled
pub const USB_CSRL0_TXRDY: usize = 0x00000002; // Transmit Packet Ready
pub const USB_CSRL0_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_CSRH0 register.
//
// *****************************************************************************
pub const USB_CSRH0_FLUSH: usize = 0x00000001; // Flush FIFO

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_COUNT0 register.
//
// *****************************************************************************
pub const USB_COUNT0_COUNT_M: usize = 0x0000007F; // FIFO Count
pub const USB_COUNT0_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP1 register.
//
// *****************************************************************************
pub const USB_TXMAXP1_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_TXMAXP1_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL1 register.
//
// *****************************************************************************
pub const USB_TXCSRL1_CLRDT: usize = 0x00000040; // Clear Data Toggle
pub const USB_TXCSRL1_STALLED: usize = 0x00000020; // Endpoint Stalled
pub const USB_TXCSRL1_STALL: usize = 0x00000010; // Send STALL
pub const USB_TXCSRL1_FLUSH: usize = 0x00000008; // Flush FIFO
pub const USB_TXCSRL1_UNDRN: usize = 0x00000004; // Underrun
pub const USB_TXCSRL1_FIFONE: usize = 0x00000002; // FIFO Not Empty
pub const USB_TXCSRL1_TXRDY: usize = 0x00000001; // Transmit Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH1 register.
//
// *****************************************************************************
pub const USB_TXCSRH1_AUTOSET: usize = 0x00000080; // Auto Set
pub const USB_TXCSRH1_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_TXCSRH1_MODE: usize = 0x00000020; // Mode
pub const USB_TXCSRH1_DMAEN: usize = 0x00000010; // DMA Request Enable
pub const USB_TXCSRH1_FDT: usize = 0x00000008; // Force Data Toggle
pub const USB_TXCSRH1_DMAMOD: usize = 0x00000004; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP1 register.
//
// *****************************************************************************
pub const USB_RXMAXP1_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_RXMAXP1_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL1 register.
//
// *****************************************************************************
pub const USB_RXCSRL1_CLRDT: usize = 0x00000080; // Clear Data Toggle
pub const USB_RXCSRL1_STALLED: usize = 0x00000040; // Endpoint Stalled
pub const USB_RXCSRL1_STALL: usize = 0x00000020; // Send STALL
pub const USB_RXCSRL1_FLUSH: usize = 0x00000010; // Flush FIFO
pub const USB_RXCSRL1_DATAERR: usize = 0x00000008; // Data Error
pub const USB_RXCSRL1_OVER: usize = 0x00000004; // Overrun
pub const USB_RXCSRL1_FULL: usize = 0x00000002; // FIFO Full
pub const USB_RXCSRL1_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH1 register.
//
// *****************************************************************************
pub const USB_RXCSRH1_AUTOCL: usize = 0x00000080; // Auto Clear
pub const USB_RXCSRH1_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_RXCSRH1_DMAEN: usize = 0x00000020; // DMA Request Enable
pub const USB_RXCSRH1_DISNYET: usize = 0x00000010; // Disable NYET
pub const USB_RXCSRH1_PIDERR: usize = 0x00000010; // PID Error
pub const USB_RXCSRH1_DMAMOD: usize = 0x00000008; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT1 register.
//
// *****************************************************************************
pub const USB_RXCOUNT1_COUNT_M: usize = 0x00001FFF; // Receive Packet Count
pub const USB_RXCOUNT1_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP2 register.
//
// *****************************************************************************
pub const USB_TXMAXP2_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_TXMAXP2_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL2 register.
//
// *****************************************************************************
pub const USB_TXCSRL2_CLRDT: usize = 0x00000040; // Clear Data Toggle
pub const USB_TXCSRL2_STALLED: usize = 0x00000020; // Endpoint Stalled
pub const USB_TXCSRL2_STALL: usize = 0x00000010; // Send STALL
pub const USB_TXCSRL2_FLUSH: usize = 0x00000008; // Flush FIFO
pub const USB_TXCSRL2_UNDRN: usize = 0x00000004; // Underrun
pub const USB_TXCSRL2_FIFONE: usize = 0x00000002; // FIFO Not Empty
pub const USB_TXCSRL2_TXRDY: usize = 0x00000001; // Transmit Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH2 register.
//
// *****************************************************************************
pub const USB_TXCSRH2_AUTOSET: usize = 0x00000080; // Auto Set
pub const USB_TXCSRH2_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_TXCSRH2_MODE: usize = 0x00000020; // Mode
pub const USB_TXCSRH2_DMAEN: usize = 0x00000010; // DMA Request Enable
pub const USB_TXCSRH2_FDT: usize = 0x00000008; // Force Data Toggle
pub const USB_TXCSRH2_DMAMOD: usize = 0x00000004; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP2 register.
//
// *****************************************************************************
pub const USB_RXMAXP2_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_RXMAXP2_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL2 register.
//
// *****************************************************************************
pub const USB_RXCSRL2_CLRDT: usize = 0x00000080; // Clear Data Toggle
pub const USB_RXCSRL2_STALLED: usize = 0x00000040; // Endpoint Stalled
pub const USB_RXCSRL2_STALL: usize = 0x00000020; // Send STALL
pub const USB_RXCSRL2_FLUSH: usize = 0x00000010; // Flush FIFO
pub const USB_RXCSRL2_DATAERR: usize = 0x00000008; // Data Error
pub const USB_RXCSRL2_OVER: usize = 0x00000004; // Overrun
pub const USB_RXCSRL2_FULL: usize = 0x00000002; // FIFO Full
pub const USB_RXCSRL2_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH2 register.
//
// *****************************************************************************
pub const USB_RXCSRH2_AUTOCL: usize = 0x00000080; // Auto Clear
pub const USB_RXCSRH2_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_RXCSRH2_DMAEN: usize = 0x00000020; // DMA Request Enable
pub const USB_RXCSRH2_DISNYET: usize = 0x00000010; // Disable NYET
pub const USB_RXCSRH2_PIDERR: usize = 0x00000010; // PID Error
pub const USB_RXCSRH2_DMAMOD: usize = 0x00000008; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT2 register.
//
// *****************************************************************************
pub const USB_RXCOUNT2_COUNT_M: usize = 0x00001FFF; // Receive Packet Count
pub const USB_RXCOUNT2_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP3 register.
//
// *****************************************************************************
pub const USB_TXMAXP3_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_TXMAXP3_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL3 register.
//
// *****************************************************************************
pub const USB_TXCSRL3_CLRDT: usize = 0x00000040; // Clear Data Toggle
pub const USB_TXCSRL3_STALLED: usize = 0x00000020; // Endpoint Stalled
pub const USB_TXCSRL3_STALL: usize = 0x00000010; // Send STALL
pub const USB_TXCSRL3_FLUSH: usize = 0x00000008; // Flush FIFO
pub const USB_TXCSRL3_UNDRN: usize = 0x00000004; // Underrun
pub const USB_TXCSRL3_FIFONE: usize = 0x00000002; // FIFO Not Empty
pub const USB_TXCSRL3_TXRDY: usize = 0x00000001; // Transmit Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH3 register.
//
// *****************************************************************************
pub const USB_TXCSRH3_AUTOSET: usize = 0x00000080; // Auto Set
pub const USB_TXCSRH3_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_TXCSRH3_MODE: usize = 0x00000020; // Mode
pub const USB_TXCSRH3_DMAEN: usize = 0x00000010; // DMA Request Enable
pub const USB_TXCSRH3_FDT: usize = 0x00000008; // Force Data Toggle
pub const USB_TXCSRH3_DMAMOD: usize = 0x00000004; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP3 register.
//
// *****************************************************************************
pub const USB_RXMAXP3_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_RXMAXP3_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL3 register.
//
// *****************************************************************************
pub const USB_RXCSRL3_CLRDT: usize = 0x00000080; // Clear Data Toggle
pub const USB_RXCSRL3_STALLED: usize = 0x00000040; // Endpoint Stalled
pub const USB_RXCSRL3_STALL: usize = 0x00000020; // Send STALL
pub const USB_RXCSRL3_FLUSH: usize = 0x00000010; // Flush FIFO
pub const USB_RXCSRL3_DATAERR: usize = 0x00000008; // Data Error
pub const USB_RXCSRL3_OVER: usize = 0x00000004; // Overrun
pub const USB_RXCSRL3_FULL: usize = 0x00000002; // FIFO Full
pub const USB_RXCSRL3_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH3 register.
//
// *****************************************************************************
pub const USB_RXCSRH3_AUTOCL: usize = 0x00000080; // Auto Clear
pub const USB_RXCSRH3_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_RXCSRH3_DMAEN: usize = 0x00000020; // DMA Request Enable
pub const USB_RXCSRH3_DISNYET: usize = 0x00000010; // Disable NYET
pub const USB_RXCSRH3_PIDERR: usize = 0x00000010; // PID Error
pub const USB_RXCSRH3_DMAMOD: usize = 0x00000008; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT3 register.
//
// *****************************************************************************
pub const USB_RXCOUNT3_COUNT_M: usize = 0x00001FFF; // Receive Packet Count
pub const USB_RXCOUNT3_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP4 register.
//
// *****************************************************************************
pub const USB_TXMAXP4_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_TXMAXP4_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL4 register.
//
// *****************************************************************************
pub const USB_TXCSRL4_CLRDT: usize = 0x00000040; // Clear Data Toggle
pub const USB_TXCSRL4_STALLED: usize = 0x00000020; // Endpoint Stalled
pub const USB_TXCSRL4_STALL: usize = 0x00000010; // Send STALL
pub const USB_TXCSRL4_FLUSH: usize = 0x00000008; // Flush FIFO
pub const USB_TXCSRL4_UNDRN: usize = 0x00000004; // Underrun
pub const USB_TXCSRL4_FIFONE: usize = 0x00000002; // FIFO Not Empty
pub const USB_TXCSRL4_TXRDY: usize = 0x00000001; // Transmit Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH4 register.
//
// *****************************************************************************
pub const USB_TXCSRH4_AUTOSET: usize = 0x00000080; // Auto Set
pub const USB_TXCSRH4_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_TXCSRH4_MODE: usize = 0x00000020; // Mode
pub const USB_TXCSRH4_DMAEN: usize = 0x00000010; // DMA Request Enable
pub const USB_TXCSRH4_FDT: usize = 0x00000008; // Force Data Toggle
pub const USB_TXCSRH4_DMAMOD: usize = 0x00000004; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP4 register.
//
// *****************************************************************************
pub const USB_RXMAXP4_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_RXMAXP4_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL4 register.
//
// *****************************************************************************
pub const USB_RXCSRL4_CLRDT: usize = 0x00000080; // Clear Data Toggle
pub const USB_RXCSRL4_STALLED: usize = 0x00000040; // Endpoint Stalled
pub const USB_RXCSRL4_STALL: usize = 0x00000020; // Send STALL
pub const USB_RXCSRL4_FLUSH: usize = 0x00000010; // Flush FIFO
pub const USB_RXCSRL4_DATAERR: usize = 0x00000008; // Data Error
pub const USB_RXCSRL4_OVER: usize = 0x00000004; // Overrun
pub const USB_RXCSRL4_FULL: usize = 0x00000002; // FIFO Full
pub const USB_RXCSRL4_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH4 register.
//
// *****************************************************************************
pub const USB_RXCSRH4_AUTOCL: usize = 0x00000080; // Auto Clear
pub const USB_RXCSRH4_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_RXCSRH4_DMAEN: usize = 0x00000020; // DMA Request Enable
pub const USB_RXCSRH4_DISNYET: usize = 0x00000010; // Disable NYET
pub const USB_RXCSRH4_PIDERR: usize = 0x00000010; // PID Error
pub const USB_RXCSRH4_DMAMOD: usize = 0x00000008; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT4 register.
//
// *****************************************************************************
pub const USB_RXCOUNT4_COUNT_M: usize = 0x00001FFF; // Receive Packet Count
pub const USB_RXCOUNT4_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP5 register.
//
// *****************************************************************************
pub const USB_TXMAXP5_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_TXMAXP5_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL5 register.
//
// *****************************************************************************
pub const USB_TXCSRL5_CLRDT: usize = 0x00000040; // Clear Data Toggle
pub const USB_TXCSRL5_STALLED: usize = 0x00000020; // Endpoint Stalled
pub const USB_TXCSRL5_STALL: usize = 0x00000010; // Send STALL
pub const USB_TXCSRL5_FLUSH: usize = 0x00000008; // Flush FIFO
pub const USB_TXCSRL5_UNDRN: usize = 0x00000004; // Underrun
pub const USB_TXCSRL5_FIFONE: usize = 0x00000002; // FIFO Not Empty
pub const USB_TXCSRL5_TXRDY: usize = 0x00000001; // Transmit Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH5 register.
//
// *****************************************************************************
pub const USB_TXCSRH5_AUTOSET: usize = 0x00000080; // Auto Set
pub const USB_TXCSRH5_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_TXCSRH5_MODE: usize = 0x00000020; // Mode
pub const USB_TXCSRH5_DMAEN: usize = 0x00000010; // DMA Request Enable
pub const USB_TXCSRH5_FDT: usize = 0x00000008; // Force Data Toggle
pub const USB_TXCSRH5_DMAMOD: usize = 0x00000004; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP5 register.
//
// *****************************************************************************
pub const USB_RXMAXP5_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_RXMAXP5_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL5 register.
//
// *****************************************************************************
pub const USB_RXCSRL5_CLRDT: usize = 0x00000080; // Clear Data Toggle
pub const USB_RXCSRL5_STALLED: usize = 0x00000040; // Endpoint Stalled
pub const USB_RXCSRL5_STALL: usize = 0x00000020; // Send STALL
pub const USB_RXCSRL5_FLUSH: usize = 0x00000010; // Flush FIFO
pub const USB_RXCSRL5_DATAERR: usize = 0x00000008; // Data Error
pub const USB_RXCSRL5_OVER: usize = 0x00000004; // Overrun
pub const USB_RXCSRL5_FULL: usize = 0x00000002; // FIFO Full
pub const USB_RXCSRL5_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH5 register.
//
// *****************************************************************************
pub const USB_RXCSRH5_AUTOCL: usize = 0x00000080; // Auto Clear
pub const USB_RXCSRH5_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_RXCSRH5_DMAEN: usize = 0x00000020; // DMA Request Enable
pub const USB_RXCSRH5_DISNYET: usize = 0x00000010; // Disable NYET
pub const USB_RXCSRH5_PIDERR: usize = 0x00000010; // PID Error
pub const USB_RXCSRH5_DMAMOD: usize = 0x00000008; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT5 register.
//
// *****************************************************************************
pub const USB_RXCOUNT5_COUNT_M: usize = 0x00001FFF; // Receive Packet Count
pub const USB_RXCOUNT5_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP6 register.
//
// *****************************************************************************
pub const USB_TXMAXP6_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_TXMAXP6_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL6 register.
//
// *****************************************************************************
pub const USB_TXCSRL6_CLRDT: usize = 0x00000040; // Clear Data Toggle
pub const USB_TXCSRL6_STALLED: usize = 0x00000020; // Endpoint Stalled
pub const USB_TXCSRL6_STALL: usize = 0x00000010; // Send STALL
pub const USB_TXCSRL6_FLUSH: usize = 0x00000008; // Flush FIFO
pub const USB_TXCSRL6_UNDRN: usize = 0x00000004; // Underrun
pub const USB_TXCSRL6_FIFONE: usize = 0x00000002; // FIFO Not Empty
pub const USB_TXCSRL6_TXRDY: usize = 0x00000001; // Transmit Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH6 register.
//
// *****************************************************************************
pub const USB_TXCSRH6_AUTOSET: usize = 0x00000080; // Auto Set
pub const USB_TXCSRH6_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_TXCSRH6_MODE: usize = 0x00000020; // Mode
pub const USB_TXCSRH6_DMAEN: usize = 0x00000010; // DMA Request Enable
pub const USB_TXCSRH6_FDT: usize = 0x00000008; // Force Data Toggle
pub const USB_TXCSRH6_DMAMOD: usize = 0x00000004; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP6 register.
//
// *****************************************************************************
pub const USB_RXMAXP6_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_RXMAXP6_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL6 register.
//
// *****************************************************************************
pub const USB_RXCSRL6_CLRDT: usize = 0x00000080; // Clear Data Toggle
pub const USB_RXCSRL6_STALLED: usize = 0x00000040; // Endpoint Stalled
pub const USB_RXCSRL6_STALL: usize = 0x00000020; // Send STALL
pub const USB_RXCSRL6_FLUSH: usize = 0x00000010; // Flush FIFO
pub const USB_RXCSRL6_DATAERR: usize = 0x00000008; // Data Error
pub const USB_RXCSRL6_OVER: usize = 0x00000004; // Overrun
pub const USB_RXCSRL6_FULL: usize = 0x00000002; // FIFO Full
pub const USB_RXCSRL6_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH6 register.
//
// *****************************************************************************
pub const USB_RXCSRH6_AUTOCL: usize = 0x00000080; // Auto Clear
pub const USB_RXCSRH6_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_RXCSRH6_DMAEN: usize = 0x00000020; // DMA Request Enable
pub const USB_RXCSRH6_DISNYET: usize = 0x00000010; // Disable NYET
pub const USB_RXCSRH6_PIDERR: usize = 0x00000010; // PID Error
pub const USB_RXCSRH6_DMAMOD: usize = 0x00000008; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT6 register.
//
// *****************************************************************************
pub const USB_RXCOUNT6_COUNT_M: usize = 0x00001FFF; // Receive Packet Count
pub const USB_RXCOUNT6_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXMAXP7 register.
//
// *****************************************************************************
pub const USB_TXMAXP7_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_TXMAXP7_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRL7 register.
//
// *****************************************************************************
pub const USB_TXCSRL7_CLRDT: usize = 0x00000040; // Clear Data Toggle
pub const USB_TXCSRL7_STALLED: usize = 0x00000020; // Endpoint Stalled
pub const USB_TXCSRL7_STALL: usize = 0x00000010; // Send STALL
pub const USB_TXCSRL7_FLUSH: usize = 0x00000008; // Flush FIFO
pub const USB_TXCSRL7_UNDRN: usize = 0x00000004; // Underrun
pub const USB_TXCSRL7_FIFONE: usize = 0x00000002; // FIFO Not Empty
pub const USB_TXCSRL7_TXRDY: usize = 0x00000001; // Transmit Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXCSRH7 register.
//
// *****************************************************************************
pub const USB_TXCSRH7_AUTOSET: usize = 0x00000080; // Auto Set
pub const USB_TXCSRH7_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_TXCSRH7_MODE: usize = 0x00000020; // Mode
pub const USB_TXCSRH7_DMAEN: usize = 0x00000010; // DMA Request Enable
pub const USB_TXCSRH7_FDT: usize = 0x00000008; // Force Data Toggle
pub const USB_TXCSRH7_DMAMOD: usize = 0x00000004; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXMAXP7 register.
//
// *****************************************************************************
pub const USB_RXMAXP7_MAXLOAD_M: usize = 0x000007FF; // Maximum Payload
pub const USB_RXMAXP7_MAXLOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRL7 register.
//
// *****************************************************************************
pub const USB_RXCSRL7_CLRDT: usize = 0x00000080; // Clear Data Toggle
pub const USB_RXCSRL7_STALLED: usize = 0x00000040; // Endpoint Stalled
pub const USB_RXCSRL7_STALL: usize = 0x00000020; // Send STALL
pub const USB_RXCSRL7_FLUSH: usize = 0x00000010; // Flush FIFO
pub const USB_RXCSRL7_DATAERR: usize = 0x00000008; // Data Error
pub const USB_RXCSRL7_OVER: usize = 0x00000004; // Overrun
pub const USB_RXCSRL7_FULL: usize = 0x00000002; // FIFO Full
pub const USB_RXCSRL7_RXRDY: usize = 0x00000001; // Receive Packet Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCSRH7 register.
//
// *****************************************************************************
pub const USB_RXCSRH7_AUTOCL: usize = 0x00000080; // Auto Clear
pub const USB_RXCSRH7_ISO: usize = 0x00000040; // Isochronous Transfers
pub const USB_RXCSRH7_DMAEN: usize = 0x00000020; // DMA Request Enable
pub const USB_RXCSRH7_PIDERR: usize = 0x00000010; // PID Error
pub const USB_RXCSRH7_DISNYET: usize = 0x00000010; // Disable NYET
pub const USB_RXCSRH7_DMAMOD: usize = 0x00000008; // DMA Request Mode

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXCOUNT7 register.
//
// *****************************************************************************
pub const USB_RXCOUNT7_COUNT_M: usize = 0x00001FFF; // Receive Packet Count
pub const USB_RXCOUNT7_COUNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_RXDPKTBUFDIS
// register.
//
// *****************************************************************************
pub const USB_RXDPKTBUFDIS_EP7: usize = 0x00000080; // EP7 RX Double-Packet Buffer Disable
pub const USB_RXDPKTBUFDIS_EP6: usize = 0x00000040; // EP6 RX Double-Packet Buffer Disable
pub const USB_RXDPKTBUFDIS_EP5: usize = 0x00000020; // EP5 RX Double-Packet Buffer Disable
pub const USB_RXDPKTBUFDIS_EP4: usize = 0x00000010; // EP4 RX Double-Packet Buffer Disable
pub const USB_RXDPKTBUFDIS_EP3: usize = 0x00000008; // EP3 RX Double-Packet Buffer Disable
pub const USB_RXDPKTBUFDIS_EP2: usize = 0x00000004; // EP2 RX Double-Packet Buffer Disable
pub const USB_RXDPKTBUFDIS_EP1: usize = 0x00000002; // EP1 RX Double-Packet Buffer Disable

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_TXDPKTBUFDIS
// register.
//
// *****************************************************************************
pub const USB_TXDPKTBUFDIS_EP7: usize = 0x00000080; // EP7 TX Double-Packet Buffer Disable
pub const USB_TXDPKTBUFDIS_EP6: usize = 0x00000040; // EP6 TX Double-Packet Buffer Disable
pub const USB_TXDPKTBUFDIS_EP5: usize = 0x00000020; // EP5 TX Double-Packet Buffer Disable
pub const USB_TXDPKTBUFDIS_EP4: usize = 0x00000010; // EP4 TX Double-Packet Buffer Disable
pub const USB_TXDPKTBUFDIS_EP3: usize = 0x00000008; // EP3 TX Double-Packet Buffer Disable
pub const USB_TXDPKTBUFDIS_EP2: usize = 0x00000004; // EP2 TX Double-Packet Buffer Disable
pub const USB_TXDPKTBUFDIS_EP1: usize = 0x00000002; // EP1 TX Double-Packet Buffer Disable

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRRIS register.
//
// *****************************************************************************
pub const USB_DRRIS_RESUME: usize = 0x00000001; // RESUME Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRIM register.
//
// *****************************************************************************
pub const USB_DRIM_RESUME: usize = 0x00000001; // RESUME Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DRISC register.
//
// *****************************************************************************
pub const USB_DRISC_RESUME: usize = 0x00000001; // RESUME Interrupt Status and Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_DMASEL register.
//
// *****************************************************************************
pub const USB_DMASEL_DMACTX_M: usize = 0x00F00000; // DMA C TX Select
pub const USB_DMASEL_DMACRX_M: usize = 0x000F0000; // DMA C RX Select
pub const USB_DMASEL_DMABTX_M: usize = 0x0000F000; // DMA B TX Select
pub const USB_DMASEL_DMABRX_M: usize = 0x00000F00; // DMA B RX Select
pub const USB_DMASEL_DMAATX_M: usize = 0x000000F0; // DMA A TX Select
pub const USB_DMASEL_DMAARX_M: usize = 0x0000000F; // DMA A RX Select
pub const USB_DMASEL_DMACTX_S: usize = 20;
pub const USB_DMASEL_DMACRX_S: usize = 16;
pub const USB_DMASEL_DMABTX_S: usize = 12;
pub const USB_DMASEL_DMABRX_S: usize = 8;
pub const USB_DMASEL_DMAATX_S: usize = 4;
pub const USB_DMASEL_DMAARX_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the USB_O_PP register.
//
// *****************************************************************************
pub const USB_PP_ECNT_M: usize = 0x0000FF00; // Endpoint Count
pub const USB_PP_USB_M: usize = 0x000000C0; // USB Capability
pub const USB_PP_USB_DEVICE: usize = 0x00000040; // DEVICE
pub const USB_PP_USB_HOSTDEVICE: usize = 0x00000080; // HOST
pub const USB_PP_USB_OTG: usize = 0x000000C0; // OTG
pub const USB_PP_PHY: usize = 0x00000010; // PHY Present
pub const USB_PP_TYPE_M: usize = 0x0000000F; // Controller Type
pub const USB_PP_TYPE_0: usize = 0x00000000; // The first-generation USB controller
pub const USB_PP_ECNT_S: usize = 8;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESIZE register.
//
// *****************************************************************************
pub const EEPROM_EESIZE_BLKCNT_M: usize = 0x07FF0000; // Number of 16-Word Blocks
pub const EEPROM_EESIZE_WORDCNT_M: usize = 0x0000FFFF; // Number of 32-Bit Words
pub const EEPROM_EESIZE_BLKCNT_S: usize = 16;
pub const EEPROM_EESIZE_WORDCNT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEBLOCK register.
//
// *****************************************************************************
pub const EEPROM_EEBLOCK_BLOCK_M: usize = 0x0000FFFF; // Current Block
pub const EEPROM_EEBLOCK_BLOCK_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEOFFSET
// register.
//
// *****************************************************************************
pub const EEPROM_EEOFFSET_OFFSET_M: usize = 0x0000000F; // Current Address Offset
pub const EEPROM_EEOFFSET_OFFSET_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWR register.
//
// *****************************************************************************
pub const EEPROM_EERDWR_VALUE_M: usize = 0xFFFFFFFF; // EEPROM Read or Write Data
pub const EEPROM_EERDWR_VALUE_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EERDWRINC
// register.
//
// *****************************************************************************
pub const EEPROM_EERDWRINC_VALUE_M: usize = 0xFFFFFFFF; // EEPROM Read or Write Data with Increment
pub const EEPROM_EERDWRINC_VALUE_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDONE register.
//
// *****************************************************************************
pub const EEPROM_EEDONE_INVPL: usize = 0x00000100; // Invalid Program Voltage Level
pub const EEPROM_EEDONE_WRBUSY: usize = 0x00000020; // Write Busy
pub const EEPROM_EEDONE_NOPERM: usize = 0x00000010; // Write Without Permission
pub const EEPROM_EEDONE_WKCOPY: usize = 0x00000008; // Working on a Copy
pub const EEPROM_EEDONE_WKERASE: usize = 0x00000004; // Working on an Erase
pub const EEPROM_EEDONE_WORKING: usize = 0x00000001; // EEPROM Working

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EESUPP register.
//
// *****************************************************************************
pub const EEPROM_EESUPP_PRETRY: usize = 0x00000008; // Programming Must Be Retried
pub const EEPROM_EESUPP_ERETRY: usize = 0x00000004; // Erase Must Be Retried
pub const EEPROM_EESUPP_EREQ: usize = 0x00000002; // Erase Required
pub const EEPROM_EESUPP_START: usize = 0x00000001; // Start Erase

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEUNLOCK
// register.
//
// *****************************************************************************
pub const EEPROM_EEUNLOCK_UNLOCK_M: usize = 0xFFFFFFFF; // EEPROM Unlock

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPROT register.
//
// *****************************************************************************
pub const EEPROM_EEPROT_ACC: usize = 0x00000008; // Access Control
pub const EEPROM_EEPROT_PROT_M: usize = 0x00000007; // Protection Control
pub const EEPROM_EEPROT_PROT_RWNPW: usize = 0x00000000; // This setting is the default. If there is no password, the block is not protected and is readable and writable
pub const EEPROM_EEPROT_PROT_RWPW: usize = 0x00000001; // If there is a password, the block is readable or writable only when unlocked
pub const EEPROM_EEPROT_PROT_RONPW: usize = 0x00000002; // If there is no password, the block is readable, not writable

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS0 register.
//
// *****************************************************************************
pub const EEPROM_EEPASS0_PASS_M: usize = 0xFFFFFFFF; // Password
pub const EEPROM_EEPASS0_PASS_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS1 register.
//
// *****************************************************************************
pub const EEPROM_EEPASS1_PASS_M: usize = 0xFFFFFFFF; // Password
pub const EEPROM_EEPASS1_PASS_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEPASS2 register.
//
// *****************************************************************************
pub const EEPROM_EEPASS2_PASS_M: usize = 0xFFFFFFFF; // Password
pub const EEPROM_EEPASS2_PASS_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEINT register.
//
// *****************************************************************************
pub const EEPROM_EEINT_INT: usize = 0x00000001; // Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEHIDE register.
//
// *****************************************************************************
pub const EEPROM_EEHIDE_HN_M: usize = 0xFFFFFFFE; // Hide Block

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_EEDBGME register.
//
// *****************************************************************************
pub const EEPROM_EEDBGME_KEY_M: usize = 0xFFFF0000; // Erase Key
pub const EEPROM_EEDBGME_ME: usize = 0x00000001; // Mass Erase
pub const EEPROM_EEDBGME_KEY_S: usize = 16;

// *****************************************************************************
//
// The following are defines for the bit fields in the EEPROM_PP register.
//
// *****************************************************************************
pub const EEPROM_PP_SIZE_M: usize = 0x0000001F; // EEPROM Size
pub const EEPROM_PP_SIZE_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_RIS register.
//
// *****************************************************************************
pub const SYSEXC_RIS_FPIXCRIS: usize = 0x00000020; // Floating-Point Inexact Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPOFCRIS: usize = 0x00000010; // Floating-Point Overflow Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPUFCRIS: usize = 0x00000008; // Floating-Point Underflow Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPIOCRIS: usize = 0x00000004; // Floating-Point Invalid Operation Raw Interrupt Status
pub const SYSEXC_RIS_FPDZCRIS: usize = 0x00000002; // Floating-Point Divide By 0 Exception Raw Interrupt Status
pub const SYSEXC_RIS_FPIDCRIS: usize = 0x00000001; // Floating-Point Input Denormal Exception Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IM register.
//
// *****************************************************************************
pub const SYSEXC_IM_FPIXCIM: usize = 0x00000020; // Floating-Point Inexact Exception Interrupt Mask
pub const SYSEXC_IM_FPOFCIM: usize = 0x00000010; // Floating-Point Overflow Exception Interrupt Mask
pub const SYSEXC_IM_FPUFCIM: usize = 0x00000008; // Floating-Point Underflow Exception Interrupt Mask
pub const SYSEXC_IM_FPIOCIM: usize = 0x00000004; // Floating-Point Invalid Operation Interrupt Mask
pub const SYSEXC_IM_FPDZCIM: usize = 0x00000002; // Floating-Point Divide By 0 Exception Interrupt Mask
pub const SYSEXC_IM_FPIDCIM: usize = 0x00000001; // Floating-Point Input Denormal Exception Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_MIS register.
//
// *****************************************************************************
pub const SYSEXC_MIS_FPIXCMIS: usize = 0x00000020; // Floating-Point Inexact Exception Masked Interrupt Status
pub const SYSEXC_MIS_FPOFCMIS: usize = 0x00000010; // Floating-Point Overflow Exception Masked Interrupt Status
pub const SYSEXC_MIS_FPUFCMIS: usize = 0x00000008; // Floating-Point Underflow Exception Masked Interrupt Status
pub const SYSEXC_MIS_FPIOCMIS: usize = 0x00000004; // Floating-Point Invalid Operation Masked Interrupt Status
pub const SYSEXC_MIS_FPDZCMIS: usize = 0x00000002; // Floating-Point Divide By 0 Exception Masked Interrupt Status
pub const SYSEXC_MIS_FPIDCMIS: usize = 0x00000001; // Floating-Point Input Denormal Exception Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSEXC_IC register.
//
// *****************************************************************************
pub const SYSEXC_IC_FPIXCIC: usize = 0x00000020; // Floating-Point Inexact Exception Interrupt Clear
pub const SYSEXC_IC_FPOFCIC: usize = 0x00000010; // Floating-Point Overflow Exception Interrupt Clear
pub const SYSEXC_IC_FPUFCIC: usize = 0x00000008; // Floating-Point Underflow Exception Interrupt Clear
pub const SYSEXC_IC_FPIOCIC: usize = 0x00000004; // Floating-Point Invalid Operation Interrupt Clear
pub const SYSEXC_IC_FPDZCIC: usize = 0x00000002; // Floating-Point Divide By 0 Exception Interrupt Clear
pub const SYSEXC_IC_FPIDCIC: usize = 0x00000001; // Floating-Point Input Denormal Exception Interrupt Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCC register.
//
// *****************************************************************************
pub const HIB_RTCC_M: usize = 0xFFFFFFFF; // RTC Counter
pub const HIB_RTCC_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCM0 register.
//
// *****************************************************************************
pub const HIB_RTCM0_M: usize = 0xFFFFFFFF; // RTC Match 0
pub const HIB_RTCM0_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCLD register.
//
// *****************************************************************************
pub const HIB_RTCLD_M: usize = 0xFFFFFFFF; // RTC Load
pub const HIB_RTCLD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_CTL register.
//
// *****************************************************************************
pub const HIB_CTL_WRC: usize = 0x80000000; // Write Complete/Capable
pub const HIB_CTL_OSCDRV: usize = 0x00020000; // Oscillator Drive Capability
pub const HIB_CTL_OSCBYP: usize = 0x00010000; // Oscillator Bypass
pub const HIB_CTL_VBATSEL_M: usize = 0x00006000; // Select for Low-Battery Comparator
pub const HIB_CTL_VBATSEL_1_9V: usize = 0x00000000; // 1.9 Volts
pub const HIB_CTL_VBATSEL_2_1V: usize = 0x00002000; // 2.1 Volts (default)
pub const HIB_CTL_VBATSEL_2_3V: usize = 0x00004000; // 2.3 Volts
pub const HIB_CTL_VBATSEL_2_5V: usize = 0x00006000; // 2.5 Volts
pub const HIB_CTL_BATCHK: usize = 0x00000400; // Check Battery Status
pub const HIB_CTL_BATWKEN: usize = 0x00000200; // Wake on Low Battery
pub const HIB_CTL_VDD3ON: usize = 0x00000100; // VDD Powered
pub const HIB_CTL_VABORT: usize = 0x00000080; // Power Cut Abort Enable
pub const HIB_CTL_CLK32EN: usize = 0x00000040; // Clocking Enable
pub const HIB_CTL_LOWBATEN: usize = 0x00000020; // Low Battery Monitoring Enable
pub const HIB_CTL_PINWEN: usize = 0x00000010; // External WAKE Pin Enable
pub const HIB_CTL_RTCWEN: usize = 0x00000008; // RTC Wake-up Enable
pub const HIB_CTL_HIBREQ: usize = 0x00000002; // Hibernation Request
pub const HIB_CTL_RTCEN: usize = 0x00000001; // RTC Timer Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_IM register.
//
// *****************************************************************************
pub const HIB_IM_WC: usize = 0x00000010; // External Write Complete/Capable Interrupt Mask
pub const HIB_IM_EXTW: usize = 0x00000008; // External Wake-Up Interrupt Mask
pub const HIB_IM_LOWBAT: usize = 0x00000004; // Low Battery Voltage Interrupt Mask
pub const HIB_IM_RTCALT0: usize = 0x00000001; // RTC Alert 0 Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_RIS register.
//
// *****************************************************************************
pub const HIB_RIS_WC: usize = 0x00000010; // Write Complete/Capable Raw Interrupt Status
pub const HIB_RIS_EXTW: usize = 0x00000008; // External Wake-Up Raw Interrupt Status
pub const HIB_RIS_LOWBAT: usize = 0x00000004; // Low Battery Voltage Raw Interrupt Status
pub const HIB_RIS_RTCALT0: usize = 0x00000001; // RTC Alert 0 Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_MIS register.
//
// *****************************************************************************
pub const HIB_MIS_WC: usize = 0x00000010; // Write Complete/Capable Masked Interrupt Status
pub const HIB_MIS_EXTW: usize = 0x00000008; // External Wake-Up Masked Interrupt Status
pub const HIB_MIS_LOWBAT: usize = 0x00000004; // Low Battery Voltage Masked Interrupt Status
pub const HIB_MIS_RTCALT0: usize = 0x00000001; // RTC Alert 0 Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_IC register.
//
// *****************************************************************************
pub const HIB_IC_WC: usize = 0x00000010; // Write Complete/Capable Masked Interrupt Clear
pub const HIB_IC_EXTW: usize = 0x00000008; // External Wake-Up Masked Interrupt Clear
pub const HIB_IC_LOWBAT: usize = 0x00000004; // Low Battery Voltage Masked Interrupt Clear
pub const HIB_IC_RTCALT0: usize = 0x00000001; // RTC Alert0 Masked Interrupt Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCT register.
//
// *****************************************************************************
pub const HIB_RTCT_TRIM_M: usize = 0x0000FFFF; // RTC Trim Value
pub const HIB_RTCT_TRIM_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_RTCSS register.
//
// *****************************************************************************
pub const HIB_RTCSS_RTCSSM_M: usize = 0x7FFF0000; // RTC Sub Seconds Match
pub const HIB_RTCSS_RTCSSC_M: usize = 0x00007FFF; // RTC Sub Seconds Count
pub const HIB_RTCSS_RTCSSM_S: usize = 16;
pub const HIB_RTCSS_RTCSSC_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the HIB_DATA register.
//
// *****************************************************************************
pub const HIB_DATA_RTD_M: usize = 0xFFFFFFFF; // Hibernation Module NV Data
pub const HIB_DATA_RTD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMA register.
//
// *****************************************************************************
pub const FLASH_FMA_OFFSET_M: usize = 0x0003FFFF; // Address Offset
pub const FLASH_FMA_OFFSET_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMD register.
//
// *****************************************************************************
pub const FLASH_FMD_DATA_M: usize = 0xFFFFFFFF; // Data Value
pub const FLASH_FMD_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMC register.
//
// *****************************************************************************
pub const FLASH_FMC_WRKEY: usize = 0xA4420000; // FLASH write key
pub const FLASH_FMC_COMT: usize = 0x00000008; // Commit Register Value
pub const FLASH_FMC_MERASE: usize = 0x00000004; // Mass Erase Flash Memory
pub const FLASH_FMC_ERASE: usize = 0x00000002; // Erase a Page of Flash Memory
pub const FLASH_FMC_WRITE: usize = 0x00000001; // Write a Word into Flash Memory

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCRIS register.
//
// *****************************************************************************
pub const FLASH_FCRIS_PROGRIS: usize = 0x00002000; // PROGVER Raw Interrupt Status
pub const FLASH_FCRIS_ERRIS: usize = 0x00000800; // ERVER Raw Interrupt Status
pub const FLASH_FCRIS_INVDRIS: usize = 0x00000400; // Invalid Data Raw Interrupt Status
pub const FLASH_FCRIS_VOLTRIS: usize = 0x00000200; // VOLTSTAT Raw Interrupt Status
pub const FLASH_FCRIS_ERIS: usize = 0x00000004; // EEPROM Raw Interrupt Status
pub const FLASH_FCRIS_PRIS: usize = 0x00000002; // Programming Raw Interrupt Status
pub const FLASH_FCRIS_ARIS: usize = 0x00000001; // Access Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCIM register.
//
// *****************************************************************************
pub const FLASH_FCIM_PROGMASK: usize = 0x00002000; // PROGVER Interrupt Mask
pub const FLASH_FCIM_ERMASK: usize = 0x00000800; // ERVER Interrupt Mask
pub const FLASH_FCIM_INVDMASK: usize = 0x00000400; // Invalid Data Interrupt Mask
pub const FLASH_FCIM_VOLTMASK: usize = 0x00000200; // VOLT Interrupt Mask
pub const FLASH_FCIM_EMASK: usize = 0x00000004; // EEPROM Interrupt Mask
pub const FLASH_FCIM_PMASK: usize = 0x00000002; // Programming Interrupt Mask
pub const FLASH_FCIM_AMASK: usize = 0x00000001; // Access Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FCMISC register.
//
// *****************************************************************************
pub const FLASH_FCMISC_PROGMISC: usize = 0x00002000; // PROGVER Masked Interrupt Status and Clear
pub const FLASH_FCMISC_ERMISC: usize = 0x00000800; // ERVER Masked Interrupt Status and Clear
pub const FLASH_FCMISC_INVDMISC: usize = 0x00000400; // Invalid Data Masked Interrupt Status and Clear
pub const FLASH_FCMISC_VOLTMISC: usize = 0x00000200; // VOLT Masked Interrupt Status and Clear
pub const FLASH_FCMISC_EMISC: usize = 0x00000004; // EEPROM Masked Interrupt Status and Clear
pub const FLASH_FCMISC_PMISC: usize = 0x00000002; // Programming Masked Interrupt Status and Clear
pub const FLASH_FCMISC_AMISC: usize = 0x00000001; // Access Masked Interrupt Status and Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FMC2 register.
//
// *****************************************************************************
pub const FLASH_FMC2_WRKEY: usize = 0xA4420000; // FLASH write key
pub const FLASH_FMC2_WRBUF: usize = 0x00000001; // Buffered Flash Memory Write

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FWBVAL register.
//
// *****************************************************************************
pub const FLASH_FWBVAL_FWB_M: usize = 0xFFFFFFFF; // Flash Memory Write Buffer

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FWBN register.
//
// *****************************************************************************
pub const FLASH_FWBN_DATA_M: usize = 0xFFFFFFFF; // Data

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_FSIZE register.
//
// *****************************************************************************
pub const FLASH_FSIZE_SIZE_M: usize = 0x0000FFFF; // Flash Size
pub const FLASH_FSIZE_SIZE_8KB: usize = 0x00000003; // 8 KB of Flash
pub const FLASH_FSIZE_SIZE_16KB: usize = 0x00000007; // 16 KB of Flash
pub const FLASH_FSIZE_SIZE_32KB: usize = 0x0000000F; // 32 KB of Flash
pub const FLASH_FSIZE_SIZE_64KB: usize = 0x0000001F; // 64 KB of Flash
pub const FLASH_FSIZE_SIZE_96KB: usize = 0x0000002F; // 96 KB of Flash
pub const FLASH_FSIZE_SIZE_128KB: usize = 0x0000003F; // 128 KB of Flash
pub const FLASH_FSIZE_SIZE_192KB: usize = 0x0000005F; // 192 KB of Flash
pub const FLASH_FSIZE_SIZE_256KB: usize = 0x0000007F; // 256 KB of Flash

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_SSIZE register.
//
// *****************************************************************************
pub const FLASH_SSIZE_SIZE_M: usize = 0x0000FFFF; // SRAM Size
pub const FLASH_SSIZE_SIZE_2KB: usize = 0x00000007; // 2 KB of SRAM
pub const FLASH_SSIZE_SIZE_4KB: usize = 0x0000000F; // 4 KB of SRAM
pub const FLASH_SSIZE_SIZE_6KB: usize = 0x00000017; // 6 KB of SRAM
pub const FLASH_SSIZE_SIZE_8KB: usize = 0x0000001F; // 8 KB of SRAM
pub const FLASH_SSIZE_SIZE_12KB: usize = 0x0000002F; // 12 KB of SRAM
pub const FLASH_SSIZE_SIZE_16KB: usize = 0x0000003F; // 16 KB of SRAM
pub const FLASH_SSIZE_SIZE_20KB: usize = 0x0000004F; // 20 KB of SRAM
pub const FLASH_SSIZE_SIZE_24KB: usize = 0x0000005F; // 24 KB of SRAM
pub const FLASH_SSIZE_SIZE_32KB: usize = 0x0000007F; // 32 KB of SRAM

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_ROMSWMAP register.
//
// *****************************************************************************
pub const FLASH_ROMSWMAP_SAFERTOS: usize = 0x00000001; // SafeRTOS Present

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_RMCTL register.
//
// *****************************************************************************
pub const FLASH_RMCTL_BA: usize = 0x00000001; // Boot Alias

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_BOOTCFG register.
//
// *****************************************************************************
pub const FLASH_BOOTCFG_NW: usize = 0x80000000; // Not Written
pub const FLASH_BOOTCFG_PORT_M: usize = 0x0000E000; // Boot GPIO Port
pub const FLASH_BOOTCFG_PORT_A: usize = 0x00000000; // Port A
pub const FLASH_BOOTCFG_PORT_B: usize = 0x00002000; // Port B
pub const FLASH_BOOTCFG_PORT_C: usize = 0x00004000; // Port C
pub const FLASH_BOOTCFG_PORT_D: usize = 0x00006000; // Port D
pub const FLASH_BOOTCFG_PORT_E: usize = 0x00008000; // Port E
pub const FLASH_BOOTCFG_PORT_F: usize = 0x0000A000; // Port F
pub const FLASH_BOOTCFG_PORT_G: usize = 0x0000C000; // Port G
pub const FLASH_BOOTCFG_PORT_H: usize = 0x0000E000; // Port H
pub const FLASH_BOOTCFG_PIN_M: usize = 0x00001C00; // Boot GPIO Pin
pub const FLASH_BOOTCFG_PIN_0: usize = 0x00000000; // Pin 0
pub const FLASH_BOOTCFG_PIN_1: usize = 0x00000400; // Pin 1
pub const FLASH_BOOTCFG_PIN_2: usize = 0x00000800; // Pin 2
pub const FLASH_BOOTCFG_PIN_3: usize = 0x00000C00; // Pin 3
pub const FLASH_BOOTCFG_PIN_4: usize = 0x00001000; // Pin 4
pub const FLASH_BOOTCFG_PIN_5: usize = 0x00001400; // Pin 5
pub const FLASH_BOOTCFG_PIN_6: usize = 0x00001800; // Pin 6
pub const FLASH_BOOTCFG_PIN_7: usize = 0x00001C00; // Pin 7
pub const FLASH_BOOTCFG_POL: usize = 0x00000200; // Boot GPIO Polarity
pub const FLASH_BOOTCFG_EN: usize = 0x00000100; // Boot GPIO Enable
pub const FLASH_BOOTCFG_DBG1: usize = 0x00000002; // Debug Control 1
pub const FLASH_BOOTCFG_DBG0: usize = 0x00000001; // Debug Control 0

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG0 register.
//
// *****************************************************************************
pub const FLASH_USERREG0_DATA_M: usize = 0xFFFFFFFF; // User Data
pub const FLASH_USERREG0_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG1 register.
//
// *****************************************************************************
pub const FLASH_USERREG1_DATA_M: usize = 0xFFFFFFFF; // User Data
pub const FLASH_USERREG1_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG2 register.
//
// *****************************************************************************
pub const FLASH_USERREG2_DATA_M: usize = 0xFFFFFFFF; // User Data
pub const FLASH_USERREG2_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the FLASH_USERREG3 register.
//
// *****************************************************************************
pub const FLASH_USERREG3_DATA_M: usize = 0xFFFFFFFF; // User Data
pub const FLASH_USERREG3_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID0 register.
//
// *****************************************************************************
pub const SYSCTL_DID0_VER_M: usize = 0x70000000; // DID0 Version
pub const SYSCTL_DID0_VER_1: usize = 0x10000000; // Second version of the DID0 register format
pub const SYSCTL_DID0_CLASS_M: usize = 0x00FF0000; // Device Class
pub const SYSCTL_DID0_CLASS_BLIZZARD: usize = 0x00050000; // Stellaris(R) Blizzard-class microcontrollers
pub const SYSCTL_DID0_MAJ_M: usize = 0x0000FF00; // Major Revision
pub const SYSCTL_DID0_MAJ_REVA: usize = 0x00000000; // Revision A (initial device)
pub const SYSCTL_DID0_MAJ_REVB: usize = 0x00000100; // Revision B (first base layer revision)
pub const SYSCTL_DID0_MAJ_REVC: usize = 0x00000200; // Revision C (second base layer revision)
pub const SYSCTL_DID0_MIN_M: usize = 0x000000FF; // Minor Revision
pub const SYSCTL_DID0_MIN_0: usize = 0x00000000; // Initial device, or a major revision update
pub const SYSCTL_DID0_MIN_1: usize = 0x00000001; // First metal layer change
pub const SYSCTL_DID0_MIN_2: usize = 0x00000002; // Second metal layer change

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID1 register.
//
// *****************************************************************************
pub const SYSCTL_DID1_VER_M: usize = 0xF0000000; // DID1 Version
pub const SYSCTL_DID1_VER_0: usize = 0x00000000; // Initial DID1 register format definition, indicating a Stellaris LM3Snnn device
pub const SYSCTL_DID1_VER_1: usize = 0x10000000; // Second version of the DID1 register format
pub const SYSCTL_DID1_FAM_M: usize = 0x0F000000; // Family
pub const SYSCTL_DID1_FAM_STELLARIS: usize = 0x00000000; // Stellaris family of microcontollers, that is, all devices with external part numbers starting with LM3S
pub const SYSCTL_DID1_PRTNO_M: usize = 0x00FF0000; // Part Number
pub const SYSCTL_DID1_PRTNO_LM4F120H5QR: usize = 0x00040000; // LM4F120H5QR
pub const SYSCTL_DID1_PINCNT_M: usize = 0x0000E000; // Package Pin Count
pub const SYSCTL_DID1_PINCNT_28: usize = 0x00000000; // 28-pin package
pub const SYSCTL_DID1_PINCNT_48: usize = 0x00002000; // 48-pin package
pub const SYSCTL_DID1_PINCNT_100: usize = 0x00004000; // 100-pin package
pub const SYSCTL_DID1_PINCNT_64: usize = 0x00006000; // 64-pin package
pub const SYSCTL_DID1_PINCNT_144: usize = 0x00008000; // 144-pin package
pub const SYSCTL_DID1_PINCNT_157: usize = 0x0000A000; // 157-pin package
pub const SYSCTL_DID1_TEMP_M: usize = 0x000000E0; // Temperature Range
pub const SYSCTL_DID1_TEMP_C: usize = 0x00000000; // Commercial temperature range (0C to 70C)
pub const SYSCTL_DID1_TEMP_I: usize = 0x00000020; // Industrial temperature range (-40C to 85C)
pub const SYSCTL_DID1_TEMP_E: usize = 0x00000040; // Extended temperature range (-40C to 105C)
pub const SYSCTL_DID1_PKG_M: usize = 0x00000018; // Package Type
pub const SYSCTL_DID1_PKG_SOIC: usize = 0x00000000; // SOIC package
pub const SYSCTL_DID1_PKG_QFP: usize = 0x00000008; // LQFP package
pub const SYSCTL_DID1_PKG_BGA: usize = 0x00000010; // BGA package
pub const SYSCTL_DID1_ROHS: usize = 0x00000004; // RoHS-Compliance
pub const SYSCTL_DID1_QUAL_M: usize = 0x00000003; // Qualification Status
pub const SYSCTL_DID1_QUAL_ES: usize = 0x00000000; // Engineering Sample (unqualified)
pub const SYSCTL_DID1_QUAL_PP: usize = 0x00000001; // Pilot Production (unqualified)
pub const SYSCTL_DID1_QUAL_FQ: usize = 0x00000002; // Fully Qualified

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC0 register.
//
// *****************************************************************************
pub const SYSCTL_DC0_SRAMSZ_M: usize = 0xFFFF0000; // SRAM Size
pub const SYSCTL_DC0_SRAMSZ_2KB: usize = 0x00070000; // 2 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_4KB: usize = 0x000F0000; // 4 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_6KB: usize = 0x00170000; // 6 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_8KB: usize = 0x001F0000; // 8 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_12KB: usize = 0x002F0000; // 12 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_16KB: usize = 0x003F0000; // 16 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_20KB: usize = 0x004F0000; // 20 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_24KB: usize = 0x005F0000; // 24 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_32KB: usize = 0x007F0000; // 32 KB of SRAM
pub const SYSCTL_DC0_FLASHSZ_M: usize = 0x0000FFFF; // Flash Size
pub const SYSCTL_DC0_FLASHSZ_8KB: usize = 0x00000003; // 8 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_16KB: usize = 0x00000007; // 16 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_32KB: usize = 0x0000000F; // 32 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_64KB: usize = 0x0000001F; // 64 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_96KB: usize = 0x0000002F; // 96 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_128K: usize = 0x0000003F; // 128 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_192K: usize = 0x0000005F; // 192 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_256K: usize = 0x0000007F; // 256 KB of Flash
pub const SYSCTL_DC0_SRAMSZ_S: usize = 16; // SRAM size shift
pub const SYSCTL_DC0_FLASHSZ_S: usize = 0; // Flash size shift

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC1 register.
//
// *****************************************************************************
pub const SYSCTL_DC1_WDT1: usize = 0x10000000; // Watchdog Timer1 Present
pub const SYSCTL_DC1_CAN1: usize = 0x02000000; // CAN Module 1 Present
pub const SYSCTL_DC1_CAN0: usize = 0x01000000; // CAN Module 0 Present
pub const SYSCTL_DC1_PWM1: usize = 0x00200000; // PWM Module 1 Present
pub const SYSCTL_DC1_PWM0: usize = 0x00100000; // PWM Module 0 Present
pub const SYSCTL_DC1_ADC1: usize = 0x00020000; // ADC Module 1 Present
pub const SYSCTL_DC1_ADC0: usize = 0x00010000; // ADC Module 0 Present
pub const SYSCTL_DC1_MINSYSDIV_M: usize = 0x0000F000; // System Clock Divider
pub const SYSCTL_DC1_MINSYSDIV_100: usize = 0x00001000; // Divide VCO (400MHZ) by 5 minimum
pub const SYSCTL_DC1_MINSYSDIV_66: usize = 0x00002000; // Divide VCO (400MHZ) by 2*2 + 2 = 6 minimum
pub const SYSCTL_DC1_MINSYSDIV_50: usize = 0x00003000; // Specifies a 50-MHz CPU clock with a PLL divider of 4
pub const SYSCTL_DC1_MINSYSDIV_40: usize = 0x00004000; // Specifies a 40-MHz CPU clock with a PLL divider of 5
pub const SYSCTL_DC1_MINSYSDIV_25: usize = 0x00007000; // Specifies a 25-MHz clock with a PLL divider of 8
pub const SYSCTL_DC1_MINSYSDIV_20: usize = 0x00009000; // Specifies a 20-MHz clock with a PLL divider of 10
pub const SYSCTL_DC1_ADC1SPD_M: usize = 0x00000C00; // Max ADC1 Speed
pub const SYSCTL_DC1_ADC1SPD_125K: usize = 0x00000000; // 125K samples/second
pub const SYSCTL_DC1_ADC1SPD_250K: usize = 0x00000400; // 250K samples/second
pub const SYSCTL_DC1_ADC1SPD_500K: usize = 0x00000800; // 500K samples/second
pub const SYSCTL_DC1_ADC1SPD_1M: usize = 0x00000C00; // 1M samples/second
pub const SYSCTL_DC1_ADC0SPD_M: usize = 0x00000300; // Max ADC0 Speed
pub const SYSCTL_DC1_ADC0SPD_125K: usize = 0x00000000; // 125K samples/second
pub const SYSCTL_DC1_ADC0SPD_250K: usize = 0x00000100; // 250K samples/second
pub const SYSCTL_DC1_ADC0SPD_500K: usize = 0x00000200; // 500K samples/second
pub const SYSCTL_DC1_ADC0SPD_1M: usize = 0x00000300; // 1M samples/second
pub const SYSCTL_DC1_MPU: usize = 0x00000080; // MPU Present
pub const SYSCTL_DC1_HIB: usize = 0x00000040; // Hibernation Module Present
pub const SYSCTL_DC1_TEMP: usize = 0x00000020; // Temp Sensor Present
pub const SYSCTL_DC1_PLL: usize = 0x00000010; // PLL Present
pub const SYSCTL_DC1_WDT0: usize = 0x00000008; // Watchdog Timer 0 Present
pub const SYSCTL_DC1_SWO: usize = 0x00000004; // SWO Trace Port Present
pub const SYSCTL_DC1_SWD: usize = 0x00000002; // SWD Present
pub const SYSCTL_DC1_JTAG: usize = 0x00000001; // JTAG Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC2 register.
//
// *****************************************************************************
pub const SYSCTL_DC2_EPI0: usize = 0x40000000; // EPI Module 0 Present
pub const SYSCTL_DC2_I2S0: usize = 0x10000000; // I2S Module 0 Present
pub const SYSCTL_DC2_COMP2: usize = 0x04000000; // Analog Comparator 2 Present
pub const SYSCTL_DC2_COMP1: usize = 0x02000000; // Analog Comparator 1 Present
pub const SYSCTL_DC2_COMP0: usize = 0x01000000; // Analog Comparator 0 Present
pub const SYSCTL_DC2_TIMER3: usize = 0x00080000; // Timer Module 3 Present
pub const SYSCTL_DC2_TIMER2: usize = 0x00040000; // Timer Module 2 Present
pub const SYSCTL_DC2_TIMER1: usize = 0x00020000; // Timer Module 1 Present
pub const SYSCTL_DC2_TIMER0: usize = 0x00010000; // Timer Module 0 Present
pub const SYSCTL_DC2_I2C1HS: usize = 0x00008000; // I2C Module 1 Speed
pub const SYSCTL_DC2_I2C1: usize = 0x00004000; // I2C Module 1 Present
pub const SYSCTL_DC2_I2C0HS: usize = 0x00002000; // I2C Module 0 Speed
pub const SYSCTL_DC2_I2C0: usize = 0x00001000; // I2C Module 0 Present
pub const SYSCTL_DC2_QEI1: usize = 0x00000200; // QEI Module 1 Present
pub const SYSCTL_DC2_QEI0: usize = 0x00000100; // QEI Module 0 Present
pub const SYSCTL_DC2_SSI1: usize = 0x00000020; // SSI Module 1 Present
pub const SYSCTL_DC2_SSI0: usize = 0x00000010; // SSI Module 0 Present
pub const SYSCTL_DC2_UART2: usize = 0x00000004; // UART Module 2 Present
pub const SYSCTL_DC2_UART1: usize = 0x00000002; // UART Module 1 Present
pub const SYSCTL_DC2_UART0: usize = 0x00000001; // UART Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC3 register.
//
// *****************************************************************************
pub const SYSCTL_DC3_32KHZ: usize = 0x80000000; // 32KHz Input Clock Available
pub const SYSCTL_DC3_CCP5: usize = 0x20000000; // CCP5 Pin Present
pub const SYSCTL_DC3_CCP4: usize = 0x10000000; // CCP4 Pin Present
pub const SYSCTL_DC3_CCP3: usize = 0x08000000; // CCP3 Pin Present
pub const SYSCTL_DC3_CCP2: usize = 0x04000000; // CCP2 Pin Present
pub const SYSCTL_DC3_CCP1: usize = 0x02000000; // CCP1 Pin Present
pub const SYSCTL_DC3_CCP0: usize = 0x01000000; // CCP0 Pin Present
pub const SYSCTL_DC3_ADC0AIN7: usize = 0x00800000; // ADC Module 0 AIN7 Pin Present
pub const SYSCTL_DC3_ADC0AIN6: usize = 0x00400000; // ADC Module 0 AIN6 Pin Present
pub const SYSCTL_DC3_ADC0AIN5: usize = 0x00200000; // ADC Module 0 AIN5 Pin Present
pub const SYSCTL_DC3_ADC0AIN4: usize = 0x00100000; // ADC Module 0 AIN4 Pin Present
pub const SYSCTL_DC3_ADC0AIN3: usize = 0x00080000; // ADC Module 0 AIN3 Pin Present
pub const SYSCTL_DC3_ADC0AIN2: usize = 0x00040000; // ADC Module 0 AIN2 Pin Present
pub const SYSCTL_DC3_ADC0AIN1: usize = 0x00020000; // ADC Module 0 AIN1 Pin Present
pub const SYSCTL_DC3_ADC0AIN0: usize = 0x00010000; // ADC Module 0 AIN0 Pin Present
pub const SYSCTL_DC3_PWMFAULT: usize = 0x00008000; // PWM Fault Pin Present
pub const SYSCTL_DC3_C2O: usize = 0x00004000; // C2o Pin Present
pub const SYSCTL_DC3_C2PLUS: usize = 0x00002000; // C2+ Pin Present
pub const SYSCTL_DC3_C2MINUS: usize = 0x00001000; // C2- Pin Present
pub const SYSCTL_DC3_C1O: usize = 0x00000800; // C1o Pin Present
pub const SYSCTL_DC3_C1PLUS: usize = 0x00000400; // C1+ Pin Present
pub const SYSCTL_DC3_C1MINUS: usize = 0x00000200; // C1- Pin Present
pub const SYSCTL_DC3_C0O: usize = 0x00000100; // C0o Pin Present
pub const SYSCTL_DC3_C0PLUS: usize = 0x00000080; // C0+ Pin Present
pub const SYSCTL_DC3_C0MINUS: usize = 0x00000040; // C0- Pin Present
pub const SYSCTL_DC3_PWM5: usize = 0x00000020; // PWM5 Pin Present
pub const SYSCTL_DC3_PWM4: usize = 0x00000010; // PWM4 Pin Present
pub const SYSCTL_DC3_PWM3: usize = 0x00000008; // PWM3 Pin Present
pub const SYSCTL_DC3_PWM2: usize = 0x00000004; // PWM2 Pin Present
pub const SYSCTL_DC3_PWM1: usize = 0x00000002; // PWM1 Pin Present
pub const SYSCTL_DC3_PWM0: usize = 0x00000001; // PWM0 Pin Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC4 register.
//
// *****************************************************************************
pub const SYSCTL_DC4_EPHY0: usize = 0x40000000; // Ethernet PHY Layer 0 Present
pub const SYSCTL_DC4_EMAC0: usize = 0x10000000; // Ethernet MAC Layer 0 Present
pub const SYSCTL_DC4_E1588: usize = 0x01000000; // 1588 Capable
pub const SYSCTL_DC4_PICAL: usize = 0x00040000; // PIOSC Calibrate
pub const SYSCTL_DC4_CCP7: usize = 0x00008000; // CCP7 Pin Present
pub const SYSCTL_DC4_CCP6: usize = 0x00004000; // CCP6 Pin Present
pub const SYSCTL_DC4_UDMA: usize = 0x00002000; // Micro-DMA Module Present
pub const SYSCTL_DC4_ROM: usize = 0x00001000; // Internal Code ROM Present
pub const SYSCTL_DC4_GPIOJ: usize = 0x00000100; // GPIO Port J Present
pub const SYSCTL_DC4_GPIOH: usize = 0x00000080; // GPIO Port H Present
pub const SYSCTL_DC4_GPIOG: usize = 0x00000040; // GPIO Port G Present
pub const SYSCTL_DC4_GPIOF: usize = 0x00000020; // GPIO Port F Present
pub const SYSCTL_DC4_GPIOE: usize = 0x00000010; // GPIO Port E Present
pub const SYSCTL_DC4_GPIOD: usize = 0x00000008; // GPIO Port D Present
pub const SYSCTL_DC4_GPIOC: usize = 0x00000004; // GPIO Port C Present
pub const SYSCTL_DC4_GPIOB: usize = 0x00000002; // GPIO Port B Present
pub const SYSCTL_DC4_GPIOA: usize = 0x00000001; // GPIO Port A Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC5 register.
//
// *****************************************************************************
pub const SYSCTL_DC5_PWMFAULT3: usize = 0x08000000; // PWM Fault 3 Pin Present
pub const SYSCTL_DC5_PWMFAULT2: usize = 0x04000000; // PWM Fault 2 Pin Present
pub const SYSCTL_DC5_PWMFAULT1: usize = 0x02000000; // PWM Fault 1 Pin Present
pub const SYSCTL_DC5_PWMFAULT0: usize = 0x01000000; // PWM Fault 0 Pin Present
pub const SYSCTL_DC5_PWMEFLT: usize = 0x00200000; // PWM Extended Fault Active
pub const SYSCTL_DC5_PWMESYNC: usize = 0x00100000; // PWM Extended SYNC Active
pub const SYSCTL_DC5_PWM7: usize = 0x00000080; // PWM7 Pin Present
pub const SYSCTL_DC5_PWM6: usize = 0x00000040; // PWM6 Pin Present
pub const SYSCTL_DC5_PWM5: usize = 0x00000020; // PWM5 Pin Present
pub const SYSCTL_DC5_PWM4: usize = 0x00000010; // PWM4 Pin Present
pub const SYSCTL_DC5_PWM3: usize = 0x00000008; // PWM3 Pin Present
pub const SYSCTL_DC5_PWM2: usize = 0x00000004; // PWM2 Pin Present
pub const SYSCTL_DC5_PWM1: usize = 0x00000002; // PWM1 Pin Present
pub const SYSCTL_DC5_PWM0: usize = 0x00000001; // PWM0 Pin Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC6 register.
//
// *****************************************************************************
pub const SYSCTL_DC6_USB0PHY: usize = 0x00000010; // USB Module 0 PHY Present
pub const SYSCTL_DC6_USB0_M: usize = 0x00000003; // USB Module 0 Present
pub const SYSCTL_DC6_USB0_DEV: usize = 0x00000001; // USB0 is Device Only
pub const SYSCTL_DC6_USB0_HOSTDEV: usize = 0x00000002; // USB is Device or Host
pub const SYSCTL_DC6_USB0_OTG: usize = 0x00000003; // USB0 is OTG

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC7 register.
//
// *****************************************************************************
pub const SYSCTL_DC7_DMACH30: usize = 0x40000000; // SW
pub const SYSCTL_DC7_DMACH29: usize = 0x20000000; // I2S0_TX / CAN1_TX
pub const SYSCTL_DC7_DMACH28: usize = 0x10000000; // I2S0_RX / CAN1_RX
pub const SYSCTL_DC7_DMACH27: usize = 0x08000000; // CAN1_TX / ADC1_SS3
pub const SYSCTL_DC7_DMACH26: usize = 0x04000000; // CAN1_RX / ADC1_SS2
pub const SYSCTL_DC7_DMACH25: usize = 0x02000000; // SSI1_TX / ADC1_SS1
pub const SYSCTL_DC7_DMACH24: usize = 0x01000000; // SSI1_RX / ADC1_SS0
pub const SYSCTL_DC7_DMACH23: usize = 0x00800000; // UART1_TX / CAN2_TX
pub const SYSCTL_DC7_DMACH22: usize = 0x00400000; // UART1_RX / CAN2_RX
pub const SYSCTL_DC7_DMACH21: usize = 0x00200000; // Timer1B / EPI0_WFIFO
pub const SYSCTL_DC7_DMACH20: usize = 0x00100000; // Timer1A / EPI0_NBRFIFO
pub const SYSCTL_DC7_DMACH19: usize = 0x00080000; // Timer0B / Timer1B
pub const SYSCTL_DC7_DMACH18: usize = 0x00040000; // Timer0A / Timer1A
pub const SYSCTL_DC7_DMACH17: usize = 0x00020000; // ADC0_SS3
pub const SYSCTL_DC7_DMACH16: usize = 0x00010000; // ADC0_SS2
pub const SYSCTL_DC7_DMACH15: usize = 0x00008000; // ADC0_SS1 / Timer2B
pub const SYSCTL_DC7_DMACH14: usize = 0x00004000; // ADC0_SS0 / Timer2A
pub const SYSCTL_DC7_DMACH13: usize = 0x00002000; // CAN0_TX / UART2_TX
pub const SYSCTL_DC7_DMACH12: usize = 0x00001000; // CAN0_RX / UART2_RX
pub const SYSCTL_DC7_DMACH11: usize = 0x00000800; // SSI0_TX / SSI1_TX
pub const SYSCTL_DC7_DMACH10: usize = 0x00000400; // SSI0_RX / SSI1_RX
pub const SYSCTL_DC7_DMACH9: usize = 0x00000200; // UART0_TX / UART1_TX
pub const SYSCTL_DC7_DMACH8: usize = 0x00000100; // UART0_RX / UART1_RX
pub const SYSCTL_DC7_DMACH7: usize = 0x00000080; // ETH_TX / Timer2B
pub const SYSCTL_DC7_DMACH6: usize = 0x00000040; // ETH_RX / Timer2A
pub const SYSCTL_DC7_DMACH5: usize = 0x00000020; // USB_EP3_TX / Timer2B
pub const SYSCTL_DC7_DMACH4: usize = 0x00000010; // USB_EP3_RX / Timer2A
pub const SYSCTL_DC7_DMACH3: usize = 0x00000008; // USB_EP2_TX / Timer3B
pub const SYSCTL_DC7_DMACH2: usize = 0x00000004; // USB_EP2_RX / Timer3A
pub const SYSCTL_DC7_DMACH1: usize = 0x00000002; // USB_EP1_TX / UART2_TX
pub const SYSCTL_DC7_DMACH0: usize = 0x00000001; // USB_EP1_RX / UART2_RX

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC8 register.
//
// *****************************************************************************
pub const SYSCTL_DC8_ADC1AIN15: usize = 0x80000000; // ADC Module 1 AIN15 Pin Present
pub const SYSCTL_DC8_ADC1AIN14: usize = 0x40000000; // ADC Module 1 AIN14 Pin Present
pub const SYSCTL_DC8_ADC1AIN13: usize = 0x20000000; // ADC Module 1 AIN13 Pin Present
pub const SYSCTL_DC8_ADC1AIN12: usize = 0x10000000; // ADC Module 1 AIN12 Pin Present
pub const SYSCTL_DC8_ADC1AIN11: usize = 0x08000000; // ADC Module 1 AIN11 Pin Present
pub const SYSCTL_DC8_ADC1AIN10: usize = 0x04000000; // ADC Module 1 AIN10 Pin Present
pub const SYSCTL_DC8_ADC1AIN9: usize = 0x02000000; // ADC Module 1 AIN9 Pin Present
pub const SYSCTL_DC8_ADC1AIN8: usize = 0x01000000; // ADC Module 1 AIN8 Pin Present
pub const SYSCTL_DC8_ADC1AIN7: usize = 0x00800000; // ADC Module 1 AIN7 Pin Present
pub const SYSCTL_DC8_ADC1AIN6: usize = 0x00400000; // ADC Module 1 AIN6 Pin Present
pub const SYSCTL_DC8_ADC1AIN5: usize = 0x00200000; // ADC Module 1 AIN5 Pin Present
pub const SYSCTL_DC8_ADC1AIN4: usize = 0x00100000; // ADC Module 1 AIN4 Pin Present
pub const SYSCTL_DC8_ADC1AIN3: usize = 0x00080000; // ADC Module 1 AIN3 Pin Present
pub const SYSCTL_DC8_ADC1AIN2: usize = 0x00040000; // ADC Module 1 AIN2 Pin Present
pub const SYSCTL_DC8_ADC1AIN1: usize = 0x00020000; // ADC Module 1 AIN1 Pin Present
pub const SYSCTL_DC8_ADC1AIN0: usize = 0x00010000; // ADC Module 1 AIN0 Pin Present
pub const SYSCTL_DC8_ADC0AIN15: usize = 0x00008000; // ADC Module 0 AIN15 Pin Present
pub const SYSCTL_DC8_ADC0AIN14: usize = 0x00004000; // ADC Module 0 AIN14 Pin Present
pub const SYSCTL_DC8_ADC0AIN13: usize = 0x00002000; // ADC Module 0 AIN13 Pin Present
pub const SYSCTL_DC8_ADC0AIN12: usize = 0x00001000; // ADC Module 0 AIN12 Pin Present
pub const SYSCTL_DC8_ADC0AIN11: usize = 0x00000800; // ADC Module 0 AIN11 Pin Present
pub const SYSCTL_DC8_ADC0AIN10: usize = 0x00000400; // ADC Module 0 AIN10 Pin Present
pub const SYSCTL_DC8_ADC0AIN9: usize = 0x00000200; // ADC Module 0 AIN9 Pin Present
pub const SYSCTL_DC8_ADC0AIN8: usize = 0x00000100; // ADC Module 0 AIN8 Pin Present
pub const SYSCTL_DC8_ADC0AIN7: usize = 0x00000080; // ADC Module 0 AIN7 Pin Present
pub const SYSCTL_DC8_ADC0AIN6: usize = 0x00000040; // ADC Module 0 AIN6 Pin Present
pub const SYSCTL_DC8_ADC0AIN5: usize = 0x00000020; // ADC Module 0 AIN5 Pin Present
pub const SYSCTL_DC8_ADC0AIN4: usize = 0x00000010; // ADC Module 0 AIN4 Pin Present
pub const SYSCTL_DC8_ADC0AIN3: usize = 0x00000008; // ADC Module 0 AIN3 Pin Present
pub const SYSCTL_DC8_ADC0AIN2: usize = 0x00000004; // ADC Module 0 AIN2 Pin Present
pub const SYSCTL_DC8_ADC0AIN1: usize = 0x00000002; // ADC Module 0 AIN1 Pin Present
pub const SYSCTL_DC8_ADC0AIN0: usize = 0x00000001; // ADC Module 0 AIN0 Pin Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PBORCTL register.
//
// *****************************************************************************
pub const SYSCTL_PBORCTL_BORIOR: usize = 0x00000002; // BOR Interrupt or Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR0 register.
//
// *****************************************************************************
pub const SYSCTL_SRCR0_WDT1: usize = 0x10000000; // WDT1 Reset Control
pub const SYSCTL_SRCR0_CAN1: usize = 0x02000000; // CAN1 Reset Control
pub const SYSCTL_SRCR0_CAN0: usize = 0x01000000; // CAN0 Reset Control
pub const SYSCTL_SRCR0_PWM0: usize = 0x00100000; // PWM Reset Control
pub const SYSCTL_SRCR0_ADC1: usize = 0x00020000; // ADC1 Reset Control
pub const SYSCTL_SRCR0_ADC0: usize = 0x00010000; // ADC0 Reset Control
pub const SYSCTL_SRCR0_HIB: usize = 0x00000040; // HIB Reset Control
pub const SYSCTL_SRCR0_WDT0: usize = 0x00000008; // WDT0 Reset Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR1 register.
//
// *****************************************************************************
pub const SYSCTL_SRCR1_COMP2: usize = 0x04000000; // Analog Comp 2 Reset Control
pub const SYSCTL_SRCR1_COMP1: usize = 0x02000000; // Analog Comp 1 Reset Control
pub const SYSCTL_SRCR1_COMP0: usize = 0x01000000; // Analog Comp 0 Reset Control
pub const SYSCTL_SRCR1_TIMER3: usize = 0x00080000; // Timer 3 Reset Control
pub const SYSCTL_SRCR1_TIMER2: usize = 0x00040000; // Timer 2 Reset Control
pub const SYSCTL_SRCR1_TIMER1: usize = 0x00020000; // Timer 1 Reset Control
pub const SYSCTL_SRCR1_TIMER0: usize = 0x00010000; // Timer 0 Reset Control
pub const SYSCTL_SRCR1_I2C1: usize = 0x00004000; // I2C1 Reset Control
pub const SYSCTL_SRCR1_I2C0: usize = 0x00001000; // I2C0 Reset Control
pub const SYSCTL_SRCR1_QEI1: usize = 0x00000200; // QEI1 Reset Control
pub const SYSCTL_SRCR1_QEI0: usize = 0x00000100; // QEI0 Reset Control
pub const SYSCTL_SRCR1_SSI1: usize = 0x00000020; // SSI1 Reset Control
pub const SYSCTL_SRCR1_SSI0: usize = 0x00000010; // SSI0 Reset Control
pub const SYSCTL_SRCR1_UART2: usize = 0x00000004; // UART2 Reset Control
pub const SYSCTL_SRCR1_UART1: usize = 0x00000002; // UART1 Reset Control
pub const SYSCTL_SRCR1_UART0: usize = 0x00000001; // UART0 Reset Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR2 register.
//
// *****************************************************************************
pub const SYSCTL_SRCR2_USB0: usize = 0x00010000; // USB0 Reset Control
pub const SYSCTL_SRCR2_UDMA: usize = 0x00002000; // Micro-DMA Reset Control
pub const SYSCTL_SRCR2_GPIOJ: usize = 0x00000100; // Port J Reset Control
pub const SYSCTL_SRCR2_GPIOH: usize = 0x00000080; // Port H Reset Control
pub const SYSCTL_SRCR2_GPIOG: usize = 0x00000040; // Port G Reset Control
pub const SYSCTL_SRCR2_GPIOF: usize = 0x00000020; // Port F Reset Control
pub const SYSCTL_SRCR2_GPIOE: usize = 0x00000010; // Port E Reset Control
pub const SYSCTL_SRCR2_GPIOD: usize = 0x00000008; // Port D Reset Control
pub const SYSCTL_SRCR2_GPIOC: usize = 0x00000004; // Port C Reset Control
pub const SYSCTL_SRCR2_GPIOB: usize = 0x00000002; // Port B Reset Control
pub const SYSCTL_SRCR2_GPIOA: usize = 0x00000001; // Port A Reset Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RIS register.
//
// *****************************************************************************
pub const SYSCTL_RIS_MOSCPUPRIS: usize = 0x00000100; // MOSC Power Up Raw Interrupt Status
pub const SYSCTL_RIS_USBPLLLRIS: usize = 0x00000080; // USB PLL Lock Raw Interrupt Status
pub const SYSCTL_RIS_PLLLRIS: usize = 0x00000040; // PLL Lock Raw Interrupt Status
pub const SYSCTL_RIS_MOFRIS: usize = 0x00000008; // Main Oscillator Fault Raw Interrupt Status
pub const SYSCTL_RIS_BORRIS: usize = 0x00000002; // Brown-Out Reset Raw Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_IMC register.
//
// *****************************************************************************
pub const SYSCTL_IMC_MOSCPUPIM: usize = 0x00000100; // MOSC Power Up Interrupt Mask
pub const SYSCTL_IMC_USBPLLLIM: usize = 0x00000080; // USB PLL Lock Interrupt Mask
pub const SYSCTL_IMC_PLLLIM: usize = 0x00000040; // PLL Lock Interrupt Mask
pub const SYSCTL_IMC_MOFIM: usize = 0x00000008; // Main Oscillator Fault Interrupt Mask
pub const SYSCTL_IMC_BORIM: usize = 0x00000002; // Brown-Out Reset Interrupt Mask

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MISC register.
//
// *****************************************************************************
pub const SYSCTL_MISC_MOSCPUPMIS: usize = 0x00000100; // MOSC Power Up Masked Interrupt Status
pub const SYSCTL_MISC_USBPLLLMIS: usize = 0x00000080; // USB PLL Lock Masked Interrupt Status
pub const SYSCTL_MISC_PLLLMIS: usize = 0x00000040; // PLL Lock Masked Interrupt Status
pub const SYSCTL_MISC_MOFMIS: usize = 0x00000008; // Main Oscillator Fault Masked Interrupt Status
pub const SYSCTL_MISC_BORMIS: usize = 0x00000002; // BOR Masked Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESC register.
//
// *****************************************************************************
pub const SYSCTL_RESC_MOSCFAIL: usize = 0x00010000; // MOSC Failure Reset
pub const SYSCTL_RESC_WDT1: usize = 0x00000020; // Watchdog Timer 1 Reset
pub const SYSCTL_RESC_SW: usize = 0x00000010; // Software Reset
pub const SYSCTL_RESC_WDT0: usize = 0x00000008; // Watchdog Timer 0 Reset
pub const SYSCTL_RESC_BOR: usize = 0x00000004; // Brown-Out Reset
pub const SYSCTL_RESC_POR: usize = 0x00000002; // Power-On Reset
pub const SYSCTL_RESC_EXT: usize = 0x00000001; // External Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC register.
//
// *****************************************************************************
pub const SYSCTL_RCC_ACG: usize = 0x08000000; // Auto Clock Gating
pub const SYSCTL_RCC_SYSDIV_M: usize = 0x07800000; // System Clock Divisor
pub const SYSCTL_RCC_USESYSDIV: usize = 0x00400000; // Enable System Clock Divider
pub const SYSCTL_RCC_PWRDN: usize = 0x00002000; // PLL Power Down
pub const SYSCTL_RCC_BYPASS: usize = 0x00000800; // PLL Bypass
pub const SYSCTL_RCC_XTAL_M: usize = 0x000007C0; // Crystal Value
pub const SYSCTL_RCC_XTAL_4MHZ: usize = 0x00000180; // 4 MHz
pub const SYSCTL_RCC_XTAL_4_09MHZ: usize = 0x000001C0; // 4.096 MHz
pub const SYSCTL_RCC_XTAL_4_91MHZ: usize = 0x00000200; // 4.9152 MHz
pub const SYSCTL_RCC_XTAL_5MHZ: usize = 0x00000240; // 5 MHz
pub const SYSCTL_RCC_XTAL_5_12MHZ: usize = 0x00000280; // 5.12 MHz
pub const SYSCTL_RCC_XTAL_6MHZ: usize = 0x000002C0; // 6 MHz
pub const SYSCTL_RCC_XTAL_6_14MHZ: usize = 0x00000300; // 6.144 MHz
pub const SYSCTL_RCC_XTAL_7_37MHZ: usize = 0x00000340; // 7.3728 MHz
pub const SYSCTL_RCC_XTAL_8MHZ: usize = 0x00000380; // 8 MHz
pub const SYSCTL_RCC_XTAL_8_19MHZ: usize = 0x000003C0; // 8.192 MHz
pub const SYSCTL_RCC_XTAL_10MHZ: usize = 0x00000400; // 10 MHz
pub const SYSCTL_RCC_XTAL_12MHZ: usize = 0x00000440; // 12 MHz
pub const SYSCTL_RCC_XTAL_12_2MHZ: usize = 0x00000480; // 12.288 MHz
pub const SYSCTL_RCC_XTAL_13_5MHZ: usize = 0x000004C0; // 13.56 MHz
pub const SYSCTL_RCC_XTAL_14_3MHZ: usize = 0x00000500; // 14.31818 MHz
pub const SYSCTL_RCC_XTAL_16MHZ: usize = 0x00000540; // 16 MHz
pub const SYSCTL_RCC_XTAL_16_3MHZ: usize = 0x00000580; // 16.384 MHz
pub const SYSCTL_RCC_XTAL_18MHZ: usize = 0x000005C0; // 18.0 MHz
pub const SYSCTL_RCC_XTAL_20MHZ: usize = 0x00000600; // 20.0 MHz
pub const SYSCTL_RCC_XTAL_24MHZ: usize = 0x00000640; // 24.0 MHz
pub const SYSCTL_RCC_XTAL_25MHZ: usize = 0x00000680; // 25.0 MHz
pub const SYSCTL_RCC_OSCSRC_M: usize = 0x00000030; // Oscillator Source
pub const SYSCTL_RCC_OSCSRC_MAIN: usize = 0x00000000; // MOSC
pub const SYSCTL_RCC_OSCSRC_INT: usize = 0x00000010; // IOSC
pub const SYSCTL_RCC_OSCSRC_INT4: usize = 0x00000020; // IOSC/4
pub const SYSCTL_RCC_OSCSRC_30: usize = 0x00000030; // 30 kHz
pub const SYSCTL_RCC_IOSCDIS: usize = 0x00000002; // Internal Oscillator Disable
pub const SYSCTL_RCC_MOSCDIS: usize = 0x00000001; // Main Oscillator Disable
pub const SYSCTL_RCC_SYSDIV_S: usize = 23;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_GPIOHBCTL
// register.
//
// *****************************************************************************
pub const SYSCTL_GPIOHBCTL_PORTF: usize = 0x00000020; // Port F Advanced High-Performance Bus
pub const SYSCTL_GPIOHBCTL_PORTE: usize = 0x00000010; // Port E Advanced High-Performance Bus
pub const SYSCTL_GPIOHBCTL_PORTD: usize = 0x00000008; // Port D Advanced High-Performance Bus
pub const SYSCTL_GPIOHBCTL_PORTC: usize = 0x00000004; // Port C Advanced High-Performance Bus
pub const SYSCTL_GPIOHBCTL_PORTB: usize = 0x00000002; // Port B Advanced High-Performance Bus
pub const SYSCTL_GPIOHBCTL_PORTA: usize = 0x00000001; // Port A Advanced High-Performance Bus

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC2 register.
//
// *****************************************************************************
pub const SYSCTL_RCC2_USERCC2: usize = 0x80000000; // Use RCC2
pub const SYSCTL_RCC2_DIV400: usize = 0x40000000; // Divide PLL as 400 MHz vs. 200 MHz
pub const SYSCTL_RCC2_SYSDIV2_M: usize = 0x1F800000; // System Clock Divisor 2
pub const SYSCTL_RCC2_SYSDIV2_2: usize = 0x00800000; // System clock /2
pub const SYSCTL_RCC2_SYSDIV2_3: usize = 0x01000000; // System clock /3
pub const SYSCTL_RCC2_SYSDIV2_4: usize = 0x01800000; // System clock /4
pub const SYSCTL_RCC2_SYSDIV2_5: usize = 0x02000000; // System clock /5
pub const SYSCTL_RCC2_SYSDIV2_6: usize = 0x02800000; // System clock /6
pub const SYSCTL_RCC2_SYSDIV2_7: usize = 0x03000000; // System clock /7
pub const SYSCTL_RCC2_SYSDIV2_8: usize = 0x03800000; // System clock /8
pub const SYSCTL_RCC2_SYSDIV2_9: usize = 0x04000000; // System clock /9
pub const SYSCTL_RCC2_SYSDIV2_10: usize = 0x04800000; // System clock /10
pub const SYSCTL_RCC2_SYSDIV2_11: usize = 0x05000000; // System clock /11
pub const SYSCTL_RCC2_SYSDIV2_12: usize = 0x05800000; // System clock /12
pub const SYSCTL_RCC2_SYSDIV2_13: usize = 0x06000000; // System clock /13
pub const SYSCTL_RCC2_SYSDIV2_14: usize = 0x06800000; // System clock /14
pub const SYSCTL_RCC2_SYSDIV2_15: usize = 0x07000000; // System clock /15
pub const SYSCTL_RCC2_SYSDIV2_16: usize = 0x07800000; // System clock /16
pub const SYSCTL_RCC2_SYSDIV2_17: usize = 0x08000000; // System clock /17
pub const SYSCTL_RCC2_SYSDIV2_18: usize = 0x08800000; // System clock /18
pub const SYSCTL_RCC2_SYSDIV2_19: usize = 0x09000000; // System clock /19
pub const SYSCTL_RCC2_SYSDIV2_20: usize = 0x09800000; // System clock /20
pub const SYSCTL_RCC2_SYSDIV2_21: usize = 0x0A000000; // System clock /21
pub const SYSCTL_RCC2_SYSDIV2_22: usize = 0x0A800000; // System clock /22
pub const SYSCTL_RCC2_SYSDIV2_23: usize = 0x0B000000; // System clock /23
pub const SYSCTL_RCC2_SYSDIV2_24: usize = 0x0B800000; // System clock /24
pub const SYSCTL_RCC2_SYSDIV2_25: usize = 0x0C000000; // System clock /25
pub const SYSCTL_RCC2_SYSDIV2_26: usize = 0x0C800000; // System clock /26
pub const SYSCTL_RCC2_SYSDIV2_27: usize = 0x0D000000; // System clock /27
pub const SYSCTL_RCC2_SYSDIV2_28: usize = 0x0D800000; // System clock /28
pub const SYSCTL_RCC2_SYSDIV2_29: usize = 0x0E000000; // System clock /29
pub const SYSCTL_RCC2_SYSDIV2_30: usize = 0x0E800000; // System clock /30
pub const SYSCTL_RCC2_SYSDIV2_31: usize = 0x0F000000; // System clock /31
pub const SYSCTL_RCC2_SYSDIV2_32: usize = 0x0F800000; // System clock /32
pub const SYSCTL_RCC2_SYSDIV2_33: usize = 0x10000000; // System clock /33
pub const SYSCTL_RCC2_SYSDIV2_34: usize = 0x10800000; // System clock /34
pub const SYSCTL_RCC2_SYSDIV2_35: usize = 0x11000000; // System clock /35
pub const SYSCTL_RCC2_SYSDIV2_36: usize = 0x11800000; // System clock /36
pub const SYSCTL_RCC2_SYSDIV2_37: usize = 0x12000000; // System clock /37
pub const SYSCTL_RCC2_SYSDIV2_38: usize = 0x12800000; // System clock /38
pub const SYSCTL_RCC2_SYSDIV2_39: usize = 0x13000000; // System clock /39
pub const SYSCTL_RCC2_SYSDIV2_40: usize = 0x13800000; // System clock /40
pub const SYSCTL_RCC2_SYSDIV2_41: usize = 0x14000000; // System clock /41
pub const SYSCTL_RCC2_SYSDIV2_42: usize = 0x14800000; // System clock /42
pub const SYSCTL_RCC2_SYSDIV2_43: usize = 0x15000000; // System clock /43
pub const SYSCTL_RCC2_SYSDIV2_44: usize = 0x15800000; // System clock /44
pub const SYSCTL_RCC2_SYSDIV2_45: usize = 0x16000000; // System clock /45
pub const SYSCTL_RCC2_SYSDIV2_46: usize = 0x16800000; // System clock /46
pub const SYSCTL_RCC2_SYSDIV2_47: usize = 0x17000000; // System clock /47
pub const SYSCTL_RCC2_SYSDIV2_48: usize = 0x17800000; // System clock /48
pub const SYSCTL_RCC2_SYSDIV2_49: usize = 0x18000000; // System clock /49
pub const SYSCTL_RCC2_SYSDIV2_50: usize = 0x18800000; // System clock /50
pub const SYSCTL_RCC2_SYSDIV2_51: usize = 0x19000000; // System clock /51
pub const SYSCTL_RCC2_SYSDIV2_52: usize = 0x19800000; // System clock /52
pub const SYSCTL_RCC2_SYSDIV2_53: usize = 0x1A000000; // System clock /53
pub const SYSCTL_RCC2_SYSDIV2_54: usize = 0x1A800000; // System clock /54
pub const SYSCTL_RCC2_SYSDIV2_55: usize = 0x1B000000; // System clock /55
pub const SYSCTL_RCC2_SYSDIV2_56: usize = 0x1B800000; // System clock /56
pub const SYSCTL_RCC2_SYSDIV2_57: usize = 0x1C000000; // System clock /57
pub const SYSCTL_RCC2_SYSDIV2_58: usize = 0x1C800000; // System clock /58
pub const SYSCTL_RCC2_SYSDIV2_59: usize = 0x1D000000; // System clock /59
pub const SYSCTL_RCC2_SYSDIV2_60: usize = 0x1D800000; // System clock /60
pub const SYSCTL_RCC2_SYSDIV2_61: usize = 0x1E000000; // System clock /61
pub const SYSCTL_RCC2_SYSDIV2_62: usize = 0x1E800000; // System clock /62
pub const SYSCTL_RCC2_SYSDIV2_63: usize = 0x1F000000; // System clock /63
pub const SYSCTL_RCC2_SYSDIV2_64: usize = 0x1F800000; // System clock /64
pub const SYSCTL_RCC2_SYSDIV2LSB: usize = 0x00400000; // Additional LSB for SYSDIV2
pub const SYSCTL_RCC2_USBPWRDN: usize = 0x00004000; // Power-Down USB PLL
pub const SYSCTL_RCC2_PWRDN2: usize = 0x00002000; // Power-Down PLL 2
pub const SYSCTL_RCC2_BYPASS2: usize = 0x00000800; // PLL Bypass 2
pub const SYSCTL_RCC2_OSCSRC2_M: usize = 0x00000070; // Oscillator Source 2
pub const SYSCTL_RCC2_OSCSRC2_MO: usize = 0x00000000; // MOSC
pub const SYSCTL_RCC2_OSCSRC2_IO: usize = 0x00000010; // PIOSC
pub const SYSCTL_RCC2_OSCSRC2_IO4: usize = 0x00000020; // PIOSC/4
pub const SYSCTL_RCC2_OSCSRC2_30: usize = 0x00000030; // 30 kHz
pub const SYSCTL_RCC2_OSCSRC2_32: usize = 0x00000070; // 32.768 kHz
pub const SYSCTL_RCC2_SYSDIV2_S: usize = 23;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MOSCCTL register.
//
// *****************************************************************************
pub const SYSCTL_MOSCCTL_NOXTAL: usize = 0x00000004; // No Crystal Connected
pub const SYSCTL_MOSCCTL_MOSCIM: usize = 0x00000002; // MOSC Failure Action
pub const SYSCTL_MOSCCTL_CVAL: usize = 0x00000001; // Clock Validation for MOSC

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC0 register.
//
// *****************************************************************************
pub const SYSCTL_RCGC0_WDT1: usize = 0x10000000; // WDT1 Clock Gating Control
pub const SYSCTL_RCGC0_CAN1: usize = 0x02000000; // CAN1 Clock Gating Control
pub const SYSCTL_RCGC0_CAN0: usize = 0x01000000; // CAN0 Clock Gating Control
pub const SYSCTL_RCGC0_PWM0: usize = 0x00100000; // PWM Clock Gating Control
pub const SYSCTL_RCGC0_ADC1: usize = 0x00020000; // ADC1 Clock Gating Control
pub const SYSCTL_RCGC0_ADC0: usize = 0x00010000; // ADC0 Clock Gating Control
pub const SYSCTL_RCGC0_ADC1SPD_M: usize = 0x00000C00; // ADC1 Sample Speed
pub const SYSCTL_RCGC0_ADC1SPD_125K: usize = 0x00000000; // 125K samples/second
pub const SYSCTL_RCGC0_ADC1SPD_250K: usize = 0x00000400; // 250K samples/second
pub const SYSCTL_RCGC0_ADC1SPD_500K: usize = 0x00000800; // 500K samples/second
pub const SYSCTL_RCGC0_ADC1SPD_1M: usize = 0x00000C00; // 1M samples/second
pub const SYSCTL_RCGC0_ADC0SPD_M: usize = 0x00000300; // ADC0 Sample Speed
pub const SYSCTL_RCGC0_ADC0SPD_125K: usize = 0x00000000; // 125K samples/second
pub const SYSCTL_RCGC0_ADC0SPD_250K: usize = 0x00000100; // 250K samples/second
pub const SYSCTL_RCGC0_ADC0SPD_500K: usize = 0x00000200; // 500K samples/second
pub const SYSCTL_RCGC0_ADC0SPD_1M: usize = 0x00000300; // 1M samples/second
pub const SYSCTL_RCGC0_HIB: usize = 0x00000040; // HIB Clock Gating Control
pub const SYSCTL_RCGC0_WDT0: usize = 0x00000008; // WDT0 Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC1 register.
//
// *****************************************************************************
pub const SYSCTL_RCGC1_COMP2: usize = 0x04000000; // Analog Comparator 2 Clock Gating
pub const SYSCTL_RCGC1_COMP1: usize = 0x02000000; // Analog Comparator 1 Clock Gating
pub const SYSCTL_RCGC1_COMP0: usize = 0x01000000; // Analog Comparator 0 Clock Gating
pub const SYSCTL_RCGC1_TIMER3: usize = 0x00080000; // Timer 3 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER2: usize = 0x00040000; // Timer 2 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER1: usize = 0x00020000; // Timer 1 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER0: usize = 0x00010000; // Timer 0 Clock Gating Control
pub const SYSCTL_RCGC1_I2C1: usize = 0x00004000; // I2C1 Clock Gating Control
pub const SYSCTL_RCGC1_I2C0: usize = 0x00001000; // I2C0 Clock Gating Control
pub const SYSCTL_RCGC1_QEI1: usize = 0x00000200; // QEI1 Clock Gating Control
pub const SYSCTL_RCGC1_QEI0: usize = 0x00000100; // QEI0 Clock Gating Control
pub const SYSCTL_RCGC1_SSI1: usize = 0x00000020; // SSI1 Clock Gating Control
pub const SYSCTL_RCGC1_SSI0: usize = 0x00000010; // SSI0 Clock Gating Control
pub const SYSCTL_RCGC1_UART2: usize = 0x00000004; // UART2 Clock Gating Control
pub const SYSCTL_RCGC1_UART1: usize = 0x00000002; // UART1 Clock Gating Control
pub const SYSCTL_RCGC1_UART0: usize = 0x00000001; // UART0 Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC2 register.
//
// *****************************************************************************
pub const SYSCTL_RCGC2_USB0: usize = 0x00010000; // USB0 Clock Gating Control
pub const SYSCTL_RCGC2_UDMA: usize = 0x00002000; // Micro-DMA Clock Gating Control
pub const SYSCTL_RCGC2_GPIOJ: usize = 0x00000100; // Port J Clock Gating Control
pub const SYSCTL_RCGC2_GPIOH: usize = 0x00000080; // Port H Clock Gating Control
pub const SYSCTL_RCGC2_GPIOG: usize = 0x00000040; // Port G Clock Gating Control
pub const SYSCTL_RCGC2_GPIOF: usize = 0x00000020; // Port F Clock Gating Control
pub const SYSCTL_RCGC2_GPIOE: usize = 0x00000010; // Port E Clock Gating Control
pub const SYSCTL_RCGC2_GPIOD: usize = 0x00000008; // Port D Clock Gating Control
pub const SYSCTL_RCGC2_GPIOC: usize = 0x00000004; // Port C Clock Gating Control
pub const SYSCTL_RCGC2_GPIOB: usize = 0x00000002; // Port B Clock Gating Control
pub const SYSCTL_RCGC2_GPIOA: usize = 0x00000001; // Port A Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC0 register.
//
// *****************************************************************************
pub const SYSCTL_SCGC0_WDT1: usize = 0x10000000; // WDT1 Clock Gating Control
pub const SYSCTL_SCGC0_CAN1: usize = 0x02000000; // CAN1 Clock Gating Control
pub const SYSCTL_SCGC0_CAN0: usize = 0x01000000; // CAN0 Clock Gating Control
pub const SYSCTL_SCGC0_PWM0: usize = 0x00100000; // PWM Clock Gating Control
pub const SYSCTL_SCGC0_ADC1: usize = 0x00020000; // ADC1 Clock Gating Control
pub const SYSCTL_SCGC0_ADC0: usize = 0x00010000; // ADC0 Clock Gating Control
pub const SYSCTL_SCGC0_HIB: usize = 0x00000040; // HIB Clock Gating Control
pub const SYSCTL_SCGC0_WDT0: usize = 0x00000008; // WDT0 Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC1 register.
//
// *****************************************************************************
pub const SYSCTL_SCGC1_COMP2: usize = 0x04000000; // Analog Comparator 2 Clock Gating
pub const SYSCTL_SCGC1_COMP1: usize = 0x02000000; // Analog Comparator 1 Clock Gating
pub const SYSCTL_SCGC1_COMP0: usize = 0x01000000; // Analog Comparator 0 Clock Gating
pub const SYSCTL_SCGC1_TIMER3: usize = 0x00080000; // Timer 3 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER2: usize = 0x00040000; // Timer 2 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER1: usize = 0x00020000; // Timer 1 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER0: usize = 0x00010000; // Timer 0 Clock Gating Control
pub const SYSCTL_SCGC1_I2C1: usize = 0x00004000; // I2C1 Clock Gating Control
pub const SYSCTL_SCGC1_I2C0: usize = 0x00001000; // I2C0 Clock Gating Control
pub const SYSCTL_SCGC1_QEI1: usize = 0x00000200; // QEI1 Clock Gating Control
pub const SYSCTL_SCGC1_QEI0: usize = 0x00000100; // QEI0 Clock Gating Control
pub const SYSCTL_SCGC1_SSI1: usize = 0x00000020; // SSI1 Clock Gating Control
pub const SYSCTL_SCGC1_SSI0: usize = 0x00000010; // SSI0 Clock Gating Control
pub const SYSCTL_SCGC1_UART2: usize = 0x00000004; // UART2 Clock Gating Control
pub const SYSCTL_SCGC1_UART1: usize = 0x00000002; // UART1 Clock Gating Control
pub const SYSCTL_SCGC1_UART0: usize = 0x00000001; // UART0 Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC2 register.
//
// *****************************************************************************
pub const SYSCTL_SCGC2_USB0: usize = 0x00010000; // USB0 Clock Gating Control
pub const SYSCTL_SCGC2_UDMA: usize = 0x00002000; // Micro-DMA Clock Gating Control
pub const SYSCTL_SCGC2_GPIOJ: usize = 0x00000100; // Port J Clock Gating Control
pub const SYSCTL_SCGC2_GPIOH: usize = 0x00000080; // Port H Clock Gating Control
pub const SYSCTL_SCGC2_GPIOG: usize = 0x00000040; // Port G Clock Gating Control
pub const SYSCTL_SCGC2_GPIOF: usize = 0x00000020; // Port F Clock Gating Control
pub const SYSCTL_SCGC2_GPIOE: usize = 0x00000010; // Port E Clock Gating Control
pub const SYSCTL_SCGC2_GPIOD: usize = 0x00000008; // Port D Clock Gating Control
pub const SYSCTL_SCGC2_GPIOC: usize = 0x00000004; // Port C Clock Gating Control
pub const SYSCTL_SCGC2_GPIOB: usize = 0x00000002; // Port B Clock Gating Control
pub const SYSCTL_SCGC2_GPIOA: usize = 0x00000001; // Port A Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC0 register.
//
// *****************************************************************************
pub const SYSCTL_DCGC0_WDT1: usize = 0x10000000; // WDT1 Clock Gating Control
pub const SYSCTL_DCGC0_CAN1: usize = 0x02000000; // CAN1 Clock Gating Control
pub const SYSCTL_DCGC0_CAN0: usize = 0x01000000; // CAN0 Clock Gating Control
pub const SYSCTL_DCGC0_PWM0: usize = 0x00100000; // PWM Clock Gating Control
pub const SYSCTL_DCGC0_ADC1: usize = 0x00020000; // ADC1 Clock Gating Control
pub const SYSCTL_DCGC0_ADC0: usize = 0x00010000; // ADC0 Clock Gating Control
pub const SYSCTL_DCGC0_HIB: usize = 0x00000040; // HIB Clock Gating Control
pub const SYSCTL_DCGC0_WDT0: usize = 0x00000008; // WDT0 Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC1 register.
//
// *****************************************************************************
pub const SYSCTL_DCGC1_COMP2: usize = 0x04000000; // Analog Comparator 2 Clock Gating
pub const SYSCTL_DCGC1_COMP1: usize = 0x02000000; // Analog Comparator 1 Clock Gating
pub const SYSCTL_DCGC1_COMP0: usize = 0x01000000; // Analog Comparator 0 Clock Gating
pub const SYSCTL_DCGC1_TIMER3: usize = 0x00080000; // Timer 3 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER2: usize = 0x00040000; // Timer 2 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER1: usize = 0x00020000; // Timer 1 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER0: usize = 0x00010000; // Timer 0 Clock Gating Control
pub const SYSCTL_DCGC1_I2C1: usize = 0x00004000; // I2C1 Clock Gating Control
pub const SYSCTL_DCGC1_I2C0: usize = 0x00001000; // I2C0 Clock Gating Control
pub const SYSCTL_DCGC1_QEI1: usize = 0x00000200; // QEI1 Clock Gating Control
pub const SYSCTL_DCGC1_QEI0: usize = 0x00000100; // QEI0 Clock Gating Control
pub const SYSCTL_DCGC1_SSI1: usize = 0x00000020; // SSI1 Clock Gating Control
pub const SYSCTL_DCGC1_SSI0: usize = 0x00000010; // SSI0 Clock Gating Control
pub const SYSCTL_DCGC1_UART2: usize = 0x00000004; // UART2 Clock Gating Control
pub const SYSCTL_DCGC1_UART1: usize = 0x00000002; // UART1 Clock Gating Control
pub const SYSCTL_DCGC1_UART0: usize = 0x00000001; // UART0 Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC2 register.
//
// *****************************************************************************
pub const SYSCTL_DCGC2_USB0: usize = 0x00010000; // USB0 Clock Gating Control
pub const SYSCTL_DCGC2_UDMA: usize = 0x00002000; // Micro-DMA Clock Gating Control
pub const SYSCTL_DCGC2_GPIOJ: usize = 0x00000100; // Port J Clock Gating Control
pub const SYSCTL_DCGC2_GPIOH: usize = 0x00000080; // Port H Clock Gating Control
pub const SYSCTL_DCGC2_GPIOG: usize = 0x00000040; // Port G Clock Gating Control
pub const SYSCTL_DCGC2_GPIOF: usize = 0x00000020; // Port F Clock Gating Control
pub const SYSCTL_DCGC2_GPIOE: usize = 0x00000010; // Port E Clock Gating Control
pub const SYSCTL_DCGC2_GPIOD: usize = 0x00000008; // Port D Clock Gating Control
pub const SYSCTL_DCGC2_GPIOC: usize = 0x00000004; // Port C Clock Gating Control
pub const SYSCTL_DCGC2_GPIOB: usize = 0x00000002; // Port B Clock Gating Control
pub const SYSCTL_DCGC2_GPIOA: usize = 0x00000001; // Port A Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPCLKCFG
// register.
//
// *****************************************************************************
pub const SYSCTL_DSLPCLKCFG_D_M: usize = 0x1F800000; // Divider Field Override
pub const SYSCTL_DSLPCLKCFG_D_1: usize = 0x00000000; // System clock /1
pub const SYSCTL_DSLPCLKCFG_D_2: usize = 0x00800000; // System clock /2
pub const SYSCTL_DSLPCLKCFG_D_3: usize = 0x01000000; // System clock /3
pub const SYSCTL_DSLPCLKCFG_D_4: usize = 0x01800000; // System clock /4
pub const SYSCTL_DSLPCLKCFG_D_64: usize = 0x1F800000; // System clock /64
pub const SYSCTL_DSLPCLKCFG_O_M: usize = 0x00000070; // Clock Source
pub const SYSCTL_DSLPCLKCFG_O_IGN: usize = 0x00000000; // MOSC
pub const SYSCTL_DSLPCLKCFG_O_IO: usize = 0x00000010; // PIOSC
pub const SYSCTL_DSLPCLKCFG_O_30: usize = 0x00000030; // 30 kHz
pub const SYSCTL_DSLPCLKCFG_O_32: usize = 0x00000070; // 32.768 kHz

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SYSPROP register.
//
// *****************************************************************************
pub const SYSCTL_SYSPROP_FPU: usize = 0x00000001; // FPU Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCCAL
// register.
//
// *****************************************************************************
pub const SYSCTL_PIOSCCAL_UTEN: usize = 0x80000000; // Use User Trim Value
pub const SYSCTL_PIOSCCAL_CAL: usize = 0x00000200; // Start Calibration
pub const SYSCTL_PIOSCCAL_UPDATE: usize = 0x00000100; // Update Trim
pub const SYSCTL_PIOSCCAL_UT_M: usize = 0x0000007F; // User Trim Value
pub const SYSCTL_PIOSCCAL_UT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCSTAT
// register.
//
// *****************************************************************************
pub const SYSCTL_PIOSCSTAT_DT_M: usize = 0x007F0000; // Default Trim Value
pub const SYSCTL_PIOSCSTAT_CR_M: usize = 0x00000300; // Calibration Result
pub const SYSCTL_PIOSCSTAT_CRNONE: usize = 0x00000000; // Calibration has not been attempted
pub const SYSCTL_PIOSCSTAT_CRPASS: usize = 0x00000100; // The last calibration operation completed to meet 1% accuracy
pub const SYSCTL_PIOSCSTAT_CRFAIL: usize = 0x00000200; // The last calibration operation failed to meet 1% accuracy
pub const SYSCTL_PIOSCSTAT_CT_M: usize = 0x0000007F; // Calibration Trim Value
pub const SYSCTL_PIOSCSTAT_DT_S: usize = 16;
pub const SYSCTL_PIOSCSTAT_CT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ0
// register.
//
// *****************************************************************************
pub const SYSCTL_PLLFREQ0_MFRAC_M: usize = 0x000FFC00; // PLL M Fractional Value
pub const SYSCTL_PLLFREQ0_MINT_M: usize = 0x000003FF; // PLL M Integer Value
pub const SYSCTL_PLLFREQ0_MFRAC_S: usize = 10;
pub const SYSCTL_PLLFREQ0_MINT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ1
// register.
//
// *****************************************************************************
pub const SYSCTL_PLLFREQ1_Q_M: usize = 0x00001F00; // PLL Q Value
pub const SYSCTL_PLLFREQ1_N_M: usize = 0x0000001F; // PLL N Value
pub const SYSCTL_PLLFREQ1_Q_S: usize = 8;
pub const SYSCTL_PLLFREQ1_N_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLSTAT register.
//
// *****************************************************************************
pub const SYSCTL_PLLSTAT_LOCK: usize = 0x00000001; // PLL Lock

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC9 register.
//
// *****************************************************************************
pub const SYSCTL_DC9_ADC1DC7: usize = 0x00800000; // ADC1 DC7 Present
pub const SYSCTL_DC9_ADC1DC6: usize = 0x00400000; // ADC1 DC6 Present
pub const SYSCTL_DC9_ADC1DC5: usize = 0x00200000; // ADC1 DC5 Present
pub const SYSCTL_DC9_ADC1DC4: usize = 0x00100000; // ADC1 DC4 Present
pub const SYSCTL_DC9_ADC1DC3: usize = 0x00080000; // ADC1 DC3 Present
pub const SYSCTL_DC9_ADC1DC2: usize = 0x00040000; // ADC1 DC2 Present
pub const SYSCTL_DC9_ADC1DC1: usize = 0x00020000; // ADC1 DC1 Present
pub const SYSCTL_DC9_ADC1DC0: usize = 0x00010000; // ADC1 DC0 Present
pub const SYSCTL_DC9_ADC0DC7: usize = 0x00000080; // ADC0 DC7 Present
pub const SYSCTL_DC9_ADC0DC6: usize = 0x00000040; // ADC0 DC6 Present
pub const SYSCTL_DC9_ADC0DC5: usize = 0x00000020; // ADC0 DC5 Present
pub const SYSCTL_DC9_ADC0DC4: usize = 0x00000010; // ADC0 DC4 Present
pub const SYSCTL_DC9_ADC0DC3: usize = 0x00000008; // ADC0 DC3 Present
pub const SYSCTL_DC9_ADC0DC2: usize = 0x00000004; // ADC0 DC2 Present
pub const SYSCTL_DC9_ADC0DC1: usize = 0x00000002; // ADC0 DC1 Present
pub const SYSCTL_DC9_ADC0DC0: usize = 0x00000001; // ADC0 DC0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_NVMSTAT register.
//
// *****************************************************************************
pub const SYSCTL_NVMSTAT_TPSW: usize = 0x00000010; // Third Party Software Present
pub const SYSCTL_NVMSTAT_FWB: usize = 0x00000001; // 32 Word Flash Write Buffer Active

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWD register.
//
// *****************************************************************************
pub const SYSCTL_PPWD_P1: usize = 0x00000002; // Watchdog Timer 1 Present
pub const SYSCTL_PPWD_P0: usize = 0x00000001; // Watchdog Timer 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPTIMER register.
//
// *****************************************************************************
pub const SYSCTL_PPTIMER_P5: usize = 0x00000020; // Timer 5 Present
pub const SYSCTL_PPTIMER_P4: usize = 0x00000010; // Timer 4 Present
pub const SYSCTL_PPTIMER_P3: usize = 0x00000008; // Timer 3 Present
pub const SYSCTL_PPTIMER_P2: usize = 0x00000004; // Timer 2 Present
pub const SYSCTL_PPTIMER_P1: usize = 0x00000002; // Timer 1 Present
pub const SYSCTL_PPTIMER_P0: usize = 0x00000001; // Timer 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPGPIO register.
//
// *****************************************************************************
pub const SYSCTL_PPGPIO_P14: usize = 0x00004000; // GPIO Port Q Present
pub const SYSCTL_PPGPIO_P13: usize = 0x00002000; // GPIO Port P Present
pub const SYSCTL_PPGPIO_P12: usize = 0x00001000; // GPIO Port N Present
pub const SYSCTL_PPGPIO_P11: usize = 0x00000800; // GPIO Port M Present
pub const SYSCTL_PPGPIO_P10: usize = 0x00000400; // GPIO Port L Present
pub const SYSCTL_PPGPIO_P9: usize = 0x00000200; // GPIO Port K Present
pub const SYSCTL_PPGPIO_P8: usize = 0x00000100; // GPIO Port J Present
pub const SYSCTL_PPGPIO_P7: usize = 0x00000080; // GPIO Port H Present
pub const SYSCTL_PPGPIO_P6: usize = 0x00000040; // GPIO Port G Present
pub const SYSCTL_PPGPIO_P5: usize = 0x00000020; // GPIO Port F Present
pub const SYSCTL_PPGPIO_P4: usize = 0x00000010; // GPIO Port E Present
pub const SYSCTL_PPGPIO_P3: usize = 0x00000008; // GPIO Port D Present
pub const SYSCTL_PPGPIO_P2: usize = 0x00000004; // GPIO Port C Present
pub const SYSCTL_PPGPIO_P1: usize = 0x00000002; // GPIO Port B Present
pub const SYSCTL_PPGPIO_P0: usize = 0x00000001; // GPIO Port A Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPDMA register.
//
// *****************************************************************************
pub const SYSCTL_PPDMA_P0: usize = 0x00000001; // uDMA Module Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPHIB register.
//
// *****************************************************************************
pub const SYSCTL_PPHIB_P0: usize = 0x00000001; // Hibernation Module Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUART register.
//
// *****************************************************************************
pub const SYSCTL_PPUART_P7: usize = 0x00000080; // UART Module 7 Present
pub const SYSCTL_PPUART_P6: usize = 0x00000040; // UART Module 6 Present
pub const SYSCTL_PPUART_P5: usize = 0x00000020; // UART Module 5 Present
pub const SYSCTL_PPUART_P4: usize = 0x00000010; // UART Module 4 Present
pub const SYSCTL_PPUART_P3: usize = 0x00000008; // UART Module 3 Present
pub const SYSCTL_PPUART_P2: usize = 0x00000004; // UART Module 2 Present
pub const SYSCTL_PPUART_P1: usize = 0x00000002; // UART Module 1 Present
pub const SYSCTL_PPUART_P0: usize = 0x00000001; // UART Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPSSI register.
//
// *****************************************************************************
pub const SYSCTL_PPSSI_P3: usize = 0x00000008; // SSI Module 3 Present
pub const SYSCTL_PPSSI_P2: usize = 0x00000004; // SSI Module 2 Present
pub const SYSCTL_PPSSI_P1: usize = 0x00000002; // SSI Module 1 Present
pub const SYSCTL_PPSSI_P0: usize = 0x00000001; // SSI Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPI2C register.
//
// *****************************************************************************
pub const SYSCTL_PPI2C_P5: usize = 0x00000020; // I2C Module 5 Present
pub const SYSCTL_PPI2C_P4: usize = 0x00000010; // I2C Module 4 Present
pub const SYSCTL_PPI2C_P3: usize = 0x00000008; // I2C Module 3 Present
pub const SYSCTL_PPI2C_P2: usize = 0x00000004; // I2C Module 2 Present
pub const SYSCTL_PPI2C_P1: usize = 0x00000002; // I2C Module 1 Present
pub const SYSCTL_PPI2C_P0: usize = 0x00000001; // I2C Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUSB register.
//
// *****************************************************************************
pub const SYSCTL_PPUSB_P0: usize = 0x00000001; // USB Module Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPCAN register.
//
// *****************************************************************************
pub const SYSCTL_PPCAN_P1: usize = 0x00000002; // CAN Module 1 Present
pub const SYSCTL_PPCAN_P0: usize = 0x00000001; // CAN Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPADC register.
//
// *****************************************************************************
pub const SYSCTL_PPADC_P1: usize = 0x00000002; // ADC Module 1 Present
pub const SYSCTL_PPADC_P0: usize = 0x00000001; // ADC Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPACMP register.
//
// *****************************************************************************
pub const SYSCTL_PPACMP_P0: usize = 0x00000001; // Analog Comparator Module Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPPWM register.
//
// *****************************************************************************
pub const SYSCTL_PPPWM_P1: usize = 0x00000002; // PWM Module 1 Present
pub const SYSCTL_PPPWM_P0: usize = 0x00000001; // PWM Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPQEI register.
//
// *****************************************************************************
pub const SYSCTL_PPQEI_P1: usize = 0x00000002; // QEI Module 1 Present
pub const SYSCTL_PPQEI_P0: usize = 0x00000001; // QEI Module 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEEPROM
// register.
//
// *****************************************************************************
pub const SYSCTL_PPEEPROM_P0: usize = 0x00000001; // EEPROM Module Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_PPWTIMER_P5: usize = 0x00000020; // Wide Timer 5 Present
pub const SYSCTL_PPWTIMER_P4: usize = 0x00000010; // Wide Timer 4 Present
pub const SYSCTL_PPWTIMER_P3: usize = 0x00000008; // Wide Timer 3 Present
pub const SYSCTL_PPWTIMER_P2: usize = 0x00000004; // Wide Timer 2 Present
pub const SYSCTL_PPWTIMER_P1: usize = 0x00000002; // Wide Timer 1 Present
pub const SYSCTL_PPWTIMER_P0: usize = 0x00000001; // Wide Timer 0 Present

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWD register.
//
// *****************************************************************************
pub const SYSCTL_SRWD_R1: usize = 0x00000002; // Watchdog Timer 1 Software Reset
pub const SYSCTL_SRWD_R0: usize = 0x00000001; // Watchdog Timer 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRTIMER register.
//
// *****************************************************************************
pub const SYSCTL_SRTIMER_R5: usize = 0x00000020; // Timer 5 Software Reset
pub const SYSCTL_SRTIMER_R4: usize = 0x00000010; // Timer 4 Software Reset
pub const SYSCTL_SRTIMER_R3: usize = 0x00000008; // Timer 3 Software Reset
pub const SYSCTL_SRTIMER_R2: usize = 0x00000004; // Timer 2 Software Reset
pub const SYSCTL_SRTIMER_R1: usize = 0x00000002; // Timer 1 Software Reset
pub const SYSCTL_SRTIMER_R0: usize = 0x00000001; // Timer 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRGPIO register.
//
// *****************************************************************************
pub const SYSCTL_SRGPIO_R14: usize = 0x00004000; // GPIO Port Q Software Reset
pub const SYSCTL_SRGPIO_R13: usize = 0x00002000; // GPIO Port P Software Reset
pub const SYSCTL_SRGPIO_R12: usize = 0x00001000; // GPIO Port N Software Reset
pub const SYSCTL_SRGPIO_R11: usize = 0x00000800; // GPIO Port M Software Reset
pub const SYSCTL_SRGPIO_R10: usize = 0x00000400; // GPIO Port L Software Reset
pub const SYSCTL_SRGPIO_R9: usize = 0x00000200; // GPIO Port K Software Reset
pub const SYSCTL_SRGPIO_R8: usize = 0x00000100; // GPIO Port J Software Reset
pub const SYSCTL_SRGPIO_R7: usize = 0x00000080; // GPIO Port H Software Reset
pub const SYSCTL_SRGPIO_R6: usize = 0x00000040; // GPIO Port G Software Reset
pub const SYSCTL_SRGPIO_R5: usize = 0x00000020; // GPIO Port F Software Reset
pub const SYSCTL_SRGPIO_R4: usize = 0x00000010; // GPIO Port E Software Reset
pub const SYSCTL_SRGPIO_R3: usize = 0x00000008; // GPIO Port D Software Reset
pub const SYSCTL_SRGPIO_R2: usize = 0x00000004; // GPIO Port C Software Reset
pub const SYSCTL_SRGPIO_R1: usize = 0x00000002; // GPIO Port B Software Reset
pub const SYSCTL_SRGPIO_R0: usize = 0x00000001; // GPIO Port A Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRDMA register.
//
// *****************************************************************************
pub const SYSCTL_SRDMA_R0: usize = 0x00000001; // uDMA Module Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRHIB register.
//
// *****************************************************************************
pub const SYSCTL_SRHIB_R0: usize = 0x00000001; // Hibernation Module Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUART register.
//
// *****************************************************************************
pub const SYSCTL_SRUART_R7: usize = 0x00000080; // UART Module 7 Software Reset
pub const SYSCTL_SRUART_R6: usize = 0x00000040; // UART Module 6 Software Reset
pub const SYSCTL_SRUART_R5: usize = 0x00000020; // UART Module 5 Software Reset
pub const SYSCTL_SRUART_R4: usize = 0x00000010; // UART Module 4 Software Reset
pub const SYSCTL_SRUART_R3: usize = 0x00000008; // UART Module 3 Software Reset
pub const SYSCTL_SRUART_R2: usize = 0x00000004; // UART Module 2 Software Reset
pub const SYSCTL_SRUART_R1: usize = 0x00000002; // UART Module 1 Software Reset
pub const SYSCTL_SRUART_R0: usize = 0x00000001; // UART Module 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRSSI register.
//
// *****************************************************************************
pub const SYSCTL_SRSSI_R3: usize = 0x00000008; // SSI Module 3 Software Reset
pub const SYSCTL_SRSSI_R2: usize = 0x00000004; // SSI Module 2 Software Reset
pub const SYSCTL_SRSSI_R1: usize = 0x00000002; // SSI Module 1 Software Reset
pub const SYSCTL_SRSSI_R0: usize = 0x00000001; // SSI Module 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRI2C register.
//
// *****************************************************************************
pub const SYSCTL_SRI2C_R5: usize = 0x00000020; // I2C Module 5 Software Reset
pub const SYSCTL_SRI2C_R4: usize = 0x00000010; // I2C Module 4 Software Reset
pub const SYSCTL_SRI2C_R3: usize = 0x00000008; // I2C Module 3 Software Reset
pub const SYSCTL_SRI2C_R2: usize = 0x00000004; // I2C Module 2 Software Reset
pub const SYSCTL_SRI2C_R1: usize = 0x00000002; // I2C Module 1 Software Reset
pub const SYSCTL_SRI2C_R0: usize = 0x00000001; // I2C Module 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUSB register.
//
// *****************************************************************************
pub const SYSCTL_SRUSB_R0: usize = 0x00000001; // USB Module Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCAN register.
//
// *****************************************************************************
pub const SYSCTL_SRCAN_R1: usize = 0x00000002; // CAN Module 1 Software Reset
pub const SYSCTL_SRCAN_R0: usize = 0x00000001; // CAN Module 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRADC register.
//
// *****************************************************************************
pub const SYSCTL_SRADC_R1: usize = 0x00000002; // ADC Module 1 Software Reset
pub const SYSCTL_SRADC_R0: usize = 0x00000001; // ADC Module 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRACMP register.
//
// *****************************************************************************
pub const SYSCTL_SRACMP_R0: usize = 0x00000001; // Analog Comparator Module 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREEPROM
// register.
//
// *****************************************************************************
pub const SYSCTL_SREEPROM_R0: usize = 0x00000001; // EEPROM Module Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_SRWTIMER_R5: usize = 0x00000020; // Wide Timer 5 Software Reset
pub const SYSCTL_SRWTIMER_R4: usize = 0x00000010; // Wide Timer 4 Software Reset
pub const SYSCTL_SRWTIMER_R3: usize = 0x00000008; // Wide Timer 3 Software Reset
pub const SYSCTL_SRWTIMER_R2: usize = 0x00000004; // Wide Timer 2 Software Reset
pub const SYSCTL_SRWTIMER_R1: usize = 0x00000002; // Wide Timer 1 Software Reset
pub const SYSCTL_SRWTIMER_R0: usize = 0x00000001; // Wide Timer 0 Software Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWD register.
//
// *****************************************************************************
pub const SYSCTL_RCGCWD_R1: usize = 0x00000002; // Watchdog Timer 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCWD_R0: usize = 0x00000001; // Watchdog Timer 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_RCGCTIMER_R5: usize = 0x00000020; // Timer 5 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R4: usize = 0x00000010; // Timer 4 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R3: usize = 0x00000008; // Timer 3 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R2: usize = 0x00000004; // Timer 2 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R1: usize = 0x00000002; // Timer 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R0: usize = 0x00000001; // Timer 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCGPIO
// register.
//
// *****************************************************************************
pub const SYSCTL_RCGCGPIO_R14: usize = 0x00004000; // GPIO Port Q Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R13: usize = 0x00002000; // GPIO Port P Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R12: usize = 0x00001000; // GPIO Port N Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R11: usize = 0x00000800; // GPIO Port M Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R10: usize = 0x00000400; // GPIO Port L Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R9: usize = 0x00000200; // GPIO Port K Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R8: usize = 0x00000100; // GPIO Port J Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R7: usize = 0x00000080; // GPIO Port H Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R6: usize = 0x00000040; // GPIO Port G Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R5: usize = 0x00000020; // GPIO Port F Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R4: usize = 0x00000010; // GPIO Port E Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R3: usize = 0x00000008; // GPIO Port D Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R2: usize = 0x00000004; // GPIO Port C Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R1: usize = 0x00000002; // GPIO Port B Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO_R0: usize = 0x00000001; // GPIO Port A Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCDMA register.
//
// *****************************************************************************
pub const SYSCTL_RCGCDMA_R0: usize = 0x00000001; // uDMA Module Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCHIB register.
//
// *****************************************************************************
pub const SYSCTL_RCGCHIB_R0: usize = 0x00000001; // Hibernation Module Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUART
// register.
//
// *****************************************************************************
pub const SYSCTL_RCGCUART_R7: usize = 0x00000080; // UART Module 7 Run Mode Clock Gating Control
pub const SYSCTL_RCGCUART_R6: usize = 0x00000040; // UART Module 6 Run Mode Clock Gating Control
pub const SYSCTL_RCGCUART_R5: usize = 0x00000020; // UART Module 5 Run Mode Clock Gating Control
pub const SYSCTL_RCGCUART_R4: usize = 0x00000010; // UART Module 4 Run Mode Clock Gating Control
pub const SYSCTL_RCGCUART_R3: usize = 0x00000008; // UART Module 3 Run Mode Clock Gating Control
pub const SYSCTL_RCGCUART_R2: usize = 0x00000004; // UART Module 2 Run Mode Clock Gating Control
pub const SYSCTL_RCGCUART_R1: usize = 0x00000002; // UART Module 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCUART_R0: usize = 0x00000001; // UART Module 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCSSI register.
//
// *****************************************************************************
pub const SYSCTL_RCGCSSI_R3: usize = 0x00000008; // SSI Module 3 Run Mode Clock Gating Control
pub const SYSCTL_RCGCSSI_R2: usize = 0x00000004; // SSI Module 2 Run Mode Clock Gating Control
pub const SYSCTL_RCGCSSI_R1: usize = 0x00000002; // SSI Module 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCSSI_R0: usize = 0x00000001; // SSI Module 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCI2C register.
//
// *****************************************************************************
pub const SYSCTL_RCGCI2C_R5: usize = 0x00000020; // I2C Module 5 Run Mode Clock Gating Control
pub const SYSCTL_RCGCI2C_R4: usize = 0x00000010; // I2C Module 4 Run Mode Clock Gating Control
pub const SYSCTL_RCGCI2C_R3: usize = 0x00000008; // I2C Module 3 Run Mode Clock Gating Control
pub const SYSCTL_RCGCI2C_R2: usize = 0x00000004; // I2C Module 2 Run Mode Clock Gating Control
pub const SYSCTL_RCGCI2C_R1: usize = 0x00000002; // I2C Module 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCI2C_R0: usize = 0x00000001; // I2C Module 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUSB register.
//
// *****************************************************************************
pub const SYSCTL_RCGCUSB_R0: usize = 0x00000001; // USB Module Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCCAN register.
//
// *****************************************************************************
pub const SYSCTL_RCGCCAN_R1: usize = 0x00000002; // CAN Module 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCCAN_R0: usize = 0x00000001; // CAN Module 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCADC register.
//
// *****************************************************************************
pub const SYSCTL_RCGCADC_R1: usize = 0x00000002; // ADC Module 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCADC_R0: usize = 0x00000001; // ADC Module 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCACMP
// register.
//
// *****************************************************************************
pub const SYSCTL_RCGCACMP_R0: usize = 0x00000001; // Analog Comparator Module 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEEPROM
// register.
//
// *****************************************************************************
pub const SYSCTL_RCGCEEPROM_R0: usize = 0x00000001; // EEPROM Module Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_RCGCWTIMER_R5: usize = 0x00000020; // Wide Timer 5 Run Mode Clock Gating Control
pub const SYSCTL_RCGCWTIMER_R4: usize = 0x00000010; // Wide Timer 4 Run Mode Clock Gating Control
pub const SYSCTL_RCGCWTIMER_R3: usize = 0x00000008; // Wide Timer 3 Run Mode Clock Gating Control
pub const SYSCTL_RCGCWTIMER_R2: usize = 0x00000004; // Wide Timer 2 Run Mode Clock Gating Control
pub const SYSCTL_RCGCWTIMER_R1: usize = 0x00000002; // Wide Timer 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCWTIMER_R0: usize = 0x00000001; // Wide Timer 0 Run Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWD register.
//
// *****************************************************************************
pub const SYSCTL_SCGCWD_S1: usize = 0x00000002; // Watchdog Timer 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCWD_S0: usize = 0x00000001; // Watchdog Timer 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_SCGCTIMER_S5: usize = 0x00000020; // Timer 5 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCTIMER_S4: usize = 0x00000010; // Timer 4 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCTIMER_S3: usize = 0x00000008; // Timer 3 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCTIMER_S2: usize = 0x00000004; // Timer 2 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCTIMER_S1: usize = 0x00000002; // Timer 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCTIMER_S0: usize = 0x00000001; // Timer 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCGPIO
// register.
//
// *****************************************************************************
pub const SYSCTL_SCGCGPIO_S14: usize = 0x00004000; // GPIO Port Q Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S13: usize = 0x00002000; // GPIO Port P Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S12: usize = 0x00001000; // GPIO Port N Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S11: usize = 0x00000800; // GPIO Port M Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S10: usize = 0x00000400; // GPIO Port L Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S9: usize = 0x00000200; // GPIO Port K Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S8: usize = 0x00000100; // GPIO Port J Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S7: usize = 0x00000080; // GPIO Port H Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S6: usize = 0x00000040; // GPIO Port G Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S5: usize = 0x00000020; // GPIO Port F Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S4: usize = 0x00000010; // GPIO Port E Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S3: usize = 0x00000008; // GPIO Port D Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S2: usize = 0x00000004; // GPIO Port C Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S1: usize = 0x00000002; // GPIO Port B Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO_S0: usize = 0x00000001; // GPIO Port A Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCDMA register.
//
// *****************************************************************************
pub const SYSCTL_SCGCDMA_S0: usize = 0x00000001; // uDMA Module Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCHIB register.
//
// *****************************************************************************
pub const SYSCTL_SCGCHIB_S0: usize = 0x00000001; // Hibernation Module Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUART
// register.
//
// *****************************************************************************
pub const SYSCTL_SCGCUART_S7: usize = 0x00000080; // UART Module 7 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCUART_S6: usize = 0x00000040; // UART Module 6 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCUART_S5: usize = 0x00000020; // UART Module 5 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCUART_S4: usize = 0x00000010; // UART Module 4 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCUART_S3: usize = 0x00000008; // UART Module 3 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCUART_S2: usize = 0x00000004; // UART Module 2 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCUART_S1: usize = 0x00000002; // UART Module 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCUART_S0: usize = 0x00000001; // UART Module 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCSSI register.
//
// *****************************************************************************
pub const SYSCTL_SCGCSSI_S3: usize = 0x00000008; // SSI Module 3 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCSSI_S2: usize = 0x00000004; // SSI Module 2 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCSSI_S1: usize = 0x00000002; // SSI Module 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCSSI_S0: usize = 0x00000001; // SSI Module 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCI2C register.
//
// *****************************************************************************
pub const SYSCTL_SCGCI2C_S5: usize = 0x00000020; // I2C Module 5 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCI2C_S4: usize = 0x00000010; // I2C Module 4 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCI2C_S3: usize = 0x00000008; // I2C Module 3 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCI2C_S2: usize = 0x00000004; // I2C Module 2 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCI2C_S1: usize = 0x00000002; // I2C Module 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCI2C_S0: usize = 0x00000001; // I2C Module 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUSB register.
//
// *****************************************************************************
pub const SYSCTL_SCGCUSB_S0: usize = 0x00000001; // USB Module Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCCAN register.
//
// *****************************************************************************
pub const SYSCTL_SCGCCAN_S1: usize = 0x00000002; // CAN Module 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCCAN_S0: usize = 0x00000001; // CAN Module 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCADC register.
//
// *****************************************************************************
pub const SYSCTL_SCGCADC_S1: usize = 0x00000002; // ADC Module 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCADC_S0: usize = 0x00000001; // ADC Module 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCACMP
// register.
//
// *****************************************************************************
pub const SYSCTL_SCGCACMP_S0: usize = 0x00000001; // Analog Comparator Module 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEEPROM
// register.
//
// *****************************************************************************
pub const SYSCTL_SCGCEEPROM_S0: usize = 0x00000001; // EEPROM Module Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_SCGCWTIMER_S5: usize = 0x00000020; // Wide Timer 5 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCWTIMER_S4: usize = 0x00000010; // Wide Timer 4 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCWTIMER_S3: usize = 0x00000008; // Wide Timer 3 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCWTIMER_S2: usize = 0x00000004; // Wide Timer 2 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCWTIMER_S1: usize = 0x00000002; // Wide Timer 1 Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCWTIMER_S0: usize = 0x00000001; // Wide Timer 0 Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWD register.
//
// *****************************************************************************
pub const SYSCTL_DCGCWD_D1: usize = 0x00000002; // Watchdog Timer 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCWD_D0: usize = 0x00000001; // Watchdog Timer 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_DCGCTIMER_D5: usize = 0x00000020; // Timer 5 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCTIMER_D4: usize = 0x00000010; // Timer 4 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCTIMER_D3: usize = 0x00000008; // Timer 3 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCTIMER_D2: usize = 0x00000004; // Timer 2 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCTIMER_D1: usize = 0x00000002; // Timer 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCTIMER_D0: usize = 0x00000001; // Timer 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCGPIO
// register.
//
// *****************************************************************************
pub const SYSCTL_DCGCGPIO_D14: usize = 0x00004000; // GPIO Port Q Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D13: usize = 0x00002000; // GPIO Port P Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D12: usize = 0x00001000; // GPIO Port N Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D11: usize = 0x00000800; // GPIO Port M Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D10: usize = 0x00000400; // GPIO Port L Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D9: usize = 0x00000200; // GPIO Port K Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D8: usize = 0x00000100; // GPIO Port J Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D7: usize = 0x00000080; // GPIO Port H Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D6: usize = 0x00000040; // GPIO Port G Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D5: usize = 0x00000020; // GPIO Port F Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D4: usize = 0x00000010; // GPIO Port E Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D3: usize = 0x00000008; // GPIO Port D Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D2: usize = 0x00000004; // GPIO Port C Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D1: usize = 0x00000002; // GPIO Port B Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCGPIO_D0: usize = 0x00000001; // GPIO Port A Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCDMA register.
//
// *****************************************************************************
pub const SYSCTL_DCGCDMA_D0: usize = 0x00000001; // uDMA Module Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCHIB register.
//
// *****************************************************************************
pub const SYSCTL_DCGCHIB_D0: usize = 0x00000001; // Hibernation Module Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUART
// register.
//
// *****************************************************************************
pub const SYSCTL_DCGCUART_D7: usize = 0x00000080; // UART Module 7 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCUART_D6: usize = 0x00000040; // UART Module 6 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCUART_D5: usize = 0x00000020; // UART Module 5 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCUART_D4: usize = 0x00000010; // UART Module 4 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCUART_D3: usize = 0x00000008; // UART Module 3 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCUART_D2: usize = 0x00000004; // UART Module 2 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCUART_D1: usize = 0x00000002; // UART Module 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCUART_D0: usize = 0x00000001; // UART Module 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCSSI register.
//
// *****************************************************************************
pub const SYSCTL_DCGCSSI_D3: usize = 0x00000008; // SSI Module 3 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCSSI_D2: usize = 0x00000004; // SSI Module 2 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCSSI_D1: usize = 0x00000002; // SSI Module 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCSSI_D0: usize = 0x00000001; // SSI Module 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCI2C register.
//
// *****************************************************************************
pub const SYSCTL_DCGCI2C_D5: usize = 0x00000020; // I2C Module 5 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCI2C_D4: usize = 0x00000010; // I2C Module 4 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCI2C_D3: usize = 0x00000008; // I2C Module 3 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCI2C_D2: usize = 0x00000004; // I2C Module 2 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCI2C_D1: usize = 0x00000002; // I2C Module 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCI2C_D0: usize = 0x00000001; // I2C Module 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUSB register.
//
// *****************************************************************************
pub const SYSCTL_DCGCUSB_D0: usize = 0x00000001; // USB Module Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCCAN register.
//
// *****************************************************************************
pub const SYSCTL_DCGCCAN_D1: usize = 0x00000002; // CAN Module 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCCAN_D0: usize = 0x00000001; // CAN Module 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCADC register.
//
// *****************************************************************************
pub const SYSCTL_DCGCADC_D1: usize = 0x00000002; // ADC Module 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCADC_D0: usize = 0x00000001; // ADC Module 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCACMP
// register.
//
// *****************************************************************************
pub const SYSCTL_DCGCACMP_D0: usize = 0x00000001; // Analog Comparator Module 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEEPROM
// register.
//
// *****************************************************************************
pub const SYSCTL_DCGCEEPROM_D0: usize = 0x00000001; // EEPROM Module Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_DCGCWTIMER_D5: usize = 0x00000020; // Wide Timer 5 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCWTIMER_D4: usize = 0x00000010; // Wide Timer 4 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCWTIMER_D3: usize = 0x00000008; // Wide Timer 3 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCWTIMER_D2: usize = 0x00000004; // Wide Timer 2 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCWTIMER_D1: usize = 0x00000002; // Wide Timer 1 Deep-Sleep Mode Clock Gating Control
pub const SYSCTL_DCGCWTIMER_D0: usize = 0x00000001; // Wide Timer 0 Deep-Sleep Mode Clock Gating Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCWD register.
//
// *****************************************************************************
pub const SYSCTL_PCWD_P1: usize = 0x00000002; // Watchdog Timer 1 Power Control
pub const SYSCTL_PCWD_P0: usize = 0x00000001; // Watchdog Timer 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCTIMER register.
//
// *****************************************************************************
pub const SYSCTL_PCTIMER_P5: usize = 0x00000020; // Timer 5 Power Control
pub const SYSCTL_PCTIMER_P4: usize = 0x00000010; // Timer 4 Power Control
pub const SYSCTL_PCTIMER_P3: usize = 0x00000008; // Timer 3 Power Control
pub const SYSCTL_PCTIMER_P2: usize = 0x00000004; // Timer 2 Power Control
pub const SYSCTL_PCTIMER_P1: usize = 0x00000002; // Timer 1 Power Control
pub const SYSCTL_PCTIMER_P0: usize = 0x00000001; // Timer 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCGPIO register.
//
// *****************************************************************************
pub const SYSCTL_PCGPIO_P14: usize = 0x00004000; // GPIO Port Q Power Control
pub const SYSCTL_PCGPIO_P13: usize = 0x00002000; // GPIO Port P Power Control
pub const SYSCTL_PCGPIO_P12: usize = 0x00001000; // GPIO Port N Power Control
pub const SYSCTL_PCGPIO_P11: usize = 0x00000800; // GPIO Port M Power Control
pub const SYSCTL_PCGPIO_P10: usize = 0x00000400; // GPIO Port L Power Control
pub const SYSCTL_PCGPIO_P9: usize = 0x00000200; // GPIO Port K Power Control
pub const SYSCTL_PCGPIO_P8: usize = 0x00000100; // GPIO Port J Power Control
pub const SYSCTL_PCGPIO_P7: usize = 0x00000080; // GPIO Port H Power Control
pub const SYSCTL_PCGPIO_P6: usize = 0x00000040; // GPIO Port G Power Control
pub const SYSCTL_PCGPIO_P5: usize = 0x00000020; // GPIO Port F Power Control
pub const SYSCTL_PCGPIO_P4: usize = 0x00000010; // GPIO Port E Power Control
pub const SYSCTL_PCGPIO_P3: usize = 0x00000008; // GPIO Port D Power Control
pub const SYSCTL_PCGPIO_P2: usize = 0x00000004; // GPIO Port C Power Control
pub const SYSCTL_PCGPIO_P1: usize = 0x00000002; // GPIO Port B Power Control
pub const SYSCTL_PCGPIO_P0: usize = 0x00000001; // GPIO Port A Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCDMA register.
//
// *****************************************************************************
pub const SYSCTL_PCDMA_P0: usize = 0x00000001; // uDMA Module Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCHIB register.
//
// *****************************************************************************
pub const SYSCTL_PCHIB_P0: usize = 0x00000001; // Hibernation Module Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCUART register.
//
// *****************************************************************************
pub const SYSCTL_PCUART_P7: usize = 0x00000080; // UART Module 7 Power Control
pub const SYSCTL_PCUART_P6: usize = 0x00000040; // UART Module 6 Power Control
pub const SYSCTL_PCUART_P5: usize = 0x00000020; // UART Module 5 Power Control
pub const SYSCTL_PCUART_P4: usize = 0x00000010; // UART Module 4 Power Control
pub const SYSCTL_PCUART_P3: usize = 0x00000008; // UART Module 3 Power Control
pub const SYSCTL_PCUART_P2: usize = 0x00000004; // UART Module 2 Power Control
pub const SYSCTL_PCUART_P1: usize = 0x00000002; // UART Module 1 Power Control
pub const SYSCTL_PCUART_P0: usize = 0x00000001; // UART Module 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCSSI register.
//
// *****************************************************************************
pub const SYSCTL_PCSSI_P3: usize = 0x00000008; // SSI Module 3 Power Control
pub const SYSCTL_PCSSI_P2: usize = 0x00000004; // SSI Module 2 Power Control
pub const SYSCTL_PCSSI_P1: usize = 0x00000002; // SSI Module 1 Power Control
pub const SYSCTL_PCSSI_P0: usize = 0x00000001; // SSI Module 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCI2C register.
//
// *****************************************************************************
pub const SYSCTL_PCI2C_P5: usize = 0x00000020; // I2C Module 5 Power Control
pub const SYSCTL_PCI2C_P4: usize = 0x00000010; // I2C Module 4 Power Control
pub const SYSCTL_PCI2C_P3: usize = 0x00000008; // I2C Module 3 Power Control
pub const SYSCTL_PCI2C_P2: usize = 0x00000004; // I2C Module 2 Power Control
pub const SYSCTL_PCI2C_P1: usize = 0x00000002; // I2C Module 1 Power Control
pub const SYSCTL_PCI2C_P0: usize = 0x00000001; // I2C Module 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCUSB register.
//
// *****************************************************************************
pub const SYSCTL_PCUSB_P0: usize = 0x00000001; // USB Module Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCCAN register.
//
// *****************************************************************************
pub const SYSCTL_PCCAN_P1: usize = 0x00000002; // CAN Module 1 Power Control
pub const SYSCTL_PCCAN_P0: usize = 0x00000001; // CAN Module 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCADC register.
//
// *****************************************************************************
pub const SYSCTL_PCADC_P1: usize = 0x00000002; // ADC Module 1 Power Control
pub const SYSCTL_PCADC_P0: usize = 0x00000001; // ADC Module 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCACMP register.
//
// *****************************************************************************
pub const SYSCTL_PCACMP_P0: usize = 0x00000001; // Analog Comparator Module 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEEPROM
// register.
//
// *****************************************************************************
pub const SYSCTL_PCEEPROM_P0: usize = 0x00000001; // EEPROM Module Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCWTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_PCWTIMER_P5: usize = 0x00000020; // Wide Timer 5 Power Control
pub const SYSCTL_PCWTIMER_P4: usize = 0x00000010; // Wide Timer 4 Power Control
pub const SYSCTL_PCWTIMER_P3: usize = 0x00000008; // Wide Timer 3 Power Control
pub const SYSCTL_PCWTIMER_P2: usize = 0x00000004; // Wide Timer 2 Power Control
pub const SYSCTL_PCWTIMER_P1: usize = 0x00000002; // Wide Timer 1 Power Control
pub const SYSCTL_PCWTIMER_P0: usize = 0x00000001; // Wide Timer 0 Power Control

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWD register.
//
// *****************************************************************************
pub const SYSCTL_PRWD_R1: usize = 0x00000002; // Watchdog Timer 1 Peripheral Ready
pub const SYSCTL_PRWD_R0: usize = 0x00000001; // Watchdog Timer 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRTIMER register.
//
// *****************************************************************************
pub const SYSCTL_PRTIMER_R5: usize = 0x00000020; // Timer 5 Peripheral Ready
pub const SYSCTL_PRTIMER_R4: usize = 0x00000010; // Timer 4 Peripheral Ready
pub const SYSCTL_PRTIMER_R3: usize = 0x00000008; // Timer 3 Peripheral Ready
pub const SYSCTL_PRTIMER_R2: usize = 0x00000004; // Timer 2 Peripheral Ready
pub const SYSCTL_PRTIMER_R1: usize = 0x00000002; // Timer 1 Peripheral Ready
pub const SYSCTL_PRTIMER_R0: usize = 0x00000001; // Timer 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRGPIO register.
//
// *****************************************************************************
pub const SYSCTL_PRGPIO_R14: usize = 0x00004000; // GPIO Port Q Peripheral Ready
pub const SYSCTL_PRGPIO_R13: usize = 0x00002000; // GPIO Port P Peripheral Ready
pub const SYSCTL_PRGPIO_R12: usize = 0x00001000; // GPIO Port N Peripheral Ready
pub const SYSCTL_PRGPIO_R11: usize = 0x00000800; // GPIO Port M Peripheral Ready
pub const SYSCTL_PRGPIO_R10: usize = 0x00000400; // GPIO Port L Peripheral Ready
pub const SYSCTL_PRGPIO_R9: usize = 0x00000200; // GPIO Port K Peripheral Ready
pub const SYSCTL_PRGPIO_R8: usize = 0x00000100; // GPIO Port J Peripheral Ready
pub const SYSCTL_PRGPIO_R7: usize = 0x00000080; // GPIO Port H Peripheral Ready
pub const SYSCTL_PRGPIO_R6: usize = 0x00000040; // GPIO Port G Peripheral Ready
pub const SYSCTL_PRGPIO_R5: usize = 0x00000020; // GPIO Port F Peripheral Ready
pub const SYSCTL_PRGPIO_R4: usize = 0x00000010; // GPIO Port E Peripheral Ready
pub const SYSCTL_PRGPIO_R3: usize = 0x00000008; // GPIO Port D Peripheral Ready
pub const SYSCTL_PRGPIO_R2: usize = 0x00000004; // GPIO Port C Peripheral Ready
pub const SYSCTL_PRGPIO_R1: usize = 0x00000002; // GPIO Port B Peripheral Ready
pub const SYSCTL_PRGPIO_R0: usize = 0x00000001; // GPIO Port A Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRDMA register.
//
// *****************************************************************************
pub const SYSCTL_PRDMA_R0: usize = 0x00000001; // uDMA Module Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRHIB register.
//
// *****************************************************************************
pub const SYSCTL_PRHIB_R0: usize = 0x00000001; // Hibernation Module Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUART register.
//
// *****************************************************************************
pub const SYSCTL_PRUART_R7: usize = 0x00000080; // UART Module 7 Peripheral Ready
pub const SYSCTL_PRUART_R6: usize = 0x00000040; // UART Module 6 Peripheral Ready
pub const SYSCTL_PRUART_R5: usize = 0x00000020; // UART Module 5 Peripheral Ready
pub const SYSCTL_PRUART_R4: usize = 0x00000010; // UART Module 4 Peripheral Ready
pub const SYSCTL_PRUART_R3: usize = 0x00000008; // UART Module 3 Peripheral Ready
pub const SYSCTL_PRUART_R2: usize = 0x00000004; // UART Module 2 Peripheral Ready
pub const SYSCTL_PRUART_R1: usize = 0x00000002; // UART Module 1 Peripheral Ready
pub const SYSCTL_PRUART_R0: usize = 0x00000001; // UART Module 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRSSI register.
//
// *****************************************************************************
pub const SYSCTL_PRSSI_R3: usize = 0x00000008; // SSI Module 3 Peripheral Ready
pub const SYSCTL_PRSSI_R2: usize = 0x00000004; // SSI Module 2 Peripheral Ready
pub const SYSCTL_PRSSI_R1: usize = 0x00000002; // SSI Module 1 Peripheral Ready
pub const SYSCTL_PRSSI_R0: usize = 0x00000001; // SSI Module 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRI2C register.
//
// *****************************************************************************
pub const SYSCTL_PRI2C_R5: usize = 0x00000020; // I2C Module 5 Peripheral Ready
pub const SYSCTL_PRI2C_R4: usize = 0x00000010; // I2C Module 4 Peripheral Ready
pub const SYSCTL_PRI2C_R3: usize = 0x00000008; // I2C Module 3 Peripheral Ready
pub const SYSCTL_PRI2C_R2: usize = 0x00000004; // I2C Module 2 Peripheral Ready
pub const SYSCTL_PRI2C_R1: usize = 0x00000002; // I2C Module 1 Peripheral Ready
pub const SYSCTL_PRI2C_R0: usize = 0x00000001; // I2C Module 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUSB register.
//
// *****************************************************************************
pub const SYSCTL_PRUSB_R0: usize = 0x00000001; // USB Module Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRCAN register.
//
// *****************************************************************************
pub const SYSCTL_PRCAN_R1: usize = 0x00000002; // CAN Module 1 Peripheral Ready
pub const SYSCTL_PRCAN_R0: usize = 0x00000001; // CAN Module 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRADC register.
//
// *****************************************************************************
pub const SYSCTL_PRADC_R1: usize = 0x00000002; // ADC Module 1 Peripheral Ready
pub const SYSCTL_PRADC_R0: usize = 0x00000001; // ADC Module 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRACMP register.
//
// *****************************************************************************
pub const SYSCTL_PRACMP_R0: usize = 0x00000001; // Analog Comparator Module 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREEPROM
// register.
//
// *****************************************************************************
pub const SYSCTL_PREEPROM_R0: usize = 0x00000001; // EEPROM Module Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWTIMER
// register.
//
// *****************************************************************************
pub const SYSCTL_PRWTIMER_R5: usize = 0x00000020; // Wide Timer 5 Peripheral Ready
pub const SYSCTL_PRWTIMER_R4: usize = 0x00000010; // Wide Timer 4 Peripheral Ready
pub const SYSCTL_PRWTIMER_R3: usize = 0x00000008; // Wide Timer 3 Peripheral Ready
pub const SYSCTL_PRWTIMER_R2: usize = 0x00000004; // Wide Timer 2 Peripheral Ready
pub const SYSCTL_PRWTIMER_R1: usize = 0x00000002; // Wide Timer 1 Peripheral Ready
pub const SYSCTL_PRWTIMER_R0: usize = 0x00000001; // Wide Timer 0 Peripheral Ready

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_STAT register.
//
// *****************************************************************************
pub const UDMA_STAT_DMACHANS_M: usize = 0x001F0000; // Available uDMA Channels Minus 1
pub const UDMA_STAT_STATE_M: usize = 0x000000F0; // Control State Machine Status
pub const UDMA_STAT_STATE_IDLE: usize = 0x00000000; // Idle
pub const UDMA_STAT_STATE_RD_CTRL: usize = 0x00000010; // Reading channel controller data
pub const UDMA_STAT_STATE_RD_SRCENDP: usize = 0x00000020; // Reading source end pointer
pub const UDMA_STAT_STATE_RD_DSTENDP: usize = 0x00000030; // Reading destination end pointer
pub const UDMA_STAT_STATE_RD_SRCDAT: usize = 0x00000040; // Reading source data
pub const UDMA_STAT_STATE_WR_DSTDAT: usize = 0x00000050; // Writing destination data
pub const UDMA_STAT_STATE_WAIT: usize = 0x00000060; // Waiting for uDMA request to clear
pub const UDMA_STAT_STATE_WR_CTRL: usize = 0x00000070; // Writing channel controller data
pub const UDMA_STAT_STATE_STALL: usize = 0x00000080; // Stalled
pub const UDMA_STAT_STATE_DONE: usize = 0x00000090; // Done
pub const UDMA_STAT_STATE_UNDEF: usize = 0x000000A0; // Undefined
pub const UDMA_STAT_MASTEN: usize = 0x00000001; // Master Enable Status
pub const UDMA_STAT_DMACHANS_S: usize = 16;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CFG register.
//
// *****************************************************************************
pub const UDMA_CFG_MASTEN: usize = 0x00000001; // Controller Master Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CTLBASE register.
//
// *****************************************************************************
pub const UDMA_CTLBASE_ADDR_M: usize = 0xFFFFFC00; // Channel Control Base Address
pub const UDMA_CTLBASE_ADDR_S: usize = 10;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTBASE register.
//
// *****************************************************************************
pub const UDMA_ALTBASE_ADDR_M: usize = 0xFFFFFFFF; // Alternate Channel Address Pointer
pub const UDMA_ALTBASE_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_WAITSTAT register.
//
// *****************************************************************************
pub const UDMA_WAITSTAT_WAITREQ_M: usize = 0xFFFFFFFF; // Channel [n] Wait Status

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_SWREQ register.
//
// *****************************************************************************
pub const UDMA_SWREQ_M: usize = 0xFFFFFFFF; // Channel [n] Software Request

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTSET
// register.
//
// *****************************************************************************
pub const UDMA_USEBURSTSET_SET_M: usize = 0xFFFFFFFF; // Channel [n] Useburst Set

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_USEBURSTCLR
// register.
//
// *****************************************************************************
pub const UDMA_USEBURSTCLR_CLR_M: usize = 0xFFFFFFFF; // Channel [n] Useburst Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKSET
// register.
//
// *****************************************************************************
pub const UDMA_REQMASKSET_SET_M: usize = 0xFFFFFFFF; // Channel [n] Request Mask Set

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_REQMASKCLR
// register.
//
// *****************************************************************************
pub const UDMA_REQMASKCLR_CLR_M: usize = 0xFFFFFFFF; // Channel [n] Request Mask Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENASET register.
//
// *****************************************************************************
pub const UDMA_ENASET_SET_M: usize = 0xFFFFFFFF; // Channel [n] Enable Set

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ENACLR register.
//
// *****************************************************************************
pub const UDMA_ENACLR_CLR_M: usize = 0xFFFFFFFF; // Clear Channel [n] Enable Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTSET register.
//
// *****************************************************************************
pub const UDMA_ALTSET_SET_M: usize = 0xFFFFFFFF; // Channel [n] Alternate Set

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ALTCLR register.
//
// *****************************************************************************
pub const UDMA_ALTCLR_CLR_M: usize = 0xFFFFFFFF; // Channel [n] Alternate Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOSET register.
//
// *****************************************************************************
pub const UDMA_PRIOSET_SET_M: usize = 0xFFFFFFFF; // Channel [n] Priority Set

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_PRIOCLR register.
//
// *****************************************************************************
pub const UDMA_PRIOCLR_CLR_M: usize = 0xFFFFFFFF; // Channel [n] Priority Clear

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_ERRCLR register.
//
// *****************************************************************************
pub const UDMA_ERRCLR_ERRCLR: usize = 0x00000001; // uDMA Bus Error Status

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHASGN register.
//
// *****************************************************************************
pub const UDMA_CHASGN_M: usize = 0xFFFFFFFF; // Channel [n] Assignment Select
pub const UDMA_CHASGN_PRIMARY: usize = 0x00000000; // Use the primary channel assignment
pub const UDMA_CHASGN_SECONDARY: usize = 0x00000001; // Use the secondary channel assignment

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHIS register.
//
// *****************************************************************************
pub const UDMA_CHIS_M: usize = 0xFFFFFFFF; // Channel [n] Interrupt Status

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP0 register.
//
// *****************************************************************************
pub const UDMA_CHMAP0_CH7SEL_M: usize = 0xF0000000; // uDMA Channel 7 Source Select
pub const UDMA_CHMAP0_CH6SEL_M: usize = 0x0F000000; // uDMA Channel 6 Source Select
pub const UDMA_CHMAP0_CH5SEL_M: usize = 0x00F00000; // uDMA Channel 5 Source Select
pub const UDMA_CHMAP0_CH4SEL_M: usize = 0x000F0000; // uDMA Channel 4 Source Select
pub const UDMA_CHMAP0_CH3SEL_M: usize = 0x0000F000; // uDMA Channel 3 Source Select
pub const UDMA_CHMAP0_CH2SEL_M: usize = 0x00000F00; // uDMA Channel 2 Source Select
pub const UDMA_CHMAP0_CH1SEL_M: usize = 0x000000F0; // uDMA Channel 1 Source Select
pub const UDMA_CHMAP0_CH0SEL_M: usize = 0x0000000F; // uDMA Channel 0 Source Select
pub const UDMA_CHMAP0_CH7SEL_S: usize = 28;
pub const UDMA_CHMAP0_CH6SEL_S: usize = 24;
pub const UDMA_CHMAP0_CH5SEL_S: usize = 20;
pub const UDMA_CHMAP0_CH4SEL_S: usize = 16;
pub const UDMA_CHMAP0_CH3SEL_S: usize = 12;
pub const UDMA_CHMAP0_CH2SEL_S: usize = 8;
pub const UDMA_CHMAP0_CH1SEL_S: usize = 4;
pub const UDMA_CHMAP0_CH0SEL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP1 register.
//
// *****************************************************************************
pub const UDMA_CHMAP1_CH15SEL_M: usize = 0xF0000000; // uDMA Channel 15 Source Select
pub const UDMA_CHMAP1_CH14SEL_M: usize = 0x0F000000; // uDMA Channel 14 Source Select
pub const UDMA_CHMAP1_CH13SEL_M: usize = 0x00F00000; // uDMA Channel 13 Source Select
pub const UDMA_CHMAP1_CH12SEL_M: usize = 0x000F0000; // uDMA Channel 12 Source Select
pub const UDMA_CHMAP1_CH11SEL_M: usize = 0x0000F000; // uDMA Channel 11 Source Select
pub const UDMA_CHMAP1_CH10SEL_M: usize = 0x00000F00; // uDMA Channel 10 Source Select
pub const UDMA_CHMAP1_CH9SEL_M: usize = 0x000000F0; // uDMA Channel 9 Source Select
pub const UDMA_CHMAP1_CH8SEL_M: usize = 0x0000000F; // uDMA Channel 8 Source Select
pub const UDMA_CHMAP1_CH15SEL_S: usize = 28;
pub const UDMA_CHMAP1_CH14SEL_S: usize = 24;
pub const UDMA_CHMAP1_CH13SEL_S: usize = 20;
pub const UDMA_CHMAP1_CH12SEL_S: usize = 16;
pub const UDMA_CHMAP1_CH11SEL_S: usize = 12;
pub const UDMA_CHMAP1_CH10SEL_S: usize = 8;
pub const UDMA_CHMAP1_CH9SEL_S: usize = 4;
pub const UDMA_CHMAP1_CH8SEL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP2 register.
//
// *****************************************************************************
pub const UDMA_CHMAP2_CH23SEL_M: usize = 0xF0000000; // uDMA Channel 23 Source Select
pub const UDMA_CHMAP2_CH22SEL_M: usize = 0x0F000000; // uDMA Channel 22 Source Select
pub const UDMA_CHMAP2_CH21SEL_M: usize = 0x00F00000; // uDMA Channel 21 Source Select
pub const UDMA_CHMAP2_CH20SEL_M: usize = 0x000F0000; // uDMA Channel 20 Source Select
pub const UDMA_CHMAP2_CH19SEL_M: usize = 0x0000F000; // uDMA Channel 19 Source Select
pub const UDMA_CHMAP2_CH18SEL_M: usize = 0x00000F00; // uDMA Channel 18 Source Select
pub const UDMA_CHMAP2_CH17SEL_M: usize = 0x000000F0; // uDMA Channel 17 Source Select
pub const UDMA_CHMAP2_CH16SEL_M: usize = 0x0000000F; // uDMA Channel 16 Source Select
pub const UDMA_CHMAP2_CH23SEL_S: usize = 28;
pub const UDMA_CHMAP2_CH22SEL_S: usize = 24;
pub const UDMA_CHMAP2_CH21SEL_S: usize = 20;
pub const UDMA_CHMAP2_CH20SEL_S: usize = 16;
pub const UDMA_CHMAP2_CH19SEL_S: usize = 12;
pub const UDMA_CHMAP2_CH18SEL_S: usize = 8;
pub const UDMA_CHMAP2_CH17SEL_S: usize = 4;
pub const UDMA_CHMAP2_CH16SEL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_CHMAP3 register.
//
// *****************************************************************************
pub const UDMA_CHMAP3_CH31SEL_M: usize = 0xF0000000; // uDMA Channel 31 Source Select
pub const UDMA_CHMAP3_CH30SEL_M: usize = 0x0F000000; // uDMA Channel 30 Source Select
pub const UDMA_CHMAP3_CH29SEL_M: usize = 0x00F00000; // uDMA Channel 29 Source Select
pub const UDMA_CHMAP3_CH28SEL_M: usize = 0x000F0000; // uDMA Channel 28 Source Select
pub const UDMA_CHMAP3_CH27SEL_M: usize = 0x0000F000; // uDMA Channel 27 Source Select
pub const UDMA_CHMAP3_CH26SEL_M: usize = 0x00000F00; // uDMA Channel 26 Source Select
pub const UDMA_CHMAP3_CH25SEL_M: usize = 0x000000F0; // uDMA Channel 25 Source Select
pub const UDMA_CHMAP3_CH24SEL_M: usize = 0x0000000F; // uDMA Channel 24 Source Select
pub const UDMA_CHMAP3_CH31SEL_S: usize = 28;
pub const UDMA_CHMAP3_CH30SEL_S: usize = 24;
pub const UDMA_CHMAP3_CH29SEL_S: usize = 20;
pub const UDMA_CHMAP3_CH28SEL_S: usize = 16;
pub const UDMA_CHMAP3_CH27SEL_S: usize = 12;
pub const UDMA_CHMAP3_CH26SEL_S: usize = 8;
pub const UDMA_CHMAP3_CH25SEL_S: usize = 4;
pub const UDMA_CHMAP3_CH24SEL_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_SRCENDP register.
//
// *****************************************************************************
pub const UDMA_SRCENDP_ADDR_M: usize = 0xFFFFFFFF; // Source Address End Pointer
pub const UDMA_SRCENDP_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_DSTENDP register.
//
// *****************************************************************************
pub const UDMA_DSTENDP_ADDR_M: usize = 0xFFFFFFFF; // Destination Address End Pointer
pub const UDMA_DSTENDP_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the UDMA_O_CHCTL register.
//
// *****************************************************************************
pub const UDMA_CHCTL_DSTINC_M: usize = 0xC0000000; // Destination Address Increment
pub const UDMA_CHCTL_DSTINC_8: usize = 0x00000000; // Byte
pub const UDMA_CHCTL_DSTINC_16: usize = 0x40000000; // Half-word
pub const UDMA_CHCTL_DSTINC_32: usize = 0x80000000; // Word
pub const UDMA_CHCTL_DSTINC_NONE: usize = 0xC0000000; // No increment
pub const UDMA_CHCTL_DSTSIZE_M: usize = 0x30000000; // Destination Data Size
pub const UDMA_CHCTL_DSTSIZE_8: usize = 0x00000000; // Byte
pub const UDMA_CHCTL_DSTSIZE_16: usize = 0x10000000; // Half-word
pub const UDMA_CHCTL_DSTSIZE_32: usize = 0x20000000; // Word
pub const UDMA_CHCTL_SRCINC_M: usize = 0x0C000000; // Source Address Increment
pub const UDMA_CHCTL_SRCINC_8: usize = 0x00000000; // Byte
pub const UDMA_CHCTL_SRCINC_16: usize = 0x04000000; // Half-word
pub const UDMA_CHCTL_SRCINC_32: usize = 0x08000000; // Word
pub const UDMA_CHCTL_SRCINC_NONE: usize = 0x0C000000; // No increment
pub const UDMA_CHCTL_SRCSIZE_M: usize = 0x03000000; // Source Data Size
pub const UDMA_CHCTL_SRCSIZE_8: usize = 0x00000000; // Byte
pub const UDMA_CHCTL_SRCSIZE_16: usize = 0x01000000; // Half-word
pub const UDMA_CHCTL_SRCSIZE_32: usize = 0x02000000; // Word
pub const UDMA_CHCTL_ARBSIZE_M: usize = 0x0003C000; // Arbitration Size
pub const UDMA_CHCTL_ARBSIZE_1: usize = 0x00000000; // 1 Transfer
pub const UDMA_CHCTL_ARBSIZE_2: usize = 0x00004000; // 2 Transfers
pub const UDMA_CHCTL_ARBSIZE_4: usize = 0x00008000; // 4 Transfers
pub const UDMA_CHCTL_ARBSIZE_8: usize = 0x0000C000; // 8 Transfers
pub const UDMA_CHCTL_ARBSIZE_16: usize = 0x00010000; // 16 Transfers
pub const UDMA_CHCTL_ARBSIZE_32: usize = 0x00014000; // 32 Transfers
pub const UDMA_CHCTL_ARBSIZE_64: usize = 0x00018000; // 64 Transfers
pub const UDMA_CHCTL_ARBSIZE_128: usize = 0x0001C000; // 128 Transfers
pub const UDMA_CHCTL_ARBSIZE_256: usize = 0x00020000; // 256 Transfers
pub const UDMA_CHCTL_ARBSIZE_512: usize = 0x00024000; // 512 Transfers
pub const UDMA_CHCTL_ARBSIZE_1024: usize = 0x00028000; // 1024 Transfers
pub const UDMA_CHCTL_XFERSIZE_M: usize = 0x00003FF0; // Transfer Size (minus 1)
pub const UDMA_CHCTL_NXTUSEBURST: usize = 0x00000008; // Next Useburst
pub const UDMA_CHCTL_XFERMODE_M: usize = 0x00000007; // uDMA Transfer Mode
pub const UDMA_CHCTL_XFERMODE_STOP: usize = 0x00000000; // Stop
pub const UDMA_CHCTL_XFERMODE_BASIC: usize = 0x00000001; // Basic
pub const UDMA_CHCTL_XFERMODE_AUTO: usize = 0x00000002; // Auto-Request
pub const UDMA_CHCTL_XFERMODE_PINGPONG: usize = 0x00000003; // Ping-Pong
pub const UDMA_CHCTL_XFERMODE_MEM_SG: usize = 0x00000004; // Memory Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_MEM_SGA: usize = 0x00000005; // Alternate Memory Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_PER_SG: usize = 0x00000006; // Peripheral Scatter-Gather
pub const UDMA_CHCTL_XFERMODE_PER_SGA: usize = 0x00000007; // Alternate Peripheral Scatter-Gather
pub const UDMA_CHCTL_XFERSIZE_S: usize = 4;

// *****************************************************************************
//
// The following are defines for the a GPIO register. See GPIO_PORTx_DATA_BITS_R.
//
// *****************************************************************************
#[repr(C, packed)]
pub struct GpioRegisters {
    pub data_mask: [RW<usize>; 255], // Data - offset sets pin mask
    pub data: RW<usize>, // Data register - sets all pins
    pub dir: RW<usize>, // Direction
    pub is: RW<usize>, // Interrupt Sense
    pub ibe: RW<usize>, // Interrupt Both Edges
    pub iev: RW<usize>, // Interrupt Event
    pub im: RW<usize>, // Interrupt Mask
    pub ris: RO<usize>, // Raw Interrupt Status
    pub mis: RW<usize>, // Masked Interrupt Status
    pub icr: RW<usize>, // Interrupt Clear
    pub afsel: RW<usize>, // Alternate function Select
    pub _padding: [usize; 55],
    pub dr2r: RW<usize>, // 2mA drive select
    pub dr4r: RW<usize>, // 4mA drive select
    pub dr8r: RW<usize>, // 8mA drive select
    pub odr: RW<usize>, // Open-drain select
    pub pur: RW<usize>, // Pull-up select
    pub pdr: RW<usize>, // Pull-down select
    pub slr: RW<usize>, // Slew-rate control
    pub den: RW<usize>, // Digital enable
    pub lock: RW<usize>, // Lock
    pub cr: RW<usize>, // Commit
    pub amsel: RW<usize>, // Analog mode select
    pub pctl: RW<usize>, // Port Control
    pub adcctl: RW<usize>, // ADC Control
    pub dmactl: RW<usize>, // DMA Control
    pub _padding2: [usize; 678],
    pub periphid4: RW<usize>, // Peripheral ID 4
    pub periphid5: RW<usize>, // Peripheral ID 5
    pub periphid6: RW<usize>, // Peripheral ID 6
    pub periphid7: RW<usize>, // Peripheral ID 7
    pub periphid0: RW<usize>, // Peripheral ID 0
    pub periphid1: RW<usize>, // Peripheral ID 1
    pub periphid2: RW<usize>, // Peripheral ID 2
    pub periphid3: RW<usize>, // Peripheral ID 3
    pub pcelld0: RW<usize>, // PrimeCell ID 0
    pub pcelld1: RW<usize>, // PrimeCell ID 1
    pub pcelld2: RW<usize>, // PrimeCell ID 2
    pub pcelld3: RW<usize>, // PrimeCell ID 3
}

#[repr(C, packed)]
pub struct UartRegisters {
    pub data: RW<usize>, // Data Register
    pub rsr: RW<usize>, // Receive Status
    pub _padding: [usize; 4],
    pub rf: RW<usize>, // Flags
    pub _padding2: usize,
    pub ilpr: RW<usize>, // IrDA Low-Power Register
    pub ibrd: RW<usize>, // Integer Baud-Rate Divisor
    pub fbrd: RW<usize>, // Fractional Baud-Rate Divisor
    pub lcrh: RW<usize>, // Line Control
    pub ctl: RW<usize>, // Control
    pub ifls: RW<usize>, // Interrupt FIFO Level Select
    pub im: RW<usize>, // Interrupt Mask
    pub ris: RW<usize>, // UARTx_RIS_R
    pub mis: RW<usize>, // UARTx_MIS_R
    pub icr: RW<usize>, // UARTx_ICR_R
    pub dmactl: RW<usize>, // UARTx_DMACTL_R
}

#[repr(C, packed)]
pub struct TimerRegisters {
    // 0x000 GPTMCFG R/W 0x0000.0000 GPTM Configuration 688
    pub cfg: RW<usize>,
    // 0x004 GPTMTAMR R/W 0x0000.0000 GPTM Timer A Mode 690
    pub tamr: RW<usize>,
    // 0x008 GPTMTBMR R/W 0x0000.0000 GPTM Timer B Mode 694
    pub tbmr: RW<usize>,
    // 0x00C GPTMCTL R/W 0x0000.0000 GPTM Control 698
    pub ctl: RW<usize>,
    // 0x010 GPTMSYNC R/W 0x0000.0000 GPTM Synchronize 702
    pub sync: RW<usize>,
    // 0x014
    pub _pad: usize,
    // 0x018 GPTMIMR R/W 0x0000.0000 GPTM Interrupt Mask 706
    pub imr: RW<usize>,
    // 0x01C GPTMRIS RO 0x0000.0000 GPTM Raw Interrupt Status 709
    pub ris: RW<usize>,
    // 0x020 GPTMMIS RO 0x0000.0000 GPTM Masked Interrupt Status 712
    pub mis: RW<usize>,
    // 0x024 GPTMICR W1C 0x0000.0000 GPTM Interrupt Clear 715
    pub icr: RW<usize>,
    // 0x028 GPTMTAILR R/W 0xFFFF.FFFF GPTM Timer A Interval Load 717
    pub tailr: RW<usize>,
    // 0x02C GPTMTBILR R/W - GPTM Timer B Interval Load 718
    pub tbilr: RW<usize>,
    // 0x030 GPTMTAMATCHR R/W 0xFFFF.FFFF GPTM Timer A Match 719
    pub tamatchr: RW<usize>,
    // 0x034 GPTMTBMATCHR R/W - GPTM Timer B Match 720
    pub tbmatchr: RW<usize>,
    // 0x038 GPTMTAPR R/W 0x0000.0000 GPTM Timer A Prescale 721
    pub tapr: RW<usize>,
    // 0x03C GPTMTBPR R/W 0x0000.0000 GPTM Timer B Prescale 722
    pub tbpr: RW<usize>,
    // 0x040 GPTMTAPMR R/W 0x0000.0000 GPTM TimerA Prescale Match 723
    pub tapmr: RW<usize>,
    // 0x044 GPTMTBPMR R/W 0x0000.0000 GPTM TimerB Prescale Match 724
    pub tbpmr: RW<usize>,
    // 0x048 GPTMTAR RO 0xFFFF.FFFF GPTM Timer A 725
    pub tar: RW<usize>,
    // 0x04C GPTMTBR RO - GPTM Timer B 726
    pub tbr: RW<usize>,
    // 0x050 GPTMTAV RW 0xFFFF.FFFF GPTM Timer A Value 727
    pub tav: RW<usize>,
    // 0x054 GPTMTBV RW - GPTM Timer B Value 728
    pub tbv: RW<usize>,
    // 0x058 GPTMRTCPD RO 0x0000.7FFF GPTM RTC Predivide 729
    pub rtcpd: RW<usize>,
    // 0x05C GPTMTAPS RO 0x0000.0000 GPTM Timer A Prescale Snapshot 730
    pub taps: RW<usize>,
    // 0x060 GPTMTBPS RO 0x0000.0000 GPTM Timer B Prescale Snapshot 731
    pub tbps: RW<usize>,
    // 0x064 GPTMTAPV RO 0x0000.0000 GPTM Timer A Prescale Value 732
    pub tapv: RW<usize>,
    // 0x068 GPTMTBPV RO 0x0000.0000 GPTM Timer B Prescale Value 733
    pub tbpv: RW<usize>,
}

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
