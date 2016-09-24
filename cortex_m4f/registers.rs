// *****************************************************************************
//
// Cortex-M4F Register Definitions
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

// *****************************************************************************
//
// NVIC registers (NVIC)
//
// *****************************************************************************

pub const NVIC_INT_TYPE_R: *mut usize = 0xE000E004 as *mut usize;
pub const NVIC_ACTLR_R: *mut usize = 0xE000E008 as *mut usize;
pub const NVIC_ST_CTRL_R: *mut usize = 0xE000E010 as *mut usize;
pub const NVIC_ST_RELOAD_R: *mut usize = 0xE000E014 as *mut usize;
pub const NVIC_ST_CURRENT_R: *mut usize = 0xE000E018 as *mut usize;
pub const NVIC_ST_CAL_R: *mut usize = 0xE000E01C as *mut usize;
pub const NVIC_EN0_R: *mut usize = 0xE000E100 as *mut usize;
pub const NVIC_EN1_R: *mut usize = 0xE000E104 as *mut usize;
pub const NVIC_EN2_R: *mut usize = 0xE000E108 as *mut usize;
pub const NVIC_EN3_R: *mut usize = 0xE000E10C as *mut usize;
pub const NVIC_EN4_R: *mut usize = 0xE000E110 as *mut usize;
pub const NVIC_DIS0_R: *mut usize = 0xE000E180 as *mut usize;
pub const NVIC_DIS1_R: *mut usize = 0xE000E184 as *mut usize;
pub const NVIC_DIS2_R: *mut usize = 0xE000E188 as *mut usize;
pub const NVIC_DIS3_R: *mut usize = 0xE000E18C as *mut usize;
pub const NVIC_DIS4_R: *mut usize = 0xE000E190 as *mut usize;
pub const NVIC_PEND0_R: *mut usize = 0xE000E200 as *mut usize;
pub const NVIC_PEND1_R: *mut usize = 0xE000E204 as *mut usize;
pub const NVIC_PEND2_R: *mut usize = 0xE000E208 as *mut usize;
pub const NVIC_PEND3_R: *mut usize = 0xE000E20C as *mut usize;
pub const NVIC_PEND4_R: *mut usize = 0xE000E210 as *mut usize;
pub const NVIC_UNPEND0_R: *mut usize = 0xE000E280 as *mut usize;
pub const NVIC_UNPEND1_R: *mut usize = 0xE000E284 as *mut usize;
pub const NVIC_UNPEND2_R: *mut usize = 0xE000E288 as *mut usize;
pub const NVIC_UNPEND3_R: *mut usize = 0xE000E28C as *mut usize;
pub const NVIC_UNPEND4_R: *mut usize = 0xE000E290 as *mut usize;
pub const NVIC_ACTIVE0_R: *mut usize = 0xE000E300 as *mut usize;
pub const NVIC_ACTIVE1_R: *mut usize = 0xE000E304 as *mut usize;
pub const NVIC_ACTIVE2_R: *mut usize = 0xE000E308 as *mut usize;
pub const NVIC_ACTIVE3_R: *mut usize = 0xE000E30C as *mut usize;
pub const NVIC_ACTIVE4_R: *mut usize = 0xE000E310 as *mut usize;
pub const NVIC_PRI0_R: *mut usize = 0xE000E400 as *mut usize;
pub const NVIC_PRI1_R: *mut usize = 0xE000E404 as *mut usize;
pub const NVIC_PRI2_R: *mut usize = 0xE000E408 as *mut usize;
pub const NVIC_PRI3_R: *mut usize = 0xE000E40C as *mut usize;
pub const NVIC_PRI4_R: *mut usize = 0xE000E410 as *mut usize;
pub const NVIC_PRI5_R: *mut usize = 0xE000E414 as *mut usize;
pub const NVIC_PRI6_R: *mut usize = 0xE000E418 as *mut usize;
pub const NVIC_PRI7_R: *mut usize = 0xE000E41C as *mut usize;
pub const NVIC_PRI8_R: *mut usize = 0xE000E420 as *mut usize;
pub const NVIC_PRI9_R: *mut usize = 0xE000E424 as *mut usize;
pub const NVIC_PRI10_R: *mut usize = 0xE000E428 as *mut usize;
pub const NVIC_PRI11_R: *mut usize = 0xE000E42C as *mut usize;
pub const NVIC_PRI12_R: *mut usize = 0xE000E430 as *mut usize;
pub const NVIC_PRI13_R: *mut usize = 0xE000E434 as *mut usize;
pub const NVIC_PRI14_R: *mut usize = 0xE000E438 as *mut usize;
pub const NVIC_PRI15_R: *mut usize = 0xE000E43C as *mut usize;
pub const NVIC_PRI16_R: *mut usize = 0xE000E440 as *mut usize;
pub const NVIC_PRI17_R: *mut usize = 0xE000E444 as *mut usize;
pub const NVIC_PRI18_R: *mut usize = 0xE000E448 as *mut usize;
pub const NVIC_PRI19_R: *mut usize = 0xE000E44C as *mut usize;
pub const NVIC_PRI20_R: *mut usize = 0xE000E450 as *mut usize;
pub const NVIC_PRI21_R: *mut usize = 0xE000E454 as *mut usize;
pub const NVIC_PRI22_R: *mut usize = 0xE000E458 as *mut usize;
pub const NVIC_PRI23_R: *mut usize = 0xE000E45C as *mut usize;
pub const NVIC_PRI24_R: *mut usize = 0xE000E460 as *mut usize;
pub const NVIC_PRI25_R: *mut usize = 0xE000E464 as *mut usize;
pub const NVIC_PRI26_R: *mut usize = 0xE000E468 as *mut usize;
pub const NVIC_PRI27_R: *mut usize = 0xE000E46C as *mut usize;
pub const NVIC_PRI28_R: *mut usize = 0xE000E470 as *mut usize;
pub const NVIC_PRI29_R: *mut usize = 0xE000E474 as *mut usize;
pub const NVIC_PRI30_R: *mut usize = 0xE000E478 as *mut usize;
pub const NVIC_PRI31_R: *mut usize = 0xE000E47C as *mut usize;
pub const NVIC_PRI32_R: *mut usize = 0xE000E480 as *mut usize;
pub const NVIC_PRI33_R: *mut usize = 0xE000E484 as *mut usize;
pub const NVIC_PRI34_R: *mut usize = 0xE000E488 as *mut usize;
pub const NVIC_CPUID_R: *mut usize = 0xE000ED00 as *mut usize;
pub const NVIC_INT_CTRL_R: *mut usize = 0xE000ED04 as *mut usize;
pub const NVIC_VTABLE_R: *mut usize = 0xE000ED08 as *mut usize;
pub const NVIC_APINT_R: *mut usize = 0xE000ED0C as *mut usize;
pub const NVIC_SYS_CTRL_R: *mut usize = 0xE000ED10 as *mut usize;
pub const NVIC_CFG_CTRL_R: *mut usize = 0xE000ED14 as *mut usize;
pub const NVIC_SYS_PRI1_R: *mut usize = 0xE000ED18 as *mut usize;
pub const NVIC_SYS_PRI2_R: *mut usize = 0xE000ED1C as *mut usize;
pub const NVIC_SYS_PRI3_R: *mut usize = 0xE000ED20 as *mut usize;
pub const NVIC_SYS_HND_CTRL_R: *mut usize = 0xE000ED24 as *mut usize;
pub const NVIC_FAULT_STAT_R: *mut usize = 0xE000ED28 as *mut usize;
pub const NVIC_HFAULT_STAT_R: *mut usize = 0xE000ED2C as *mut usize;
pub const NVIC_DEBUG_STAT_R: *mut usize = 0xE000ED30 as *mut usize;
pub const NVIC_MM_ADDR_R: *mut usize = 0xE000ED34 as *mut usize;
pub const NVIC_FAULT_ADDR_R: *mut usize = 0xE000ED38 as *mut usize;
pub const NVIC_CPAC_R: *mut usize = 0xE000ED88 as *mut usize;
pub const NVIC_MPU_TYPE_R: *mut usize = 0xE000ED90 as *mut usize;
pub const NVIC_MPU_CTRL_R: *mut usize = 0xE000ED94 as *mut usize;
pub const NVIC_MPU_NUMBER_R: *mut usize = 0xE000ED98 as *mut usize;
pub const NVIC_MPU_BASE_R: *mut usize = 0xE000ED9C as *mut usize;
pub const NVIC_MPU_ATTR_R: *mut usize = 0xE000EDA0 as *mut usize;
pub const NVIC_MPU_BASE1_R: *mut usize = 0xE000EDA4 as *mut usize;
pub const NVIC_MPU_ATTR1_R: *mut usize = 0xE000EDA8 as *mut usize;
pub const NVIC_MPU_BASE2_R: *mut usize = 0xE000EDAC as *mut usize;
pub const NVIC_MPU_ATTR2_R: *mut usize = 0xE000EDB0 as *mut usize;
pub const NVIC_MPU_BASE3_R: *mut usize = 0xE000EDB4 as *mut usize;
pub const NVIC_MPU_ATTR3_R: *mut usize = 0xE000EDB8 as *mut usize;
pub const NVIC_DBG_CTRL_R: *mut usize = 0xE000EDF0 as *mut usize;
pub const NVIC_DBG_XFER_R: *mut usize = 0xE000EDF4 as *mut usize;
pub const NVIC_DBG_DATA_R: *mut usize = 0xE000EDF8 as *mut usize;
pub const NVIC_DBG_INT_R: *mut usize = 0xE000EDFC as *mut usize;
pub const NVIC_SW_TRIG_R: *mut usize = 0xE000EF00 as *mut usize;
pub const NVIC_FPCC_R: *mut usize = 0xE000EF34 as *mut usize;
pub const NVIC_FPCA_R: *mut usize = 0xE000EF38 as *mut usize;
pub const NVIC_FPDSC_R: *mut usize = 0xE000EF3C as *mut usize;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_INT_TYPE register.
//
// *****************************************************************************
pub const NVIC_INT_TYPE_LINES_M: usize = 0x0000001F; // Number of interrupt lines (x32)
pub const NVIC_INT_TYPE_LINES_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTLR register.
//
// *****************************************************************************
pub const NVIC_ACTLR_DISOOFP: usize = 0x00000200; // Disable Out-Of-Order Floating Point
pub const NVIC_ACTLR_DISFPCA: usize = 0x00000100; // Disable CONTROL
pub const NVIC_ACTLR_DISFOLD: usize = 0x00000004; // Disable IT Folding
pub const NVIC_ACTLR_DISWBUF: usize = 0x00000002; // Disable Write Buffer
pub const NVIC_ACTLR_DISMCYC: usize = 0x00000001; // Disable Interrupts of Multiple Cycle Instructions

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CTRL register.
//
// *****************************************************************************
pub const NVIC_ST_CTRL_COUNT: usize = 0x00010000; // Count Flag
pub const NVIC_ST_CTRL_CLK_SRC: usize = 0x00000004; // Clock Source
pub const NVIC_ST_CTRL_INTEN: usize = 0x00000002; // Interrupt Enable
pub const NVIC_ST_CTRL_ENABLE: usize = 0x00000001; // Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_RELOAD register.
//
// *****************************************************************************
pub const NVIC_ST_RELOAD_M: usize = 0x00FFFFFF; // Reload Value
pub const NVIC_ST_RELOAD_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CURRENT
// register.
//
// *****************************************************************************
pub const NVIC_ST_CURRENT_M: usize = 0x00FFFFFF; // Current Value
pub const NVIC_ST_CURRENT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ST_CAL register.
//
// *****************************************************************************
pub const NVIC_ST_CAL_NOREF: usize = 0x80000000; // No reference clock
pub const NVIC_ST_CAL_SKEW: usize = 0x40000000; // Clock skew
pub const NVIC_ST_CAL_ONEMS_M: usize = 0x00FFFFFF; // 1ms reference value
pub const NVIC_ST_CAL_ONEMS_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN0 register.
//
// *****************************************************************************
pub const NVIC_EN0_INT_M: usize = 0xFFFFFFFF; // Interrupt Enable
pub const NVIC_EN0_INT0: usize = 0x00000001; // Interrupt 0 enable
pub const NVIC_EN0_INT1: usize = 0x00000002; // Interrupt 1 enable
pub const NVIC_EN0_INT2: usize = 0x00000004; // Interrupt 2 enable
pub const NVIC_EN0_INT3: usize = 0x00000008; // Interrupt 3 enable
pub const NVIC_EN0_INT4: usize = 0x00000010; // Interrupt 4 enable
pub const NVIC_EN0_INT5: usize = 0x00000020; // Interrupt 5 enable
pub const NVIC_EN0_INT6: usize = 0x00000040; // Interrupt 6 enable
pub const NVIC_EN0_INT7: usize = 0x00000080; // Interrupt 7 enable
pub const NVIC_EN0_INT8: usize = 0x00000100; // Interrupt 8 enable
pub const NVIC_EN0_INT9: usize = 0x00000200; // Interrupt 9 enable
pub const NVIC_EN0_INT10: usize = 0x00000400; // Interrupt 10 enable
pub const NVIC_EN0_INT11: usize = 0x00000800; // Interrupt 11 enable
pub const NVIC_EN0_INT12: usize = 0x00001000; // Interrupt 12 enable
pub const NVIC_EN0_INT13: usize = 0x00002000; // Interrupt 13 enable
pub const NVIC_EN0_INT14: usize = 0x00004000; // Interrupt 14 enable
pub const NVIC_EN0_INT15: usize = 0x00008000; // Interrupt 15 enable
pub const NVIC_EN0_INT16: usize = 0x00010000; // Interrupt 16 enable
pub const NVIC_EN0_INT17: usize = 0x00020000; // Interrupt 17 enable
pub const NVIC_EN0_INT18: usize = 0x00040000; // Interrupt 18 enable
pub const NVIC_EN0_INT19: usize = 0x00080000; // Interrupt 19 enable
pub const NVIC_EN0_INT20: usize = 0x00100000; // Interrupt 20 enable
pub const NVIC_EN0_INT21: usize = 0x00200000; // Interrupt 21 enable
pub const NVIC_EN0_INT22: usize = 0x00400000; // Interrupt 22 enable
pub const NVIC_EN0_INT23: usize = 0x00800000; // Interrupt 23 enable
pub const NVIC_EN0_INT24: usize = 0x01000000; // Interrupt 24 enable
pub const NVIC_EN0_INT25: usize = 0x02000000; // Interrupt 25 enable
pub const NVIC_EN0_INT26: usize = 0x04000000; // Interrupt 26 enable
pub const NVIC_EN0_INT27: usize = 0x08000000; // Interrupt 27 enable
pub const NVIC_EN0_INT28: usize = 0x10000000; // Interrupt 28 enable
pub const NVIC_EN0_INT29: usize = 0x20000000; // Interrupt 29 enable
pub const NVIC_EN0_INT30: usize = 0x40000000; // Interrupt 30 enable
pub const NVIC_EN0_INT31: usize = 0x80000000; // Interrupt 31 enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN1 register.
//
// *****************************************************************************
pub const NVIC_EN1_INT_M: usize = 0xFFFFFFFF; // Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN2 register.
//
// *****************************************************************************
pub const NVIC_EN2_INT_M: usize = 0xFFFFFFFF; // Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN3 register.
//
// *****************************************************************************
pub const NVIC_EN3_INT_M: usize = 0xFFFFFFFF; // Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_EN4 register.
//
// *****************************************************************************
pub const NVIC_EN4_INT_M: usize = 0x000007FF; // Interrupt Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS0 register.
//
// *****************************************************************************
pub const NVIC_DIS0_INT_M: usize = 0xFFFFFFFF; // Interrupt Disable
pub const NVIC_DIS0_INT0: usize = 0x00000001; // Interrupt 0 disable
pub const NVIC_DIS0_INT1: usize = 0x00000002; // Interrupt 1 disable
pub const NVIC_DIS0_INT2: usize = 0x00000004; // Interrupt 2 disable
pub const NVIC_DIS0_INT3: usize = 0x00000008; // Interrupt 3 disable
pub const NVIC_DIS0_INT4: usize = 0x00000010; // Interrupt 4 disable
pub const NVIC_DIS0_INT5: usize = 0x00000020; // Interrupt 5 disable
pub const NVIC_DIS0_INT6: usize = 0x00000040; // Interrupt 6 disable
pub const NVIC_DIS0_INT7: usize = 0x00000080; // Interrupt 7 disable
pub const NVIC_DIS0_INT8: usize = 0x00000100; // Interrupt 8 disable
pub const NVIC_DIS0_INT9: usize = 0x00000200; // Interrupt 9 disable
pub const NVIC_DIS0_INT10: usize = 0x00000400; // Interrupt 10 disable
pub const NVIC_DIS0_INT11: usize = 0x00000800; // Interrupt 11 disable
pub const NVIC_DIS0_INT12: usize = 0x00001000; // Interrupt 12 disable
pub const NVIC_DIS0_INT13: usize = 0x00002000; // Interrupt 13 disable
pub const NVIC_DIS0_INT14: usize = 0x00004000; // Interrupt 14 disable
pub const NVIC_DIS0_INT15: usize = 0x00008000; // Interrupt 15 disable
pub const NVIC_DIS0_INT16: usize = 0x00010000; // Interrupt 16 disable
pub const NVIC_DIS0_INT17: usize = 0x00020000; // Interrupt 17 disable
pub const NVIC_DIS0_INT18: usize = 0x00040000; // Interrupt 18 disable
pub const NVIC_DIS0_INT19: usize = 0x00080000; // Interrupt 19 disable
pub const NVIC_DIS0_INT20: usize = 0x00100000; // Interrupt 20 disable
pub const NVIC_DIS0_INT21: usize = 0x00200000; // Interrupt 21 disable
pub const NVIC_DIS0_INT22: usize = 0x00400000; // Interrupt 22 disable
pub const NVIC_DIS0_INT23: usize = 0x00800000; // Interrupt 23 disable
pub const NVIC_DIS0_INT24: usize = 0x01000000; // Interrupt 24 disable
pub const NVIC_DIS0_INT25: usize = 0x02000000; // Interrupt 25 disable
pub const NVIC_DIS0_INT26: usize = 0x04000000; // Interrupt 26 disable
pub const NVIC_DIS0_INT27: usize = 0x08000000; // Interrupt 27 disable
pub const NVIC_DIS0_INT28: usize = 0x10000000; // Interrupt 28 disable
pub const NVIC_DIS0_INT29: usize = 0x20000000; // Interrupt 29 disable
pub const NVIC_DIS0_INT30: usize = 0x40000000; // Interrupt 30 disable
pub const NVIC_DIS0_INT31: usize = 0x80000000; // Interrupt 31 disable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS1 register.
//
// *****************************************************************************
pub const NVIC_DIS1_INT_M: usize = 0xFFFFFFFF; // Interrupt Disable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS2 register.
//
// *****************************************************************************
pub const NVIC_DIS2_INT_M: usize = 0xFFFFFFFF; // Interrupt Disable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS3 register.
//
// *****************************************************************************
pub const NVIC_DIS3_INT_M: usize = 0xFFFFFFFF; // Interrupt Disable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DIS4 register.
//
// *****************************************************************************
pub const NVIC_DIS4_INT_M: usize = 0x000007FF; // Interrupt Disable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND0 register.
//
// *****************************************************************************
pub const NVIC_PEND0_INT_M: usize = 0xFFFFFFFF; // Interrupt Set Pending
pub const NVIC_PEND0_INT0: usize = 0x00000001; // Interrupt 0 pend
pub const NVIC_PEND0_INT1: usize = 0x00000002; // Interrupt 1 pend
pub const NVIC_PEND0_INT2: usize = 0x00000004; // Interrupt 2 pend
pub const NVIC_PEND0_INT3: usize = 0x00000008; // Interrupt 3 pend
pub const NVIC_PEND0_INT4: usize = 0x00000010; // Interrupt 4 pend
pub const NVIC_PEND0_INT5: usize = 0x00000020; // Interrupt 5 pend
pub const NVIC_PEND0_INT6: usize = 0x00000040; // Interrupt 6 pend
pub const NVIC_PEND0_INT7: usize = 0x00000080; // Interrupt 7 pend
pub const NVIC_PEND0_INT8: usize = 0x00000100; // Interrupt 8 pend
pub const NVIC_PEND0_INT9: usize = 0x00000200; // Interrupt 9 pend
pub const NVIC_PEND0_INT10: usize = 0x00000400; // Interrupt 10 pend
pub const NVIC_PEND0_INT11: usize = 0x00000800; // Interrupt 11 pend
pub const NVIC_PEND0_INT12: usize = 0x00001000; // Interrupt 12 pend
pub const NVIC_PEND0_INT13: usize = 0x00002000; // Interrupt 13 pend
pub const NVIC_PEND0_INT14: usize = 0x00004000; // Interrupt 14 pend
pub const NVIC_PEND0_INT15: usize = 0x00008000; // Interrupt 15 pend
pub const NVIC_PEND0_INT16: usize = 0x00010000; // Interrupt 16 pend
pub const NVIC_PEND0_INT17: usize = 0x00020000; // Interrupt 17 pend
pub const NVIC_PEND0_INT18: usize = 0x00040000; // Interrupt 18 pend
pub const NVIC_PEND0_INT19: usize = 0x00080000; // Interrupt 19 pend
pub const NVIC_PEND0_INT20: usize = 0x00100000; // Interrupt 20 pend
pub const NVIC_PEND0_INT21: usize = 0x00200000; // Interrupt 21 pend
pub const NVIC_PEND0_INT22: usize = 0x00400000; // Interrupt 22 pend
pub const NVIC_PEND0_INT23: usize = 0x00800000; // Interrupt 23 pend
pub const NVIC_PEND0_INT24: usize = 0x01000000; // Interrupt 24 pend
pub const NVIC_PEND0_INT25: usize = 0x02000000; // Interrupt 25 pend
pub const NVIC_PEND0_INT26: usize = 0x04000000; // Interrupt 26 pend
pub const NVIC_PEND0_INT27: usize = 0x08000000; // Interrupt 27 pend
pub const NVIC_PEND0_INT28: usize = 0x10000000; // Interrupt 28 pend
pub const NVIC_PEND0_INT29: usize = 0x20000000; // Interrupt 29 pend
pub const NVIC_PEND0_INT30: usize = 0x40000000; // Interrupt 30 pend
pub const NVIC_PEND0_INT31: usize = 0x80000000; // Interrupt 31 pend

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND1 register.
//
// *****************************************************************************
pub const NVIC_PEND1_INT_M: usize = 0xFFFFFFFF; // Interrupt Set Pending

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND2 register.
//
// *****************************************************************************
pub const NVIC_PEND2_INT_M: usize = 0xFFFFFFFF; // Interrupt Set Pending

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND3 register.
//
// *****************************************************************************
pub const NVIC_PEND3_INT_M: usize = 0xFFFFFFFF; // Interrupt Set Pending

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PEND4 register.
//
// *****************************************************************************
pub const NVIC_PEND4_INT_M: usize = 0x000007FF; // Interrupt Set Pending

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND0 register.
//
// *****************************************************************************
pub const NVIC_UNPEND0_INT_M: usize = 0xFFFFFFFF; // Interrupt Clear Pending
pub const NVIC_UNPEND0_INT0: usize = 0x00000001; // Interrupt 0 unpend
pub const NVIC_UNPEND0_INT1: usize = 0x00000002; // Interrupt 1 unpend
pub const NVIC_UNPEND0_INT2: usize = 0x00000004; // Interrupt 2 unpend
pub const NVIC_UNPEND0_INT3: usize = 0x00000008; // Interrupt 3 unpend
pub const NVIC_UNPEND0_INT4: usize = 0x00000010; // Interrupt 4 unpend
pub const NVIC_UNPEND0_INT5: usize = 0x00000020; // Interrupt 5 unpend
pub const NVIC_UNPEND0_INT6: usize = 0x00000040; // Interrupt 6 unpend
pub const NVIC_UNPEND0_INT7: usize = 0x00000080; // Interrupt 7 unpend
pub const NVIC_UNPEND0_INT8: usize = 0x00000100; // Interrupt 8 unpend
pub const NVIC_UNPEND0_INT9: usize = 0x00000200; // Interrupt 9 unpend
pub const NVIC_UNPEND0_INT10: usize = 0x00000400; // Interrupt 10 unpend
pub const NVIC_UNPEND0_INT11: usize = 0x00000800; // Interrupt 11 unpend
pub const NVIC_UNPEND0_INT12: usize = 0x00001000; // Interrupt 12 unpend
pub const NVIC_UNPEND0_INT13: usize = 0x00002000; // Interrupt 13 unpend
pub const NVIC_UNPEND0_INT14: usize = 0x00004000; // Interrupt 14 unpend
pub const NVIC_UNPEND0_INT15: usize = 0x00008000; // Interrupt 15 unpend
pub const NVIC_UNPEND0_INT16: usize = 0x00010000; // Interrupt 16 unpend
pub const NVIC_UNPEND0_INT17: usize = 0x00020000; // Interrupt 17 unpend
pub const NVIC_UNPEND0_INT18: usize = 0x00040000; // Interrupt 18 unpend
pub const NVIC_UNPEND0_INT19: usize = 0x00080000; // Interrupt 19 unpend
pub const NVIC_UNPEND0_INT20: usize = 0x00100000; // Interrupt 20 unpend
pub const NVIC_UNPEND0_INT21: usize = 0x00200000; // Interrupt 21 unpend
pub const NVIC_UNPEND0_INT22: usize = 0x00400000; // Interrupt 22 unpend
pub const NVIC_UNPEND0_INT23: usize = 0x00800000; // Interrupt 23 unpend
pub const NVIC_UNPEND0_INT24: usize = 0x01000000; // Interrupt 24 unpend
pub const NVIC_UNPEND0_INT25: usize = 0x02000000; // Interrupt 25 unpend
pub const NVIC_UNPEND0_INT26: usize = 0x04000000; // Interrupt 26 unpend
pub const NVIC_UNPEND0_INT27: usize = 0x08000000; // Interrupt 27 unpend
pub const NVIC_UNPEND0_INT28: usize = 0x10000000; // Interrupt 28 unpend
pub const NVIC_UNPEND0_INT29: usize = 0x20000000; // Interrupt 29 unpend
pub const NVIC_UNPEND0_INT30: usize = 0x40000000; // Interrupt 30 unpend
pub const NVIC_UNPEND0_INT31: usize = 0x80000000; // Interrupt 31 unpend

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND1 register.
//
// *****************************************************************************
pub const NVIC_UNPEND1_INT_M: usize = 0xFFFFFFFF; // Interrupt Clear Pending
pub const NVIC_UNPEND1_INT32: usize = 0x00000001; // Interrupt 32 unpend
pub const NVIC_UNPEND1_INT33: usize = 0x00000002; // Interrupt 33 unpend
pub const NVIC_UNPEND1_INT34: usize = 0x00000004; // Interrupt 34 unpend
pub const NVIC_UNPEND1_INT35: usize = 0x00000008; // Interrupt 35 unpend
pub const NVIC_UNPEND1_INT36: usize = 0x00000010; // Interrupt 36 unpend
pub const NVIC_UNPEND1_INT37: usize = 0x00000020; // Interrupt 37 unpend
pub const NVIC_UNPEND1_INT38: usize = 0x00000040; // Interrupt 38 unpend
pub const NVIC_UNPEND1_INT39: usize = 0x00000080; // Interrupt 39 unpend
pub const NVIC_UNPEND1_INT40: usize = 0x00000100; // Interrupt 40 unpend
pub const NVIC_UNPEND1_INT41: usize = 0x00000200; // Interrupt 41 unpend
pub const NVIC_UNPEND1_INT42: usize = 0x00000400; // Interrupt 42 unpend
pub const NVIC_UNPEND1_INT43: usize = 0x00000800; // Interrupt 43 unpend
pub const NVIC_UNPEND1_INT44: usize = 0x00001000; // Interrupt 44 unpend
pub const NVIC_UNPEND1_INT45: usize = 0x00002000; // Interrupt 45 unpend
pub const NVIC_UNPEND1_INT46: usize = 0x00004000; // Interrupt 46 unpend
pub const NVIC_UNPEND1_INT47: usize = 0x00008000; // Interrupt 47 unpend
pub const NVIC_UNPEND1_INT48: usize = 0x00010000; // Interrupt 48 unpend
pub const NVIC_UNPEND1_INT49: usize = 0x00020000; // Interrupt 49 unpend
pub const NVIC_UNPEND1_INT50: usize = 0x00040000; // Interrupt 50 unpend
pub const NVIC_UNPEND1_INT51: usize = 0x00080000; // Interrupt 51 unpend
pub const NVIC_UNPEND1_INT52: usize = 0x00100000; // Interrupt 52 unpend
pub const NVIC_UNPEND1_INT53: usize = 0x00200000; // Interrupt 53 unpend
pub const NVIC_UNPEND1_INT54: usize = 0x00400000; // Interrupt 54 unpend
pub const NVIC_UNPEND1_INT55: usize = 0x00800000; // Interrupt 55 unpend

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND2 register.
//
// *****************************************************************************
pub const NVIC_UNPEND2_INT_M: usize = 0xFFFFFFFF; // Interrupt Clear Pending

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND3 register.
//
// *****************************************************************************
pub const NVIC_UNPEND3_INT_M: usize = 0xFFFFFFFF; // Interrupt Clear Pending

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_UNPEND4 register.
//
// *****************************************************************************
pub const NVIC_UNPEND4_INT_M: usize = 0x000007FF; // Interrupt Clear Pending

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE0 register.
//
// *****************************************************************************
pub const NVIC_ACTIVE0_INT_M: usize = 0xFFFFFFFF; // Interrupt Active
pub const NVIC_ACTIVE0_INT0: usize = 0x00000001; // Interrupt 0 active
pub const NVIC_ACTIVE0_INT1: usize = 0x00000002; // Interrupt 1 active
pub const NVIC_ACTIVE0_INT2: usize = 0x00000004; // Interrupt 2 active
pub const NVIC_ACTIVE0_INT3: usize = 0x00000008; // Interrupt 3 active
pub const NVIC_ACTIVE0_INT4: usize = 0x00000010; // Interrupt 4 active
pub const NVIC_ACTIVE0_INT5: usize = 0x00000020; // Interrupt 5 active
pub const NVIC_ACTIVE0_INT6: usize = 0x00000040; // Interrupt 6 active
pub const NVIC_ACTIVE0_INT7: usize = 0x00000080; // Interrupt 7 active
pub const NVIC_ACTIVE0_INT8: usize = 0x00000100; // Interrupt 8 active
pub const NVIC_ACTIVE0_INT9: usize = 0x00000200; // Interrupt 9 active
pub const NVIC_ACTIVE0_INT10: usize = 0x00000400; // Interrupt 10 active
pub const NVIC_ACTIVE0_INT11: usize = 0x00000800; // Interrupt 11 active
pub const NVIC_ACTIVE0_INT12: usize = 0x00001000; // Interrupt 12 active
pub const NVIC_ACTIVE0_INT13: usize = 0x00002000; // Interrupt 13 active
pub const NVIC_ACTIVE0_INT14: usize = 0x00004000; // Interrupt 14 active
pub const NVIC_ACTIVE0_INT15: usize = 0x00008000; // Interrupt 15 active
pub const NVIC_ACTIVE0_INT16: usize = 0x00010000; // Interrupt 16 active
pub const NVIC_ACTIVE0_INT17: usize = 0x00020000; // Interrupt 17 active
pub const NVIC_ACTIVE0_INT18: usize = 0x00040000; // Interrupt 18 active
pub const NVIC_ACTIVE0_INT19: usize = 0x00080000; // Interrupt 19 active
pub const NVIC_ACTIVE0_INT20: usize = 0x00100000; // Interrupt 20 active
pub const NVIC_ACTIVE0_INT21: usize = 0x00200000; // Interrupt 21 active
pub const NVIC_ACTIVE0_INT22: usize = 0x00400000; // Interrupt 22 active
pub const NVIC_ACTIVE0_INT23: usize = 0x00800000; // Interrupt 23 active
pub const NVIC_ACTIVE0_INT24: usize = 0x01000000; // Interrupt 24 active
pub const NVIC_ACTIVE0_INT25: usize = 0x02000000; // Interrupt 25 active
pub const NVIC_ACTIVE0_INT26: usize = 0x04000000; // Interrupt 26 active
pub const NVIC_ACTIVE0_INT27: usize = 0x08000000; // Interrupt 27 active
pub const NVIC_ACTIVE0_INT28: usize = 0x10000000; // Interrupt 28 active
pub const NVIC_ACTIVE0_INT29: usize = 0x20000000; // Interrupt 29 active
pub const NVIC_ACTIVE0_INT30: usize = 0x40000000; // Interrupt 30 active
pub const NVIC_ACTIVE0_INT31: usize = 0x80000000; // Interrupt 31 active

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE1 register.
//
// *****************************************************************************
pub const NVIC_ACTIVE1_INT_M: usize = 0xFFFFFFFF; // Interrupt Active

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE2 register.
//
// *****************************************************************************
pub const NVIC_ACTIVE2_INT_M: usize = 0xFFFFFFFF; // Interrupt Active

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE3 register.
//
// *****************************************************************************
pub const NVIC_ACTIVE3_INT_M: usize = 0xFFFFFFFF; // Interrupt Active

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_ACTIVE4 register.
//
// *****************************************************************************
pub const NVIC_ACTIVE4_INT_M: usize = 0x000007FF; // Interrupt Active

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI0 register.
//
// *****************************************************************************
pub const NVIC_PRI0_INT3_M: usize = 0xE0000000; // Interrupt 3 Priority Mask
pub const NVIC_PRI0_INT2_M: usize = 0x00E00000; // Interrupt 2 Priority Mask
pub const NVIC_PRI0_INT1_M: usize = 0x0000E000; // Interrupt 1 Priority Mask
pub const NVIC_PRI0_INT0_M: usize = 0x000000E0; // Interrupt 0 Priority Mask
pub const NVIC_PRI0_INT3_S: usize = 29;
pub const NVIC_PRI0_INT2_S: usize = 21;
pub const NVIC_PRI0_INT1_S: usize = 13;
pub const NVIC_PRI0_INT0_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI1 register.
//
// *****************************************************************************
pub const NVIC_PRI1_INT7_M: usize = 0xE0000000; // Interrupt 7 Priority Mask
pub const NVIC_PRI1_INT6_M: usize = 0x00E00000; // Interrupt 6 Priority Mask
pub const NVIC_PRI1_INT5_M: usize = 0x0000E000; // Interrupt 5 Priority Mask
pub const NVIC_PRI1_INT4_M: usize = 0x000000E0; // Interrupt 4 Priority Mask
pub const NVIC_PRI1_INT7_S: usize = 29;
pub const NVIC_PRI1_INT6_S: usize = 21;
pub const NVIC_PRI1_INT5_S: usize = 13;
pub const NVIC_PRI1_INT4_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI2 register.
//
// *****************************************************************************
pub const NVIC_PRI2_INT11_M: usize = 0xE0000000; // Interrupt 11 Priority Mask
pub const NVIC_PRI2_INT10_M: usize = 0x00E00000; // Interrupt 10 Priority Mask
pub const NVIC_PRI2_INT9_M: usize = 0x0000E000; // Interrupt 9 Priority Mask
pub const NVIC_PRI2_INT8_M: usize = 0x000000E0; // Interrupt 8 Priority Mask
pub const NVIC_PRI2_INT11_S: usize = 29;
pub const NVIC_PRI2_INT10_S: usize = 21;
pub const NVIC_PRI2_INT9_S: usize = 13;
pub const NVIC_PRI2_INT8_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI3 register.
//
// *****************************************************************************
pub const NVIC_PRI3_INT15_M: usize = 0xE0000000; // Interrupt 15 Priority Mask
pub const NVIC_PRI3_INT14_M: usize = 0x00E00000; // Interrupt 14 Priority Mask
pub const NVIC_PRI3_INT13_M: usize = 0x0000E000; // Interrupt 13 Priority Mask
pub const NVIC_PRI3_INT12_M: usize = 0x000000E0; // Interrupt 12 Priority Mask
pub const NVIC_PRI3_INT15_S: usize = 29;
pub const NVIC_PRI3_INT14_S: usize = 21;
pub const NVIC_PRI3_INT13_S: usize = 13;
pub const NVIC_PRI3_INT12_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI4 register.
//
// *****************************************************************************
pub const NVIC_PRI4_INT19_M: usize = 0xE0000000; // Interrupt 19 Priority Mask
pub const NVIC_PRI4_INT18_M: usize = 0x00E00000; // Interrupt 18 Priority Mask
pub const NVIC_PRI4_INT17_M: usize = 0x0000E000; // Interrupt 17 Priority Mask
pub const NVIC_PRI4_INT16_M: usize = 0x000000E0; // Interrupt 16 Priority Mask
pub const NVIC_PRI4_INT19_S: usize = 29;
pub const NVIC_PRI4_INT18_S: usize = 21;
pub const NVIC_PRI4_INT17_S: usize = 13;
pub const NVIC_PRI4_INT16_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI5 register.
//
// *****************************************************************************
pub const NVIC_PRI5_INT23_M: usize = 0xE0000000; // Interrupt 23 Priority Mask
pub const NVIC_PRI5_INT22_M: usize = 0x00E00000; // Interrupt 22 Priority Mask
pub const NVIC_PRI5_INT21_M: usize = 0x0000E000; // Interrupt 21 Priority Mask
pub const NVIC_PRI5_INT20_M: usize = 0x000000E0; // Interrupt 20 Priority Mask
pub const NVIC_PRI5_INT23_S: usize = 29;
pub const NVIC_PRI5_INT22_S: usize = 21;
pub const NVIC_PRI5_INT21_S: usize = 13;
pub const NVIC_PRI5_INT20_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI6 register.
//
// *****************************************************************************
pub const NVIC_PRI6_INT27_M: usize = 0xE0000000; // Interrupt 27 Priority Mask
pub const NVIC_PRI6_INT26_M: usize = 0x00E00000; // Interrupt 26 Priority Mask
pub const NVIC_PRI6_INT25_M: usize = 0x0000E000; // Interrupt 25 Priority Mask
pub const NVIC_PRI6_INT24_M: usize = 0x000000E0; // Interrupt 24 Priority Mask
pub const NVIC_PRI6_INT27_S: usize = 29;
pub const NVIC_PRI6_INT26_S: usize = 21;
pub const NVIC_PRI6_INT25_S: usize = 13;
pub const NVIC_PRI6_INT24_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI7 register.
//
// *****************************************************************************
pub const NVIC_PRI7_INT31_M: usize = 0xE0000000; // Interrupt 31 Priority Mask
pub const NVIC_PRI7_INT30_M: usize = 0x00E00000; // Interrupt 30 Priority Mask
pub const NVIC_PRI7_INT29_M: usize = 0x0000E000; // Interrupt 29 Priority Mask
pub const NVIC_PRI7_INT28_M: usize = 0x000000E0; // Interrupt 28 Priority Mask
pub const NVIC_PRI7_INT31_S: usize = 29;
pub const NVIC_PRI7_INT30_S: usize = 21;
pub const NVIC_PRI7_INT29_S: usize = 13;
pub const NVIC_PRI7_INT28_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI8 register.
//
// *****************************************************************************
pub const NVIC_PRI8_INT35_M: usize = 0xE0000000; // Interrupt 35 Priority Mask
pub const NVIC_PRI8_INT34_M: usize = 0x00E00000; // Interrupt 34 Priority Mask
pub const NVIC_PRI8_INT33_M: usize = 0x0000E000; // Interrupt 33 Priority Mask
pub const NVIC_PRI8_INT32_M: usize = 0x000000E0; // Interrupt 32 Priority Mask
pub const NVIC_PRI8_INT35_S: usize = 29;
pub const NVIC_PRI8_INT34_S: usize = 21;
pub const NVIC_PRI8_INT33_S: usize = 13;
pub const NVIC_PRI8_INT32_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI9 register.
//
// *****************************************************************************
pub const NVIC_PRI9_INT39_M: usize = 0xE0000000; // Interrupt 39 Priority Mask
pub const NVIC_PRI9_INT38_M: usize = 0x00E00000; // Interrupt 38 Priority Mask
pub const NVIC_PRI9_INT37_M: usize = 0x0000E000; // Interrupt 37 Priority Mask
pub const NVIC_PRI9_INT36_M: usize = 0x000000E0; // Interrupt 36 Priority Mask
pub const NVIC_PRI9_INT39_S: usize = 29;
pub const NVIC_PRI9_INT38_S: usize = 21;
pub const NVIC_PRI9_INT37_S: usize = 13;
pub const NVIC_PRI9_INT36_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI10 register.
//
// *****************************************************************************
pub const NVIC_PRI10_INT43_M: usize = 0xE0000000; // Interrupt 43 Priority Mask
pub const NVIC_PRI10_INT42_M: usize = 0x00E00000; // Interrupt 42 Priority Mask
pub const NVIC_PRI10_INT41_M: usize = 0x0000E000; // Interrupt 41 Priority Mask
pub const NVIC_PRI10_INT40_M: usize = 0x000000E0; // Interrupt 40 Priority Mask
pub const NVIC_PRI10_INT43_S: usize = 29;
pub const NVIC_PRI10_INT42_S: usize = 21;
pub const NVIC_PRI10_INT41_S: usize = 13;
pub const NVIC_PRI10_INT40_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI11 register.
//
// *****************************************************************************
pub const NVIC_PRI11_INT47_M: usize = 0xE0000000; // Interrupt 47 Priority Mask
pub const NVIC_PRI11_INT46_M: usize = 0x00E00000; // Interrupt 46 Priority Mask
pub const NVIC_PRI11_INT45_M: usize = 0x0000E000; // Interrupt 45 Priority Mask
pub const NVIC_PRI11_INT44_M: usize = 0x000000E0; // Interrupt 44 Priority Mask
pub const NVIC_PRI11_INT47_S: usize = 29;
pub const NVIC_PRI11_INT46_S: usize = 21;
pub const NVIC_PRI11_INT45_S: usize = 13;
pub const NVIC_PRI11_INT44_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI12 register.
//
// *****************************************************************************
pub const NVIC_PRI12_INT51_M: usize = 0xE0000000; // Interrupt 51 Priority Mask
pub const NVIC_PRI12_INT50_M: usize = 0x00E00000; // Interrupt 50 Priority Mask
pub const NVIC_PRI12_INT49_M: usize = 0x0000E000; // Interrupt 49 Priority Mask
pub const NVIC_PRI12_INT48_M: usize = 0x000000E0; // Interrupt 48 Priority Mask
pub const NVIC_PRI12_INT51_S: usize = 29;
pub const NVIC_PRI12_INT50_S: usize = 21;
pub const NVIC_PRI12_INT49_S: usize = 13;
pub const NVIC_PRI12_INT48_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI13 register.
//
// *****************************************************************************
pub const NVIC_PRI13_INT55_M: usize = 0xE0000000; // Interrupt 55 Priority Mask
pub const NVIC_PRI13_INT54_M: usize = 0x00E00000; // Interrupt 54 Priority Mask
pub const NVIC_PRI13_INT53_M: usize = 0x0000E000; // Interrupt 53 Priority Mask
pub const NVIC_PRI13_INT52_M: usize = 0x000000E0; // Interrupt 52 Priority Mask
pub const NVIC_PRI13_INT55_S: usize = 29;
pub const NVIC_PRI13_INT54_S: usize = 21;
pub const NVIC_PRI13_INT53_S: usize = 13;
pub const NVIC_PRI13_INT52_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI14 register.
//
// *****************************************************************************
pub const NVIC_PRI14_INTD_M: usize = 0xE0000000; // Interrupt 59 Priority Mask
pub const NVIC_PRI14_INTC_M: usize = 0x00E00000; // Interrupt 58 Priority Mask
pub const NVIC_PRI14_INTB_M: usize = 0x0000E000; // Interrupt 57 Priority Mask
pub const NVIC_PRI14_INTA_M: usize = 0x000000E0; // Interrupt 56 Priority Mask
pub const NVIC_PRI14_INTD_S: usize = 29;
pub const NVIC_PRI14_INTC_S: usize = 21;
pub const NVIC_PRI14_INTB_S: usize = 13;
pub const NVIC_PRI14_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI15 register.
//
// *****************************************************************************
pub const NVIC_PRI15_INTD_M: usize = 0xE0000000; // Interrupt 63 Priority Mask
pub const NVIC_PRI15_INTC_M: usize = 0x00E00000; // Interrupt 62 Priority Mask
pub const NVIC_PRI15_INTB_M: usize = 0x0000E000; // Interrupt 61 Priority Mask
pub const NVIC_PRI15_INTA_M: usize = 0x000000E0; // Interrupt 60 Priority Mask
pub const NVIC_PRI15_INTD_S: usize = 29;
pub const NVIC_PRI15_INTC_S: usize = 21;
pub const NVIC_PRI15_INTB_S: usize = 13;
pub const NVIC_PRI15_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI16 register.
//
// *****************************************************************************
pub const NVIC_PRI16_INTD_M: usize = 0xE0000000; // Interrupt 67 Priority Mask
pub const NVIC_PRI16_INTC_M: usize = 0x00E00000; // Interrupt 66 Priority Mask
pub const NVIC_PRI16_INTB_M: usize = 0x0000E000; // Interrupt 65 Priority Mask
pub const NVIC_PRI16_INTA_M: usize = 0x000000E0; // Interrupt 64 Priority Mask
pub const NVIC_PRI16_INTD_S: usize = 29;
pub const NVIC_PRI16_INTC_S: usize = 21;
pub const NVIC_PRI16_INTB_S: usize = 13;
pub const NVIC_PRI16_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI17 register.
//
// *****************************************************************************
pub const NVIC_PRI17_INTD_M: usize = 0xE0000000; // Interrupt 71 Priority Mask
pub const NVIC_PRI17_INTC_M: usize = 0x00E00000; // Interrupt 70 Priority Mask
pub const NVIC_PRI17_INTB_M: usize = 0x0000E000; // Interrupt 69 Priority Mask
pub const NVIC_PRI17_INTA_M: usize = 0x000000E0; // Interrupt 68 Priority Mask
pub const NVIC_PRI17_INTD_S: usize = 29;
pub const NVIC_PRI17_INTC_S: usize = 21;
pub const NVIC_PRI17_INTB_S: usize = 13;
pub const NVIC_PRI17_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI18 register.
//
// *****************************************************************************
pub const NVIC_PRI18_INTD_M: usize = 0xE0000000; // Interrupt 75 Priority Mask
pub const NVIC_PRI18_INTC_M: usize = 0x00E00000; // Interrupt 74 Priority Mask
pub const NVIC_PRI18_INTB_M: usize = 0x0000E000; // Interrupt 73 Priority Mask
pub const NVIC_PRI18_INTA_M: usize = 0x000000E0; // Interrupt 72 Priority Mask
pub const NVIC_PRI18_INTD_S: usize = 29;
pub const NVIC_PRI18_INTC_S: usize = 21;
pub const NVIC_PRI18_INTB_S: usize = 13;
pub const NVIC_PRI18_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI19 register.
//
// *****************************************************************************
pub const NVIC_PRI19_INTD_M: usize = 0xE0000000; // Interrupt 79 Priority Mask
pub const NVIC_PRI19_INTC_M: usize = 0x00E00000; // Interrupt 78 Priority Mask
pub const NVIC_PRI19_INTB_M: usize = 0x0000E000; // Interrupt 77 Priority Mask
pub const NVIC_PRI19_INTA_M: usize = 0x000000E0; // Interrupt 76 Priority Mask
pub const NVIC_PRI19_INTD_S: usize = 29;
pub const NVIC_PRI19_INTC_S: usize = 21;
pub const NVIC_PRI19_INTB_S: usize = 13;
pub const NVIC_PRI19_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI20 register.
//
// *****************************************************************************
pub const NVIC_PRI20_INTD_M: usize = 0xE0000000; // Interrupt 83 Priority Mask
pub const NVIC_PRI20_INTC_M: usize = 0x00E00000; // Interrupt 82 Priority Mask
pub const NVIC_PRI20_INTB_M: usize = 0x0000E000; // Interrupt 81 Priority Mask
pub const NVIC_PRI20_INTA_M: usize = 0x000000E0; // Interrupt 80 Priority Mask
pub const NVIC_PRI20_INTD_S: usize = 29;
pub const NVIC_PRI20_INTC_S: usize = 21;
pub const NVIC_PRI20_INTB_S: usize = 13;
pub const NVIC_PRI20_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI21 register.
//
// *****************************************************************************
pub const NVIC_PRI21_INTD_M: usize = 0xE0000000; // Interrupt 87 Priority Mask
pub const NVIC_PRI21_INTC_M: usize = 0x00E00000; // Interrupt 86 Priority Mask
pub const NVIC_PRI21_INTB_M: usize = 0x0000E000; // Interrupt 85 Priority Mask
pub const NVIC_PRI21_INTA_M: usize = 0x000000E0; // Interrupt 84 Priority Mask
pub const NVIC_PRI21_INTD_S: usize = 29;
pub const NVIC_PRI21_INTC_S: usize = 21;
pub const NVIC_PRI21_INTB_S: usize = 13;
pub const NVIC_PRI21_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI22 register.
//
// *****************************************************************************
pub const NVIC_PRI22_INTD_M: usize = 0xE0000000; // Interrupt 91 Priority Mask
pub const NVIC_PRI22_INTC_M: usize = 0x00E00000; // Interrupt 90 Priority Mask
pub const NVIC_PRI22_INTB_M: usize = 0x0000E000; // Interrupt 89 Priority Mask
pub const NVIC_PRI22_INTA_M: usize = 0x000000E0; // Interrupt 88 Priority Mask
pub const NVIC_PRI22_INTD_S: usize = 29;
pub const NVIC_PRI22_INTC_S: usize = 21;
pub const NVIC_PRI22_INTB_S: usize = 13;
pub const NVIC_PRI22_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI23 register.
//
// *****************************************************************************
pub const NVIC_PRI23_INTD_M: usize = 0xE0000000; // Interrupt 95 Priority Mask
pub const NVIC_PRI23_INTC_M: usize = 0x00E00000; // Interrupt 94 Priority Mask
pub const NVIC_PRI23_INTB_M: usize = 0x0000E000; // Interrupt 93 Priority Mask
pub const NVIC_PRI23_INTA_M: usize = 0x000000E0; // Interrupt 92 Priority Mask
pub const NVIC_PRI23_INTD_S: usize = 29;
pub const NVIC_PRI23_INTC_S: usize = 21;
pub const NVIC_PRI23_INTB_S: usize = 13;
pub const NVIC_PRI23_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI24 register.
//
// *****************************************************************************
pub const NVIC_PRI24_INTD_M: usize = 0xE0000000; // Interrupt 99 Priority Mask
pub const NVIC_PRI24_INTC_M: usize = 0x00E00000; // Interrupt 98 Priority Mask
pub const NVIC_PRI24_INTB_M: usize = 0x0000E000; // Interrupt 97 Priority Mask
pub const NVIC_PRI24_INTA_M: usize = 0x000000E0; // Interrupt 96 Priority Mask
pub const NVIC_PRI24_INTD_S: usize = 29;
pub const NVIC_PRI24_INTC_S: usize = 21;
pub const NVIC_PRI24_INTB_S: usize = 13;
pub const NVIC_PRI24_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI25 register.
//
// *****************************************************************************
pub const NVIC_PRI25_INTD_M: usize = 0xE0000000; // Interrupt 103 Priority Mask
pub const NVIC_PRI25_INTC_M: usize = 0x00E00000; // Interrupt 102 Priority Mask
pub const NVIC_PRI25_INTB_M: usize = 0x0000E000; // Interrupt 101 Priority Mask
pub const NVIC_PRI25_INTA_M: usize = 0x000000E0; // Interrupt 100 Priority Mask
pub const NVIC_PRI25_INTD_S: usize = 29;
pub const NVIC_PRI25_INTC_S: usize = 21;
pub const NVIC_PRI25_INTB_S: usize = 13;
pub const NVIC_PRI25_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI26 register.
//
// *****************************************************************************
pub const NVIC_PRI26_INTD_M: usize = 0xE0000000; // Interrupt 107 Priority Mask
pub const NVIC_PRI26_INTC_M: usize = 0x00E00000; // Interrupt 106 Priority Mask
pub const NVIC_PRI26_INTB_M: usize = 0x0000E000; // Interrupt 105 Priority Mask
pub const NVIC_PRI26_INTA_M: usize = 0x000000E0; // Interrupt 104 Priority Mask
pub const NVIC_PRI26_INTD_S: usize = 29;
pub const NVIC_PRI26_INTC_S: usize = 21;
pub const NVIC_PRI26_INTB_S: usize = 13;
pub const NVIC_PRI26_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI27 register.
//
// *****************************************************************************
pub const NVIC_PRI27_INTD_M: usize = 0xE0000000; // Interrupt 111 Priority Mask
pub const NVIC_PRI27_INTC_M: usize = 0x00E00000; // Interrupt 110 Priority Mask
pub const NVIC_PRI27_INTB_M: usize = 0x0000E000; // Interrupt 109 Priority Mask
pub const NVIC_PRI27_INTA_M: usize = 0x000000E0; // Interrupt 108 Priority Mask
pub const NVIC_PRI27_INTD_S: usize = 29;
pub const NVIC_PRI27_INTC_S: usize = 21;
pub const NVIC_PRI27_INTB_S: usize = 13;
pub const NVIC_PRI27_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI28 register.
//
// *****************************************************************************
pub const NVIC_PRI28_INTD_M: usize = 0xE0000000; // Interrupt 115 Priority Mask
pub const NVIC_PRI28_INTC_M: usize = 0x00E00000; // Interrupt 114 Priority Mask
pub const NVIC_PRI28_INTB_M: usize = 0x0000E000; // Interrupt 113 Priority Mask
pub const NVIC_PRI28_INTA_M: usize = 0x000000E0; // Interrupt 112 Priority Mask
pub const NVIC_PRI28_INTD_S: usize = 29;
pub const NVIC_PRI28_INTC_S: usize = 21;
pub const NVIC_PRI28_INTB_S: usize = 13;
pub const NVIC_PRI28_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI29 register.
//
// *****************************************************************************
pub const NVIC_PRI29_INTD_M: usize = 0xE0000000; // Interrupt 119 Priority Mask
pub const NVIC_PRI29_INTC_M: usize = 0x00E00000; // Interrupt 118 Priority Mask
pub const NVIC_PRI29_INTB_M: usize = 0x0000E000; // Interrupt 117 Priority Mask
pub const NVIC_PRI29_INTA_M: usize = 0x000000E0; // Interrupt 116 Priority Mask
pub const NVIC_PRI29_INTD_S: usize = 29;
pub const NVIC_PRI29_INTC_S: usize = 21;
pub const NVIC_PRI29_INTB_S: usize = 13;
pub const NVIC_PRI29_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI30 register.
//
// *****************************************************************************
pub const NVIC_PRI30_INTD_M: usize = 0xE0000000; // Interrupt 123 Priority Mask
pub const NVIC_PRI30_INTC_M: usize = 0x00E00000; // Interrupt 122 Priority Mask
pub const NVIC_PRI30_INTB_M: usize = 0x0000E000; // Interrupt 121 Priority Mask
pub const NVIC_PRI30_INTA_M: usize = 0x000000E0; // Interrupt 120 Priority Mask
pub const NVIC_PRI30_INTD_S: usize = 29;
pub const NVIC_PRI30_INTC_S: usize = 21;
pub const NVIC_PRI30_INTB_S: usize = 13;
pub const NVIC_PRI30_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI31 register.
//
// *****************************************************************************
pub const NVIC_PRI31_INTD_M: usize = 0xE0000000; // Interrupt 127 Priority Mask
pub const NVIC_PRI31_INTC_M: usize = 0x00E00000; // Interrupt 126 Priority Mask
pub const NVIC_PRI31_INTB_M: usize = 0x0000E000; // Interrupt 125 Priority Mask
pub const NVIC_PRI31_INTA_M: usize = 0x000000E0; // Interrupt 124 Priority Mask
pub const NVIC_PRI31_INTD_S: usize = 29;
pub const NVIC_PRI31_INTC_S: usize = 21;
pub const NVIC_PRI31_INTB_S: usize = 13;
pub const NVIC_PRI31_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI32 register.
//
// *****************************************************************************
pub const NVIC_PRI32_INTD_M: usize = 0xE0000000; // Interrupt 131 Priority Mask
pub const NVIC_PRI32_INTC_M: usize = 0x00E00000; // Interrupt 130 Priority Mask
pub const NVIC_PRI32_INTB_M: usize = 0x0000E000; // Interrupt 129 Priority Mask
pub const NVIC_PRI32_INTA_M: usize = 0x000000E0; // Interrupt 128 Priority Mask
pub const NVIC_PRI32_INTD_S: usize = 29;
pub const NVIC_PRI32_INTC_S: usize = 21;
pub const NVIC_PRI32_INTB_S: usize = 13;
pub const NVIC_PRI32_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI33 register.
//
// *****************************************************************************
pub const NVIC_PRI33_INTD_M: usize = 0xE0000000; // Interrupt Priority for Interrupt [4n+3]
pub const NVIC_PRI33_INTC_M: usize = 0x00E00000; // Interrupt Priority for Interrupt [4n+2]
pub const NVIC_PRI33_INTB_M: usize = 0x0000E000; // Interrupt Priority for Interrupt [4n+1]
pub const NVIC_PRI33_INTA_M: usize = 0x000000E0; // Interrupt Priority for Interrupt [4n]
pub const NVIC_PRI33_INTD_S: usize = 29;
pub const NVIC_PRI33_INTC_S: usize = 21;
pub const NVIC_PRI33_INTB_S: usize = 13;
pub const NVIC_PRI33_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_PRI34 register.
//
// *****************************************************************************
pub const NVIC_PRI34_INTD_M: usize = 0xE0000000; // Interrupt Priority for Interrupt [4n+3]
pub const NVIC_PRI34_INTC_M: usize = 0x00E00000; // Interrupt Priority for Interrupt [4n+2]
pub const NVIC_PRI34_INTB_M: usize = 0x0000E000; // Interrupt Priority for Interrupt [4n+1]
pub const NVIC_PRI34_INTA_M: usize = 0x000000E0; // Interrupt Priority for Interrupt [4n]
pub const NVIC_PRI34_INTD_S: usize = 29;
pub const NVIC_PRI34_INTC_S: usize = 21;
pub const NVIC_PRI34_INTB_S: usize = 13;
pub const NVIC_PRI34_INTA_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPUID register.
//
// *****************************************************************************
pub const NVIC_CPUID_IMP_M: usize = 0xFF000000; // Implementer Code
pub const NVIC_CPUID_IMP_ARM: usize = 0x41000000; // ARM
pub const NVIC_CPUID_VAR_M: usize = 0x00F00000; // Variant Number
pub const NVIC_CPUID_CON_M: usize = 0x000F0000; // Constant
pub const NVIC_CPUID_PARTNO_M: usize = 0x0000FFF0; // Part Number
pub const NVIC_CPUID_PARTNO_CM4: usize = 0x0000C240; // Cortex-M4 processor
pub const NVIC_CPUID_REV_M: usize = 0x0000000F; // Revision Number

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_INT_CTRL register.
//
// *****************************************************************************
pub const NVIC_INT_CTRL_NMI_SET: usize = 0x80000000; // NMI Set Pending
pub const NVIC_INT_CTRL_PEND_SV: usize = 0x10000000; // PendSV Set Pending
pub const NVIC_INT_CTRL_UNPEND_SV: usize = 0x08000000; // PendSV Clear Pending
pub const NVIC_INT_CTRL_PENDSTSET: usize = 0x04000000; // SysTick Set Pending
pub const NVIC_INT_CTRL_PENDSTCLR: usize = 0x02000000; // SysTick Clear Pending
pub const NVIC_INT_CTRL_ISR_PRE: usize = 0x00800000; // Debug Interrupt Handling
pub const NVIC_INT_CTRL_ISR_PEND: usize = 0x00400000; // Interrupt Pending
pub const NVIC_INT_CTRL_VEC_PEN_M: usize = 0x000FF000; // Interrupt Pending Vector Number
pub const NVIC_INT_CTRL_VEC_PEN_NMI: usize = 0x00002000; // NMI
pub const NVIC_INT_CTRL_VEC_PEN_HARD: usize = 0x00003000; // Hard fault
pub const NVIC_INT_CTRL_VEC_PEN_MEM: usize = 0x00004000; // Memory management fault
pub const NVIC_INT_CTRL_VEC_PEN_BUS: usize = 0x00005000; // Bus fault
pub const NVIC_INT_CTRL_VEC_PEN_USG: usize = 0x00006000; // Usage fault
pub const NVIC_INT_CTRL_VEC_PEN_SVC: usize = 0x0000B000; // SVCall
pub const NVIC_INT_CTRL_VEC_PEN_PNDSV: usize = 0x0000E000; // PendSV
pub const NVIC_INT_CTRL_VEC_PEN_TICK: usize = 0x0000F000; // SysTick
pub const NVIC_INT_CTRL_RET_BASE: usize = 0x00000800; // Return to Base
pub const NVIC_INT_CTRL_VEC_ACT_M: usize = 0x000000FF; // Interrupt Pending Vector Number
pub const NVIC_INT_CTRL_VEC_ACT_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_VTABLE register.
//
// *****************************************************************************
pub const NVIC_VTABLE_BASE: usize = 0x20000000; // Vector Table Base
pub const NVIC_VTABLE_OFFSET_M: usize = 0x1FFFFC00; // Vector Table Offset
pub const NVIC_VTABLE_OFFSET_S: usize = 10;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_APINT register.
//
// *****************************************************************************
pub const NVIC_APINT_VECTKEY_M: usize = 0xFFFF0000; // Register Key
pub const NVIC_APINT_VECTKEY: usize = 0x05FA0000; // Vector key
pub const NVIC_APINT_ENDIANESS: usize = 0x00008000; // Data Endianess
pub const NVIC_APINT_PRIGROUP_M: usize = 0x00000700; // Interrupt Priority Grouping
pub const NVIC_APINT_PRIGROUP_7_1: usize = 0x00000000; // Priority group 7.1 split
pub const NVIC_APINT_PRIGROUP_6_2: usize = 0x00000100; // Priority group 6.2 split
pub const NVIC_APINT_PRIGROUP_5_3: usize = 0x00000200; // Priority group 5.3 split
pub const NVIC_APINT_PRIGROUP_4_4: usize = 0x00000300; // Priority group 4.4 split
pub const NVIC_APINT_PRIGROUP_3_5: usize = 0x00000400; // Priority group 3.5 split
pub const NVIC_APINT_PRIGROUP_2_6: usize = 0x00000500; // Priority group 2.6 split
pub const NVIC_APINT_PRIGROUP_1_7: usize = 0x00000600; // Priority group 1.7 split
pub const NVIC_APINT_PRIGROUP_0_8: usize = 0x00000700; // Priority group 0.8 split
pub const NVIC_APINT_SYSRESETREQ: usize = 0x00000004; // System Reset Request
pub const NVIC_APINT_VECT_CLR_ACT: usize = 0x00000002; // Clear Active NMI / Fault
pub const NVIC_APINT_VECT_RESET: usize = 0x00000001; // System Reset

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_CTRL register.
//
// *****************************************************************************
pub const NVIC_SYS_CTRL_SEVONPEND: usize = 0x00000010; // Wake Up on Pending
pub const NVIC_SYS_CTRL_SLEEPDEEP: usize = 0x00000004; // Deep Sleep Enable
pub const NVIC_SYS_CTRL_SLEEPEXIT: usize = 0x00000002; // Sleep on ISR Exit

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CFG_CTRL register.
//
// *****************************************************************************
pub const NVIC_CFG_CTRL_STKALIGN: usize = 0x00000200; // Stack Alignment on Exception Entry
pub const NVIC_CFG_CTRL_BFHFNMIGN: usize = 0x00000100; // Ignore Bus Fault in NMI and Fault
pub const NVIC_CFG_CTRL_DIV0: usize = 0x00000010; // Trap on Divide by 0
pub const NVIC_CFG_CTRL_UNALIGNED: usize = 0x00000008; // Trap on Unaligned Access
pub const NVIC_CFG_CTRL_MAIN_PEND: usize = 0x00000002; // Allow Main Interrupt Trigger
pub const NVIC_CFG_CTRL_BASE_THR: usize = 0x00000001; // Thread State Control

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI1 register.
//
// *****************************************************************************
pub const NVIC_SYS_PRI1_USAGE_M: usize = 0x00E00000; // Usage Fault Priority
pub const NVIC_SYS_PRI1_BUS_M: usize = 0x0000E000; // Bus Fault Priority
pub const NVIC_SYS_PRI1_MEM_M: usize = 0x000000E0; // Memory Management Fault Priority
pub const NVIC_SYS_PRI1_USAGE_S: usize = 21;
pub const NVIC_SYS_PRI1_BUS_S: usize = 13;
pub const NVIC_SYS_PRI1_MEM_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI2 register.
//
// *****************************************************************************
pub const NVIC_SYS_PRI2_SVC_M: usize = 0xE0000000; // SVCall Priority
pub const NVIC_SYS_PRI2_SVC_S: usize = 29;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_PRI3 register.
//
// *****************************************************************************
pub const NVIC_SYS_PRI3_TICK_M: usize = 0xE0000000; // SysTick Exception Priority
pub const NVIC_SYS_PRI3_PENDSV_M: usize = 0x00E00000; // PendSV Priority
pub const NVIC_SYS_PRI3_DEBUG_M: usize = 0x000000E0; // Debug Priority
pub const NVIC_SYS_PRI3_TICK_S: usize = 29;
pub const NVIC_SYS_PRI3_PENDSV_S: usize = 21;
pub const NVIC_SYS_PRI3_DEBUG_S: usize = 5;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SYS_HND_CTRL
// register.
//
// *****************************************************************************
pub const NVIC_SYS_HND_CTRL_USAGE: usize = 0x00040000; // Usage Fault Enable
pub const NVIC_SYS_HND_CTRL_BUS: usize = 0x00020000; // Bus Fault Enable
pub const NVIC_SYS_HND_CTRL_MEM: usize = 0x00010000; // Memory Management Fault Enable
pub const NVIC_SYS_HND_CTRL_SVC: usize = 0x00008000; // SVC Call Pending
pub const NVIC_SYS_HND_CTRL_BUSP: usize = 0x00004000; // Bus Fault Pending
pub const NVIC_SYS_HND_CTRL_MEMP: usize = 0x00002000; // Memory Management Fault Pending
pub const NVIC_SYS_HND_CTRL_USAGEP: usize = 0x00001000; // Usage Fault Pending
pub const NVIC_SYS_HND_CTRL_TICK: usize = 0x00000800; // SysTick Exception Active
pub const NVIC_SYS_HND_CTRL_PNDSV: usize = 0x00000400; // PendSV Exception Active
pub const NVIC_SYS_HND_CTRL_MON: usize = 0x00000100; // Debug Monitor Active
pub const NVIC_SYS_HND_CTRL_SVCA: usize = 0x00000080; // SVC Call Active
pub const NVIC_SYS_HND_CTRL_USGA: usize = 0x00000008; // Usage Fault Active
pub const NVIC_SYS_HND_CTRL_BUSA: usize = 0x00000002; // Bus Fault Active
pub const NVIC_SYS_HND_CTRL_MEMA: usize = 0x00000001; // Memory Management Fault Active

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_STAT
// register.
//
// *****************************************************************************
pub const NVIC_FAULT_STAT_DIV0: usize = 0x02000000; // Divide-by-Zero Usage Fault
pub const NVIC_FAULT_STAT_UNALIGN: usize = 0x01000000; // Unaligned Access Usage Fault
pub const NVIC_FAULT_STAT_NOCP: usize = 0x00080000; // No Coprocessor Usage Fault
pub const NVIC_FAULT_STAT_INVPC: usize = 0x00040000; // Invalid PC Load Usage Fault
pub const NVIC_FAULT_STAT_INVSTAT: usize = 0x00020000; // Invalid State Usage Fault
pub const NVIC_FAULT_STAT_UNDEF: usize = 0x00010000; // Undefined Instruction Usage Fault
pub const NVIC_FAULT_STAT_BFARV: usize = 0x00008000; // Bus Fault Address Register Valid
pub const NVIC_FAULT_STAT_BLSPERR: usize = 0x00002000; // Bus Fault on Floating-Point Lazy State Preservation
pub const NVIC_FAULT_STAT_BSTKE: usize = 0x00001000; // Stack Bus Fault
pub const NVIC_FAULT_STAT_BUSTKE: usize = 0x00000800; // Unstack Bus Fault
pub const NVIC_FAULT_STAT_IMPRE: usize = 0x00000400; // Imprecise Data Bus Error
pub const NVIC_FAULT_STAT_PRECISE: usize = 0x00000200; // Precise Data Bus Error
pub const NVIC_FAULT_STAT_IBUS: usize = 0x00000100; // Instruction Bus Error
pub const NVIC_FAULT_STAT_MMARV: usize = 0x00000080; // Memory Management Fault Address Register Valid
pub const NVIC_FAULT_STAT_MLSPERR: usize = 0x00000020; // Memory Management Fault on Floating-Point Lazy State Preservation
pub const NVIC_FAULT_STAT_MSTKE: usize = 0x00000010; // Stack Access Violation
pub const NVIC_FAULT_STAT_MUSTKE: usize = 0x00000008; // Unstack Access Violation
pub const NVIC_FAULT_STAT_DERR: usize = 0x00000002; // Data Access Violation
pub const NVIC_FAULT_STAT_IERR: usize = 0x00000001; // Instruction Access Violation

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_HFAULT_STAT
// register.
//
// *****************************************************************************
pub const NVIC_HFAULT_STAT_DBG: usize = 0x80000000; // Debug Event
pub const NVIC_HFAULT_STAT_FORCED: usize = 0x40000000; // Forced Hard Fault
pub const NVIC_HFAULT_STAT_VECT: usize = 0x00000002; // Vector Table Read Fault

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DEBUG_STAT
// register.
//
// *****************************************************************************
pub const NVIC_DEBUG_STAT_EXTRNL: usize = 0x00000010; // EDBGRQ asserted
pub const NVIC_DEBUG_STAT_VCATCH: usize = 0x00000008; // Vector catch
pub const NVIC_DEBUG_STAT_DWTTRAP: usize = 0x00000004; // DWT match
pub const NVIC_DEBUG_STAT_BKPT: usize = 0x00000002; // Breakpoint instruction
pub const NVIC_DEBUG_STAT_HALTED: usize = 0x00000001; // Halt request

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MM_ADDR register.
//
// *****************************************************************************
pub const NVIC_MM_ADDR_M: usize = 0xFFFFFFFF; // Fault Address
pub const NVIC_MM_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FAULT_ADDR
// register.
//
// *****************************************************************************
pub const NVIC_FAULT_ADDR_M: usize = 0xFFFFFFFF; // Fault Address
pub const NVIC_FAULT_ADDR_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_CPAC register.
//
// *****************************************************************************
pub const NVIC_CPAC_CP11_M: usize = 0x00C00000; // CP11 Coprocessor Access Privilege
pub const NVIC_CPAC_CP11_DIS: usize = 0x00000000; // Access Denied
pub const NVIC_CPAC_CP11_PRIV: usize = 0x00400000; // Privileged Access Only
pub const NVIC_CPAC_CP11_FULL: usize = 0x00C00000; // Full Access
pub const NVIC_CPAC_CP10_M: usize = 0x00300000; // CP10 Coprocessor Access Privilege
pub const NVIC_CPAC_CP10_DIS: usize = 0x00000000; // Access Denied
pub const NVIC_CPAC_CP10_PRIV: usize = 0x00100000; // Privileged Access Only
pub const NVIC_CPAC_CP10_FULL: usize = 0x00300000; // Full Access

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_TYPE register.
//
// *****************************************************************************
pub const NVIC_MPU_TYPE_IREGION_M: usize = 0x00FF0000; // Number of I Regions
pub const NVIC_MPU_TYPE_DREGION_M: usize = 0x0000FF00; // Number of D Regions
pub const NVIC_MPU_TYPE_SEPARATE: usize = 0x00000001; // Separate or Unified MPU
pub const NVIC_MPU_TYPE_IREGION_S: usize = 16;
pub const NVIC_MPU_TYPE_DREGION_S: usize = 8;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_CTRL register.
//
// *****************************************************************************
pub const NVIC_MPU_CTRL_PRIVDEFEN: usize = 0x00000004; // MPU Default Region
pub const NVIC_MPU_CTRL_HFNMIENA: usize = 0x00000002; // MPU Enabled During Faults
pub const NVIC_MPU_CTRL_ENABLE: usize = 0x00000001; // MPU Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_NUMBER
// register.
//
// *****************************************************************************
pub const NVIC_MPU_NUMBER_M: usize = 0x00000007; // MPU Region to Access
pub const NVIC_MPU_NUMBER_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE register.
//
// *****************************************************************************
pub const NVIC_MPU_BASE_ADDR_M: usize = 0xFFFFFFE0; // Base Address Mask
pub const NVIC_MPU_BASE_VALID: usize = 0x00000010; // Region Number Valid
pub const NVIC_MPU_BASE_REGION_M: usize = 0x00000007; // Region Number
pub const NVIC_MPU_BASE_ADDR_S: usize = 5;
pub const NVIC_MPU_BASE_REGION_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR register.
//
// *****************************************************************************
pub const NVIC_MPU_ATTR_XN: usize = 0x10000000; // Instruction Access Disable
pub const NVIC_MPU_ATTR_AP_M: usize = 0x07000000; // Access Privilege
pub const NVIC_MPU_ATTR_TEX_M: usize = 0x00380000; // Type Extension Mask
pub const NVIC_MPU_ATTR_SHAREABLE: usize = 0x00040000; // Shareable
pub const NVIC_MPU_ATTR_CACHEABLE: usize = 0x00020000; // Cacheable
pub const NVIC_MPU_ATTR_BUFFRABLE: usize = 0x00010000; // Bufferable
pub const NVIC_MPU_ATTR_SRD_M: usize = 0x0000FF00; // Subregion Disable Bits
pub const NVIC_MPU_ATTR_SIZE_M: usize = 0x0000003E; // Region Size Mask
pub const NVIC_MPU_ATTR_ENABLE: usize = 0x00000001; // Region Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE1 register.
//
// *****************************************************************************
pub const NVIC_MPU_BASE1_ADDR_M: usize = 0xFFFFFFE0; // Base Address Mask
pub const NVIC_MPU_BASE1_VALID: usize = 0x00000010; // Region Number Valid
pub const NVIC_MPU_BASE1_REGION_M: usize = 0x00000007; // Region Number
pub const NVIC_MPU_BASE1_ADDR_S: usize = 5;
pub const NVIC_MPU_BASE1_REGION_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR1 register.
//
// *****************************************************************************
pub const NVIC_MPU_ATTR1_XN: usize = 0x10000000; // Instruction Access Disable
pub const NVIC_MPU_ATTR1_AP_M: usize = 0x07000000; // Access Privilege
pub const NVIC_MPU_ATTR1_TEX_M: usize = 0x00380000; // Type Extension Mask
pub const NVIC_MPU_ATTR1_SHAREABLE: usize = 0x00040000; // Shareable
pub const NVIC_MPU_ATTR1_CACHEABLE: usize = 0x00020000; // Cacheable
pub const NVIC_MPU_ATTR1_BUFFRABLE: usize = 0x00010000; // Bufferable
pub const NVIC_MPU_ATTR1_SRD_M: usize = 0x0000FF00; // Subregion Disable Bits
pub const NVIC_MPU_ATTR1_SIZE_M: usize = 0x0000003E; // Region Size Mask
pub const NVIC_MPU_ATTR1_ENABLE: usize = 0x00000001; // Region Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE2 register.
//
// *****************************************************************************
pub const NVIC_MPU_BASE2_ADDR_M: usize = 0xFFFFFFE0; // Base Address Mask
pub const NVIC_MPU_BASE2_VALID: usize = 0x00000010; // Region Number Valid
pub const NVIC_MPU_BASE2_REGION_M: usize = 0x00000007; // Region Number
pub const NVIC_MPU_BASE2_ADDR_S: usize = 5;
pub const NVIC_MPU_BASE2_REGION_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR2 register.
//
// *****************************************************************************
pub const NVIC_MPU_ATTR2_XN: usize = 0x10000000; // Instruction Access Disable
pub const NVIC_MPU_ATTR2_AP_M: usize = 0x07000000; // Access Privilege
pub const NVIC_MPU_ATTR2_TEX_M: usize = 0x00380000; // Type Extension Mask
pub const NVIC_MPU_ATTR2_SHAREABLE: usize = 0x00040000; // Shareable
pub const NVIC_MPU_ATTR2_CACHEABLE: usize = 0x00020000; // Cacheable
pub const NVIC_MPU_ATTR2_BUFFRABLE: usize = 0x00010000; // Bufferable
pub const NVIC_MPU_ATTR2_SRD_M: usize = 0x0000FF00; // Subregion Disable Bits
pub const NVIC_MPU_ATTR2_SIZE_M: usize = 0x0000003E; // Region Size Mask
pub const NVIC_MPU_ATTR2_ENABLE: usize = 0x00000001; // Region Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_BASE3 register.
//
// *****************************************************************************
pub const NVIC_MPU_BASE3_ADDR_M: usize = 0xFFFFFFE0; // Base Address Mask
pub const NVIC_MPU_BASE3_VALID: usize = 0x00000010; // Region Number Valid
pub const NVIC_MPU_BASE3_REGION_M: usize = 0x00000007; // Region Number
pub const NVIC_MPU_BASE3_ADDR_S: usize = 5;
pub const NVIC_MPU_BASE3_REGION_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_MPU_ATTR3 register.
//
// *****************************************************************************
pub const NVIC_MPU_ATTR3_XN: usize = 0x10000000; // Instruction Access Disable
pub const NVIC_MPU_ATTR3_AP_M: usize = 0x07000000; // Access Privilege
pub const NVIC_MPU_ATTR3_TEX_M: usize = 0x00380000; // Type Extension Mask
pub const NVIC_MPU_ATTR3_SHAREABLE: usize = 0x00040000; // Shareable
pub const NVIC_MPU_ATTR3_CACHEABLE: usize = 0x00020000; // Cacheable
pub const NVIC_MPU_ATTR3_BUFFRABLE: usize = 0x00010000; // Bufferable
pub const NVIC_MPU_ATTR3_SRD_M: usize = 0x0000FF00; // Subregion Disable Bits
pub const NVIC_MPU_ATTR3_SIZE_M: usize = 0x0000003E; // Region Size Mask
pub const NVIC_MPU_ATTR3_ENABLE: usize = 0x00000001; // Region Enable

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_CTRL register.
//
// *****************************************************************************
pub const NVIC_DBG_CTRL_DBGKEY_M: usize = 0xFFFF0000; // Debug key mask
pub const NVIC_DBG_CTRL_DBGKEY: usize = 0xA05F0000; // Debug key
pub const NVIC_DBG_CTRL_S_RESET_ST: usize = 0x02000000; // Core has reset since last read
pub const NVIC_DBG_CTRL_S_RETIRE_ST: usize = 0x01000000; // Core has executed insruction since last read
pub const NVIC_DBG_CTRL_S_LOCKUP: usize = 0x00080000; // Core is locked up
pub const NVIC_DBG_CTRL_S_SLEEP: usize = 0x00040000; // Core is sleeping
pub const NVIC_DBG_CTRL_S_HALT: usize = 0x00020000; // Core status on halt
pub const NVIC_DBG_CTRL_S_REGRDY: usize = 0x00010000; // Register read/write available
pub const NVIC_DBG_CTRL_C_SNAPSTALL: usize = 0x00000020; // Breaks a stalled load/store
pub const NVIC_DBG_CTRL_C_MASKINT: usize = 0x00000008; // Mask interrupts when stepping
pub const NVIC_DBG_CTRL_C_STEP: usize = 0x00000004; // Step the core
pub const NVIC_DBG_CTRL_C_HALT: usize = 0x00000002; // Halt the core
pub const NVIC_DBG_CTRL_C_DEBUGEN: usize = 0x00000001; // Enable debug

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_XFER register.
//
// *****************************************************************************
pub const NVIC_DBG_XFER_REG_WNR: usize = 0x00010000; // Write or not read
pub const NVIC_DBG_XFER_REG_SEL_M: usize = 0x0000001F; // Register
pub const NVIC_DBG_XFER_REG_R0: usize = 0x00000000; // Register R0
pub const NVIC_DBG_XFER_REG_R1: usize = 0x00000001; // Register R1
pub const NVIC_DBG_XFER_REG_R2: usize = 0x00000002; // Register R2
pub const NVIC_DBG_XFER_REG_R3: usize = 0x00000003; // Register R3
pub const NVIC_DBG_XFER_REG_R4: usize = 0x00000004; // Register R4
pub const NVIC_DBG_XFER_REG_R5: usize = 0x00000005; // Register R5
pub const NVIC_DBG_XFER_REG_R6: usize = 0x00000006; // Register R6
pub const NVIC_DBG_XFER_REG_R7: usize = 0x00000007; // Register R7
pub const NVIC_DBG_XFER_REG_R8: usize = 0x00000008; // Register R8
pub const NVIC_DBG_XFER_REG_R9: usize = 0x00000009; // Register R9
pub const NVIC_DBG_XFER_REG_R10: usize = 0x0000000A; // Register R10
pub const NVIC_DBG_XFER_REG_R11: usize = 0x0000000B; // Register R11
pub const NVIC_DBG_XFER_REG_R12: usize = 0x0000000C; // Register R12
pub const NVIC_DBG_XFER_REG_R13: usize = 0x0000000D; // Register R13
pub const NVIC_DBG_XFER_REG_R14: usize = 0x0000000E; // Register R14
pub const NVIC_DBG_XFER_REG_R15: usize = 0x0000000F; // Register R15
pub const NVIC_DBG_XFER_REG_FLAGS: usize = 0x00000010; // xPSR/Flags register
pub const NVIC_DBG_XFER_REG_MSP: usize = 0x00000011; // Main SP
pub const NVIC_DBG_XFER_REG_PSP: usize = 0x00000012; // Process SP
pub const NVIC_DBG_XFER_REG_DSP: usize = 0x00000013; // Deep SP
pub const NVIC_DBG_XFER_REG_CFBP: usize = 0x00000014; // Control/Fault/BasePri/PriMask

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_DATA register.
//
// *****************************************************************************
pub const NVIC_DBG_DATA_M: usize = 0xFFFFFFFF; // Data temporary cache
pub const NVIC_DBG_DATA_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_DBG_INT register.
//
// *****************************************************************************
pub const NVIC_DBG_INT_HARDERR: usize = 0x00000400; // Debug trap on hard fault
pub const NVIC_DBG_INT_INTERR: usize = 0x00000200; // Debug trap on interrupt errors
pub const NVIC_DBG_INT_BUSERR: usize = 0x00000100; // Debug trap on bus error
pub const NVIC_DBG_INT_STATERR: usize = 0x00000080; // Debug trap on usage fault state
pub const NVIC_DBG_INT_CHKERR: usize = 0x00000040; // Debug trap on usage fault check
pub const NVIC_DBG_INT_NOCPERR: usize = 0x00000020; // Debug trap on coprocessor error
pub const NVIC_DBG_INT_MMERR: usize = 0x00000010; // Debug trap on mem manage fault
pub const NVIC_DBG_INT_RESET: usize = 0x00000008; // Core reset status
pub const NVIC_DBG_INT_RSTPENDCLR: usize = 0x00000004; // Clear pending core reset
pub const NVIC_DBG_INT_RSTPENDING: usize = 0x00000002; // Core reset is pending
pub const NVIC_DBG_INT_RSTVCATCH: usize = 0x00000001; // Reset vector catch

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_SW_TRIG register.
//
// *****************************************************************************
pub const NVIC_SW_TRIG_INTID_M: usize = 0x000000FF; // Interrupt ID
pub const NVIC_SW_TRIG_INTID_S: usize = 0;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPCC register.
//
// *****************************************************************************
pub const NVIC_FPCC_ASPEN: usize = 0x80000000; // Automatic State Preservation Enable
pub const NVIC_FPCC_LSPEN: usize = 0x40000000; // Lazy State Preservation Enable
pub const NVIC_FPCC_MONRDY: usize = 0x00000100; // Monitor Ready
pub const NVIC_FPCC_BFRDY: usize = 0x00000040; // Bus Fault Ready
pub const NVIC_FPCC_MMRDY: usize = 0x00000020; // Memory Management Fault Ready
pub const NVIC_FPCC_HFRDY: usize = 0x00000010; // Hard Fault Ready
pub const NVIC_FPCC_THREAD: usize = 0x00000008; // Thread Mode
pub const NVIC_FPCC_USER: usize = 0x00000002; // User Privilege Level
pub const NVIC_FPCC_LSPACT: usize = 0x00000001; // Lazy State Preservation Active

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPCA register.
//
// *****************************************************************************
pub const NVIC_FPCA_ADDRESS_M: usize = 0xFFFFFFF8; // Address
pub const NVIC_FPCA_ADDRESS_S: usize = 3;

// *****************************************************************************
//
// The following are defines for the bit fields in the NVIC_FPDSC register.
//
// *****************************************************************************
pub const NVIC_FPDSC_AHP: usize = 0x04000000; // AHP Bit Default
pub const NVIC_FPDSC_DN: usize = 0x02000000; // DN Bit Default
pub const NVIC_FPDSC_FZ: usize = 0x01000000; // FZ Bit Default
pub const NVIC_FPDSC_RMODE_M: usize = 0x00C00000; // RMODE Bit Default
pub const NVIC_FPDSC_RMODE_RN: usize = 0x00000000; // Round to Nearest (RN) mode
pub const NVIC_FPDSC_RMODE_RP: usize = 0x00400000; // Round towards Plus Infinity (RP) mode
pub const NVIC_FPDSC_RMODE_RM: usize = 0x00800000; // Round towards Minus Infinity (RM) mode
pub const NVIC_FPDSC_RMODE_RZ: usize = 0x00C00000; // Round towards Zero (RZ) mode

// ****************************************************************************
//
// End Of File
//
// ****************************************************************************
