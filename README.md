# Rust drivers for Cortex-M4F based Texas Instruments LM4F120 (and relatives)

The Texas Instruments LM4F120 is a Cortex-M4F based microcontroller. It's available on the [Texas Instruments Stellaris Launchpad](http://www.ti.com/tool/ek-lm4f120xl) (not to be confused with the older MSP430 Launchpad). TI cancelled the Stellaris/LM4F range not long after it came out and replaced it with Tiva-C/TM4C range. The [Tiva-C Launchpad TM4C123G-XL](http://www.ti.com/tool/ek-tm4c123gxl) is almost exactly the same as a Stellaris Launchpad and should be software compatible. The Ethernet-enabled [TM4C1294 Connected Launchpad](http://www.ti.com/tool/ek-tm4c1294xl) is not supported.

For a bare metal example program written in Rust (https://rust-lang.org) for the Stellaris Launchpad using this crate, see [launchpad-rs](https://github.com/thejpster/launchpad-rs).
