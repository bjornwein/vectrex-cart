// Copyright 2019 Björn Weinehall
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.
#![no_std]
#![no_main]

// cargo objdump --release -- -d -S --disassemble-functions=EXTI1 target/thumbv7em-none-eabihf/release/vectrex-cart

// pick a panicking behavior
// extern crate panic_halt; // you can put a breakpoint on `rust_begin_unwind` to catch panics
// extern crate panic_abort; // requires nightly
// extern crate panic_itm; // logs messages over ITM; requires ITM support
extern crate panic_semihosting; // logs messages to the host stderr; requires a debugger

extern crate embedded_hal as hal;
extern crate stm32f407g_disc as board;

use cortex_m_rt::entry;
// use cortex_m_semihosting::hprintln;

use board::gpio::Speed;
use board::hal::delay::Delay;
use board::hal::prelude::*;
use board::hal::stm32::{self, interrupt, Interrupt};

use cortex_m::asm;
use cortex_m::peripheral::Peripherals;

mod stm32f407;
use stm32f407::*;

// ROM ptr shared with Interrupt handler. May point at either RAM or flash
static mut CART_MEMORY: *const u8 = 0 as *const u8;

// Flash addresses of available carts
static mut CARTS: [*const u8; 256] = [0 as *const u8; 256];

unsafe fn setup_gpio_pins() {
    let gpioa = Gpio::gpioa();
    let gpiob = Gpio::gpiob();
    let gpiod = Gpio::gpiod();
    let gpioe = Gpio::gpioe();

    // Configure data output pins PE8-PE15 (bypass the HAL, though)
    gpioe.otyper1.write(0b1111_1111); // 1: output open-drain
    gpioe.ospeedr1.write(0b11111111_11111111); // 11: Very High Speed
    gpioe.pupdr1.write(0b00000000_00000000); // 00: No pull-up/pull-down
    gpioe.odr1.write(0xff);
    //gpioe.pupdr1.write(0b01010101_01010101); // 01: Pull-up
    //gpioe.pupdr1.write(0b10101010_10101010); // 10: Pull-down
    gpioe.moder1.write(0b01010101_01010101); // 01: General Purpose output mode

    // Configure address input pins
    gpioa
        .moder0
        .modify(|v| (v & 0b11111111_00000011) | 0b00000000_00000000); // 00: Input
    gpioa
        .pupdr0
        .modify(|v| (v & 0b11111111_00000011) | 0b00000000_01010100); // 01: Pull-up

    gpiob
        .moder1
        .modify(|v| (v & 0b00000000_11111111) | 0b00000000_00000000); // 00: Input
    gpiob
        .pupdr1
        .modify(|v| (v & 0b00000000_11111111) | 0b01010101_00000000); // 01: Pull-up

    gpiod
        .moder0
        .modify(|v| (v & 0b00001111_00000000) | 0b00000000_00000000); // 00: Input
    gpiod
        .pupdr0
        .modify(|v| (v & 0b00001111_00000000) | 0b01010000_01010101); // 01: Pull-up

    gpiod
        .moder1
        .modify(|v| (v & 0b11111111_00000000) | 0b00000000_00000000); // 00: Input
    gpiod
        .pupdr1
        .modify(|v| (v & 0b11111111_00000000) | 0b00000000_01010101); // 01: Pull-up

    gpioe
        .moder0
        .modify(|v| (v & 0b11110000_11111111) | 0b00000000_00000000); // 00: Input
    gpioe
        .pupdr0
        .modify(|v| (v & 0b11110000_11111111) | 0b00000101_00000000); // 01: Pull-up

    // Set up GPIOA Interrupt on PA1 (/OE)
    let syscfg = Syscfg::syscfg();
    let exti = Exti::exti();
    syscfg.exticr1.modify(|v| (v & !(0xf << 4)) | (0b0000 << 4));

    // Trigger on rising edge
    let line_1_bit = 1 << 1;
    exti.rtsr.modify(|v| v | line_1_bit);
    exti.ftsr.modify(|v| v & !line_1_bit);
    // Enable interrupt
    exti.imr.modify(|v| v | line_1_bit);
}

#[entry]
fn main() -> ! {
    // Configure LED outputs, using HAL
    let p = stm32::Peripherals::take().unwrap();
    let cp = Peripherals::take().unwrap();
    let gpiod = p.GPIOD.split();
    let mut green = gpiod
        .pd12
        .into_push_pull_output()
        .set_speed(Speed::VeryHigh);
    let mut blue = gpiod
        .pd15
        .into_push_pull_output()
        .set_speed(Speed::VeryHigh);

    // Enable GPIO clocks (Needed when bypassing the HAL)
    use crate::stm32::RCC;
    let rcc = unsafe { &(*RCC::ptr()) };
    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
    rcc.ahb1enr.modify(|_, w| w.gpioben().set_bit());
    rcc.ahb1enr.modify(|_, w| w.gpioden().set_bit());
    rcc.ahb1enr.modify(|_, w| w.gpioeen().set_bit());

    unsafe { setup_gpio_pins() }

    // Constrain clock registers
    let rcc = p.RCC.constrain();

    // Helps speed
    let flash = p.FLASH;
    flash.acr.modify(|_, w| w.prften().set_bit());
    flash.acr.modify(|_, w| w.dcen().set_bit());
    flash.acr.modify(|_, w| w.icen().set_bit());

    // Configure clock to 168 MHz (i.e. the maximum) and freeze it
    let clocks = rcc
        .cfgr
        //.use_hse(8.mhz())
        .sysclk(168.mhz())
        //.pclk1(42.mhz())
        //.pclk2(84.mhz())
        .freeze();

    let mut loader: [u8; 32768] = [0; 32768];
    let loader_rom = include_bytes!("carts/multicart.bin");

    unsafe {
        core::ptr::copy(
            loader_rom as *const u8,
            &mut loader[0] as *mut u8,
            loader_rom.len(),
        );
        core::ptr::write_volatile(&mut CART_MEMORY, &loader[0] as *const u8);

        core::ptr::write_volatile(
            &mut CARTS[0],
            include_bytes!("carts/Pole Position (1982).vec") as *const u8,
        );
    }

    unsafe { board::NVIC::unmask(Interrupt::EXTI1) }

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);
    blue.set_high();

    loop {
        green.set_high();
        delay.delay_ms(100_u16);
        green.set_low();
        delay.delay_ms(100_u16);
    }
}

static mut write_param: u8 = 0; // FIXME there's an idiomatic way for this in embedded rust

#[interrupt]
fn EXTI1() {
    let gpioa = Gpio::gpioa();
    let gpiob = Gpio::gpiob();
    let gpiod = Gpio::gpiod();
    let gpioe = Gpio::gpioe();
    let exti = Exti::exti();

    // unsafe { gpioe.odr1.write(0b11111111) } // Indicate interrupt start

    // Clear interrupt pending bit
    unsafe { exti.pr.write(0b00000000_00000000_00000000_00000010) }

    // Decode address lines (mixed between multiple GPIOs
    const AMASK_PD: u16 = 0b0000_1111_1100_1111; /* PD0-3 + PD6-11 */
    const AMASK_PE: u16 = 0b0000_0000_0011_0000; /* PE4-5 */
    const AMASK_PB: u16 = 0b0111_0000_0000_0000; /* PB12-14 */
    let addr: u16 = (gpiod.idr.read() & AMASK_PD)
        | (gpioe.idr.read() & AMASK_PE)
        | (gpiob.idr.read() & AMASK_PB);

    let pa = gpioa.idr.read();

    // Check if this is a cart access (ce == high, ce_inv == low)
    let ce = (pa & 0b0100) == 0; // Read *CE
    if !ce {
        unsafe {
            gpioe.otyper1.write(0b1111_1111); // 1: output open-drain
            gpioe.odr1.write(0xff);
        };
        return;
    }

    let we = (pa & 0b1000) == 0; // Read *WE
    if we {
        // Reconfigure data pins for reading
        unsafe {
            gpioe.moder1.write(0b00000000_00000000); // 00: Input
            gpioe.pupdr1.write(0b01010101_01010100); // 01: Pull-up
        }

        let v = (gpioe.idr.read() >> 8) as u8;
        if addr == 0x7ffe {
            unsafe { write_param = v };
        } else if addr == 0x7fff {
            if v == 1 {
                unsafe { core::ptr::write_volatile(&mut CART_MEMORY, CARTS[0]) }
            }
        }

        unsafe {
            gpioe.moder1.write(0b01010101_01010101); // 01: General Purpose output mode
            gpioe.pupdr1.write(0b00000000_00000000); // 00: No pull-up/pull-down
        }
        return;
    }

    // Load the byte from ram array
    let v = unsafe {
        let rom = core::ptr::read_volatile(&CART_MEMORY);
        *rom.offset(addr as isize)
    };

    /* At this point we're ~320ns after /OE high. Need > 333ns (or wait for /CE low) */
    // rpt_nop!(1);

    unsafe {
        gpioe.odr1.write(v);
        gpioe.otyper1.write(0b0000_0000); // 0: output push-pull
    };
}
