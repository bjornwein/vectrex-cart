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

// use cortex_m::asm;
use cortex_m::peripheral::Peripherals;

use core::ptr::{copy, null, null_mut, read_volatile, write_volatile};

mod stm32f407;
use stm32f407::*;

// ROM ptr shared with Interrupt handler.
// May point at any cart in flash, or even in RAM
static mut CART_MEMORY: *const u8 = null();
// ROM ptr to the loader binary
static mut LOADER_MEMORY: *const u8 = null();
// RAM ptr to memory extension and RPC command buffer
static mut EXTENDED_MEMORY: *mut u8 = null_mut();

const RAMFILEDATA: usize = 0xC002;

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

fn write_string_pointer(buf: &mut [u8], ptr: usize) -> &mut [u8] {
    buf[0] = (ptr >> 8) as u8;
    buf[1] = (ptr & 0xff) as u8;

    &mut buf[2..]
}

fn write_string<'a>(strings: &'a mut [u8], string: &str) -> &'a mut [u8] {
    let src = string.as_ptr() as *const u8;
    let dst = strings.as_mut_ptr() as *mut u8;
    unsafe { copy(src, dst, string.len()) }

    strings[string.len()] = 0x80; // String terminator
    &mut strings[string.len() + 1..]
}

/* Load ROMs into flash (at compile time), and
 * set up the multicart loader data table (at runtime).
 * TODO: the table can be constructed at compile time,
 * stored in flash and just copied onto the RAM area on
 * boot.
 * TODO: technically the table can be injected into the
 * multicart ROM at compile time and we don't need this
 * at all at runtime.
 * TODO: replace multicart protocol with a RAM area in the unused space?
 */
fn setup_carts(ram: &mut [u8]) {
    let loader = include_bytes!("carts/multicart.bin");
    unsafe { write_volatile(&mut CART_MEMORY, &loader[0] as *const u8) }
    unsafe { write_volatile(&mut LOADER_MEMORY, &loader[0] as *const u8) }

    let (mut pointers, mut strings) = ram[RAMFILEDATA - 0x8000..].split_at_mut(257 * 2);
    let mut string_ptr = RAMFILEDATA + 257 * 2;
    let mut cart_idx = 0;

    macro_rules! add_cart {
        ($name:expr, $file:expr) => {
            let rom = include_bytes!(concat!("carts/", $file)) as *const u8;
            unsafe { write_volatile(&mut CARTS[cart_idx], rom) }

            #[allow(unused_assignments)]
            {
                let string = $name;
                pointers = write_string_pointer(pointers, string_ptr);
                strings = write_string(strings, string);
                string_ptr += string.len() + 1;

                cart_idx += 1;
            }
        };
    };

    add_cart!("POLE POSITION (1982)", "Pole Position (1982).vec");
    add_cart!("POLAR RESCUE  (1983)", "Polar Rescue (1983).vec");
    add_cart!("RIP-OFF       (1982)", "Rip-Off (1982).vec");

    // String list end
    write_string_pointer(pointers, 0);
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

    // 18432 bytes RAM expansion
    let mut extension: [u8; 0xC800 - 0x8000] = [0; 0xC800 - 0x8000];
    unsafe { write_volatile(&mut EXTENDED_MEMORY, &mut extension[0] as *mut u8) }

    setup_carts(&mut extension);

    unsafe { board::NVIC::unmask(Interrupt::EXTI1) }

    // Get delay provider
    let mut delay = Delay::new(cp.SYST, clocks);

    loop {
        blue.set_high();
        green.set_high();
        delay.delay_ms(500_u16);
        green.set_low();
        delay.delay_ms(500_u16);
    }
}

#[interrupt]
fn EXTI1() {
    static mut BOOT_TRIGGERED: u8 = 0;

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

    // Check if this is a cart access (*ce == low)
    let pa = gpioa.idr.read();
    let we = (pa & 0b1000) == 0; // Read *WE
    let ce = (pa & 0b0100) == 0; // Read *CE
    match (ce, we, addr) {
        (true, false, addr) => {
            // Normal cart ROM read. Critical path
            unsafe {
                let rom = read_volatile(&CART_MEMORY);
                gpioe.odr1.write(*rom.offset(addr as isize));
                gpioe.otyper1.write(0b0000_0000); // 0: output push-pull
            };
        }
        (false, false, addr) if addr <= 0xC7FF - 0x8000 => {
            // RAM extension read
            unsafe {
                let ram = read_volatile(&EXTENDED_MEMORY);
                gpioe.odr1.write(*ram.offset(addr as isize));
                gpioe.otyper1.write(0b0000_0000); // 0: output push-pull
            };
        }
        (false, true, addr) if addr <= 0xC7FF - 0x8000 => {
            // RAM extension write
            unsafe {
                gpioe.otyper1.write(0b1111_1111); // 1: output open-drain
                gpioe.odr1.write(0xff);
            };

            // Reconfigure data pins for reading
            unsafe {
                gpioe.moder1.write(0b00000000_00000000); // 00: Input
                gpioe.pupdr1.write(0b01010101_01010100); // 01: Pull-up
            }

            unsafe { gpiod.bsrr.write(0b10000000_00000000_00000000_00000000) }

            let ram = unsafe { read_volatile(&EXTENDED_MEMORY) };
            let byte = (gpioe.idr.read() >> 8) as u8;
            if addr == 0xC7FF - 0x8000 {
                match byte {
                    // RPC command
                    0x01 => unsafe {
                        // Switch to the selected cart
                        let param = *ram.offset((0xC7FE - 0x8000) as isize);
                        write_volatile(&mut CART_MEMORY, CARTS[param as usize]);
                    },
                    _ => {}
                }
            }
            unsafe { *ram.offset(addr as isize) = byte }

            unsafe {
                gpioe.moder1.write(0b01010101_01010101); // 01: General Purpose output mode
                gpioe.pupdr1.write(0b00000000_00000000); // 00: No pull-up/pull-down
            }
        }
        (false, false, 0x7000) => {
            // "Cold_Boot" read, used to detect reboots
            unsafe {
                gpioe.otyper1.write(0b1111_1111); // 1: output open-drain
                gpioe.odr1.write(0xff);
            };

            let rom = unsafe { core::ptr::read_volatile(&CART_MEMORY) };
            let loader = unsafe { core::ptr::read_volatile(&LOADER_MEMORY) };
            if rom == loader {
                // Ignore bootups from multicart loader
                *BOOT_TRIGGERED = 0;
            } else if *BOOT_TRIGGERED > 0 {
                // Turn off blue led for fun and profit
                unsafe { gpiod.bsrr.write(0b10000000_00000000_00000000_00000000) }

                // Swap in multicart loader under the hood
                unsafe { core::ptr::write_volatile(&mut CART_MEMORY, loader) }
                *BOOT_TRIGGERED = 0;
            } else {
                *BOOT_TRIGGERED = 1;
            }
        }
        _ => {
            // Anything else
            unsafe {
                gpioe.otyper1.write(0b1111_1111); // 1: output open-drain
                gpioe.odr1.write(0xff);
            };
        }
    };
}
