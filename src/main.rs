#![no_std]
#![no_main]

/*
 * Copyright Björn Weinehall 2019
 *
 * References:
 *
 * STM32F407 Reference Manual
 *   ST Microelectronics
 *   https://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
 *
 * STM32F407G-DISC1 User Manual
 *   ST Microelectronics
 *   https://www.st.com/content/ccc/resource/technical/document/user_manual/70/fe/4a/3f/e7/e1/4f/7d/DM00039084.pdf/files/DM00039084.pdf/jcr:content/translations/en.DM00039084.pdf
 *
 * How to construct a Vectrex Multicart (Very simple)
 *   Tursi (M. Brent)
 *   http://www.harmlesslion.com/text/vectrex_multicart.htm
 *
 * Emulating a GameBoy Cartridge with an STM32F4
 *   Dhole
 *   http://dhole.github.io/post/gameboy_cartridge_emu_1/
 *
 * Extreme Vectrex multicart
 *   Jeroen "Sprite" Domburg
 *   https://spritesmods.com/?art=veccart&page=1
 */

/*
 * Which development is board used shouldn't matter much. However,
 * make sure the used GPIOs aren't connected to board components
 * such as LEDs, USB, IMU, etc...
 *
 * (The plentiful on-board stuff on the STM32F407G-DISC1 is the reason
 *  why the address lines are such a mess)
 *
 * Vectrex  STM32F407   Note
 * *OE      PA1         "inverse CPU E clock". Triggers cart emulation interrupt once every cycle.
 * *CE      PA2         "inverse Chip Enable". A15 on CPU? Only push data when low.
 * *WE      PA3         "inverse Write Enable". Not used (yet)
 *
 * A0       PD0
 * A1       PD1
 * A2       PD2
 * A3       PD3
 * A4       PE4
 * A5       PE5
 * A6       PD6
 * A7       PD7
 * A8       PD8
 * A9       PD9
 * A10      PD10
 * A11      PD11
 * A12      PB12
 * A13      PB13
 * A14      PB14
 * X        PB15       "Software controlled line" - Not used (yet)
 *
 * D0       PE8
 * ..       ..
 * D7       PE15
 *
 * GND      GND
 * +5V      +5V        Powers the board. Just make sure to unplug the USB cable!
 *
 * *HALT    -
 * *CART    -
 * *NMI     -
 * *IRQ     -
 */

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

use core::sync::atomic::{AtomicPtr, Ordering};

mod stm32f407;
use stm32f407::*;

const ROM: &'static [u8; 8192] = include_bytes!("Pole Position (1982).vec");

static CART_MEMORY: AtomicPtr<u8> = AtomicPtr::new(0 as *mut u8);

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

    let mut cart: [u8; 65536] = [0; 65536];

    // Load cart from flash to RAM
    unsafe {
        core::ptr::copy(ROM as *const u8, &mut cart[0] as *mut u8, ROM.len());
        CART_MEMORY.store(&mut cart[0] as *mut u8, Ordering::Release);
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

    // Check if this is a cart access (ce == high, ce_inv == low)
    let ce = (gpioa.idr.read() & 0b0100) != 0; // Read /CE
    if ce {
        unsafe { gpioe.otyper1.write(0b1111_1111) } // 1: output open-drain
        unsafe { gpioe.odr1.write(0xff) }
        return;
    }

    // Decode address lines (mixed between multiple GPIOs
    const AMASK_PD: u16 = 0b0000_1111_1100_1111; /* PD0-3 + PD6-11 */
    const AMASK_PE: u16 = 0b0000_0000_0011_0000; /* PE4-5 */
    const AMASK_PB: u16 = 0b0111_0000_0000_0000; /* PB12-14 */
    let addr: u16 = (gpiod.idr.read() & AMASK_PD)
        | (gpioe.idr.read() & AMASK_PE)
        | (gpiob.idr.read() & AMASK_PB);

    // Load the byte from ram array
    let cart = CART_MEMORY.load(Ordering::Acquire);
    let v = unsafe { *cart.offset(addr as isize) };

    /* At this point we're ~320ns after /OE high. Need > 333ns (or wait for /CE low) */
    // rpt_nop!(1);

    unsafe { gpioe.odr1.write(v) }
    unsafe { gpioe.otyper1.write(0b0000_0000) } // 0: output push-pull
}
