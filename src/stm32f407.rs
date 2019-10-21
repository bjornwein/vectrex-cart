// Copyright 2019 Björn Weinehall
//
// Licensed under the Apache License, Version 2.0, <LICENSE-APACHE or
// http://apache.org/licenses/LICENSE-2.0> or the MIT license <LICENSE-MIT or
// http://opensource.org/licenses/MIT>, at your option. This file may not be
// copied, modified, or distributed except according to those terms.

/*
 * Custom wrappers for some CPU registers
 */

use volatile_register::{RO, RW, WO};

#[repr(C)]
pub struct Syscfg {
    _fsmc: u32,
    _pmc: u32,
    pub exticr1: RW<u16>,
    _reserved: u16,
}
const SYSCFG_ADDR: u32 = 0x4001_3800;

impl Syscfg {
    pub fn syscfg() -> &'static mut Syscfg {
        unsafe { &mut *(SYSCFG_ADDR as *mut Syscfg) }
    }
}

#[repr(C)]
pub struct Exti {
    // See 12.3 EXTI registers
    pub imr: RW<u32>,
    _emr: u32,
    pub rtsr: RW<u32>, // rising trigger selection register
    pub ftsr: RW<u32>, // falling trigger selection register
    _swier: u32,
    pub pr: RW<u32>, // pending register
}
const EXTI_ADDR: u32 = 0x4001_3C00;

impl Exti {
    pub fn exti() -> &'static mut Exti {
        unsafe { &mut *(EXTI_ADDR as *mut Exti) }
    }
}

#[repr(C)]
pub struct Gpio {
    // See 8.4.11 "GPIO register map"
    pub moder0: RW<u16>, // offset: 0x00
    pub moder1: RW<u16>,
    pub otyper0: RW<u8>, // offset: 0x04
    pub otyper1: RW<u8>,
    _reserved1: u16,
    pub ospeedr0: RW<u16>, // offset: 0x08
    pub ospeedr1: RW<u16>,
    pub pupdr0: RW<u16>, // offset: 0x0c
    pub pupdr1: RW<u16>,
    pub idr: RO<u16>, // offset: 0x10
    _reserved2: u16,
    pub odr0: WO<u8>, // offset: 0x14
    pub odr1: WO<u8>,
    _reserved3: u16,
    pub bsrr: WO<u32>, // offset: 0x18
    _lckr: u32,        // offset 0x1c
    _afrl: u32,        // offset 0x20
    _afrh: u32,        // offset 0x24
}

impl Gpio {
    pub fn gpioa() -> &'static mut Gpio {
        unsafe { &mut *(0x4002_0000 as *mut Gpio) }
    }
    pub fn gpiob() -> &'static mut Gpio {
        unsafe { &mut *(0x4002_0400 as *mut Gpio) }
    }
    // pub fn gpioc() -> &'static mut Gpio { unsafe { &mut *(0x4002_0800 as *mut Gpio) } }
    pub fn gpiod() -> &'static mut Gpio {
        unsafe { &mut *(0x4002_0C00 as *mut Gpio) }
    }
    pub fn gpioe() -> &'static mut Gpio {
        unsafe { &mut *(0x4002_1000 as *mut Gpio) }
    }
    // pub fn gpiof() -> &'static mut Gpio { unsafe { &mut *(0x4002_1400 as *mut Gpio) } }
    // pub fn gpiog() -> &'static mut Gpio { unsafe { &mut *(0x4002_1800 as *mut Gpio) } }
    // pub fn gpioh() -> &'static mut Gpio { unsafe { &mut *(0x4002_1C00 as *mut Gpio) } }
    // pub fn gpioi() -> &'static mut Gpio { unsafe { &mut *(0x4002_2000 as *mut Gpio) } }
    // pub fn gpioj() -> &'static mut Gpio { unsafe { &mut *(0x4002_2400 as *mut Gpio) } }
}

/*
macro_rules! rpt {
    ( 0; $x:block ) => {};
    ( 1; $x:block ) => {
        $x;
    };
    ( 2; $x:block ) => {
        $x;
        $x;
    };
    ( 3; $x:block ) => {
        rpt!(2; $x);
        $x;
    };
    ( 4; $x:block) => {
        rpt!(2; $x);
        rpt!(2; $x);
    };
    ( 5; $x:block ) => {
        rpt!(4; $x);
        $x;
    };
    ( 6 ; $x:block) => {
        rpt!(4; $x);
        rpt!(2; $x);
    };
    ( 7 ; $x:block) => {
        rpt!(6; $x);
        $x;
    };
    ( 8 ; $x:block) => {
        rpt!(6; $x);
        rpt!(2; $x);
    };
    ( 9; $x:block ) => {
        rpt!(8; $x);
        $x;
    };
    ( 10; $x:block ) => {
        rpt!(8; $x);
        rpt!(2; $x);
    };

    // Repeat $block n*m times (0 <= n <= 10, 0 <= m <= 10)
    ( $n:tt; $m:tt; $block:block ) => {
        rpt!($n; { rpt!($m; $block); });
    };
}

macro_rules! rpt_nop {
    // Nop n times (0 <= n <= 10)
    ( $n:tt ) => {
        rpt!($n; { asm::nop() })
    };
    // Nop n*m times (0 <= n <= 10, 0 <= m <= 10)
    ( $n:tt; $m:tt ) => {
        rpt!($n; $m; { asm::nop() })
    };
}
*/
