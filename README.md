# vectrex-cart
A programmable Vectrex multicart based on ARM SOC

- [ ] TODO: beautiful image of cart

This hack uses a cheap-ish STM32F407G development kit (STM32F407G-DISC1) to emulate vectrex carts. The carts are loaded from the internal flash of the STM32F4.

## Hardware

I used an official STM32F407G-DISC1 board from STM. It is easy to source, and has a built-in ST-link programming / debug interface which is nice. There are smaller and cheaper development kits out there that may work as well.

The exact development board used shouldn't matter much. However, if you don't use a STM32F407G-DISC1,
make sure the used GPIOs aren't connected to board components such as LEDs, USB, IMU, etc...

(The plentiful on-board stuff on the STM32F407G-DISC1 is the reason why the address lines are such a mess)

For the physical Vectrex interface I simply desoldered the ROM from an old cart and replaced it with wires to the corresponding STM32F GPIO pins. This doesn't look good, but it works and requires only a minimum of soldering. 

### Pin mapping
Vectrex | STM32F407 | Note
--------|-----------|-----
*OE | PA1 | "inverse CPU E clock". Triggers cart emulation interrupt once every cycle.
*CE | PA2 |        "inverse Chip Enable". A15 on CPU? Only push data when low.
*WE | PA3 |        "inverse Write Enable". Not used (yet)
A0  | PD0
A1  | PD1
A2  | PD2
A3  | PD3
A4  | PE4
A5  | PE5
A6  | PD6
A7  | PD7
A8  | PD8
A9  | PD9
A10 | PD10
A11 | PD11
A12 | PB12
A13 | PB13
A14 | PB14
X   | PB15 | "Software controlled line" - Not used (yet)
D0  | PE8
...  | ...
D7  | PE15
GND | GND
+5V | +5V | Powers the board. Just make sure to unplug the USB cable!
*HALT |  -
*CART |  -
*NMI  |  -
*IRQ  |  -

## Software

To build the ARM code you need an Cortex-M4F rust toolchain. See the Embedded Rust Book for installation instructions: https://rust-embedded.github.io/book/intro/tooling.html

The main work is done by an interrupt routing that incidentally has just about good enough timing to handle the data. There should be a few cycles left outside to add more stuff later.

The code is very far from idiomatic rust, but I didn't feel the need for safe abstractions for this little project. Suggestions on improvements are welcome.

I chose to use Jeroen Domburg's *Extreme Multicart* protocol and multicart loader ROM, so there is a simple protocol implemented to pass data back

## References
* STM32F407 Reference Manual - *ST Microelectronics* - https://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
* STM32F407G-DISC1 User Manual - *ST Microelectronics* - https://www.st.com/content/ccc/resource/technical/document/user_manual/70/fe/4a/3f/e7/e1/4f/7d/DM00039084.pdf/files/DM00039084.pdf/jcr:content/translations/en.DM00039084.pdf
* How to construct a Vectrex Multicart (Very simple) - *Tursi (M. Brent)* - http://www.harmlesslion.com/text/vectrex_multicart.htm
* Emulating a GameBoy Cartridge with an STM32F4 - *Dhole* - http://dhole.github.io/post/gameboy_cartridge_emu_1/
* Extreme Vectrex multicart - *Jeroen "Sprite" Domburg* - https://spritesmods.com/?art=veccart&page=1

## License

Licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.

### 3rd party components

Jeroen Domburg's [multicart/multicart.asm] is licensed under GNU LGPL v3. See that file for more information.

