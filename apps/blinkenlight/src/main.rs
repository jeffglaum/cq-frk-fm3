#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use mb9bf61xt;

#[entry]
fn main() -> ! {

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // Set to GPIO mode.
    gpio.pfrf().write(|w| {w.pf3().clear_bit()});

    // Set to open-drain mode.
    gpio.pzrf().write(|w| {w.pf3().set_bit()});

    // Set to output.
    gpio.ddrf().write(|w| {w.pf3().set_bit()});

    // Set to low level.
    gpio.pdorf().write(|w| {w.pf3().clear_bit()});

    loop {}
}
