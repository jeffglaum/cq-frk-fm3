#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::{entry, exception};
use cortex_m::peripheral::{syst, Peripherals};
use mb9bf61xt;

// SysTick timer interval (in ms).
const SYSTICK_PERIOD: u32 = 1_000;

// SysTick exception handler.
#[exception]
fn SysTick() {

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // Invert the output.
    let current_pf3_val  = gpio.pdorf().read().pf3().bit_is_set();
    gpio.pdorf().write(|w| {w.pf3().bit(! current_pf3_val)});

    // Reset the SysTick timer.
    let cp = unsafe { Peripherals::steal() };
    let mut systick = cp.SYST;

    systick.set_reload(SYSTICK_PERIOD);
}

fn initialize_gpio() {

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
}

fn initialize_systick() {

    let cp = unsafe { Peripherals::steal() };
    let mut systick = cp.SYST;

    systick.set_clock_source(syst::SystClkSource::Core);
    systick.set_reload(SYSTICK_PERIOD);
    systick.clear_current();
    systick.enable_interrupt();
    systick.enable_counter();
}

#[entry]
fn main() -> ! {

    // Initialize the GPIO driving the external LED.
    initialize_gpio();

    // Initialize SysTick.
    initialize_systick();

    loop {}
}
