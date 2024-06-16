#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m::peripheral::{syst, Peripherals};
use cortex_m_rt::{entry, exception};
use mb9bf61xt;
use rtt_target::{rprintln, rtt_init_print};

// SysTick timer interval: 1s (based on 4MHz CPU clock).
const SYSTICK_PERIOD: u32 = 4_000_000;

fn disable_wdg() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let wdg = p.HWWDT;

    // Unlock the watchdog registers.
    wdg.wdg_lck().write(|w| unsafe { w.bits(0x1ACCE551) });
    wdg.wdg_lck().write(|w| unsafe { w.bits(0xE5331AAE) });

    // Disable the hardware watchdog timer.
    wdg.wdg_ctl().modify(|_, w| w.inten().clear_bit());
}

fn initialize_gpio() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // Set to GPIO mode.
    gpio.pfrf().write(|w| w.pf3().clear_bit());

    // Set to open-drain mode.
    gpio.pzrf().write(|w| w.pf3().set_bit());

    // Set to output.
    gpio.ddrf().write(|w| w.pf3().set_bit());

    // Set to low level.
    gpio.pdorf().write(|w| w.pf3().clear_bit());
}

fn initialize_systick() {
    let cp = unsafe { Peripherals::steal() };
    let mut systick = cp.SYST;

    systick.set_clock_source(syst::SystClkSource::Core);
    systick.set_reload(SYSTICK_PERIOD);
    systick.clear_current();
    systick.enable_counter();
    systick.enable_interrupt();
}

#[entry]
fn main() -> ! {
    // Disable hardware watchdog.
    disable_wdg();

    // Initialze RTT debug message interface.
    rtt_init_print!();

    // Initialize the GPIO driving the external LED.
    rprintln!("INFO: Initializing gpio.");
    initialize_gpio();

    // Initialize SysTick.
    rprintln!("INFO: Initializing systick.");
    initialize_systick();

    loop {}
}

// SysTick exception handler.
// NOTE: not sure why but this empirically needs to be defined after main.
#[exception]
fn SysTick() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // Invert the output.
    let current_pf3_val = gpio.pdorf().read().pf3().bit_is_set();
    gpio.pdorf().write(|w| w.pf3().bit(!current_pf3_val));
}
