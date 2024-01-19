#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use mb9bf61xt;

mod serial;
pub use crate::serial::Mb9bf61xtUart;

fn disable_wdg() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let wdg = p.HWWDT;

    // Unlock the watchdog registers.
    wdg.wdg_lck().write(|w| unsafe { w.bits(0x1ACCE551) });
    wdg.wdg_lck().write(|w| unsafe { w.bits(0xE5331AAE) });

    // Disable the hardware watchdog timer.
    wdg.wdg_ctl().modify(|_, w| w.inten().clear_bit());
}

fn init_clock() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let clock = p.CRG;

    // Force-clear all possible oscillization stabilization wait interrupt causes (main, sub, and PLL).
    clock.int_clr().write(|w| unsafe { w.bits(0x7) });

    // Set the main oscillator stabilization wait time (use default ~500ns).
    clock.csw_tmr().write(|w| unsafe { w.bits(0) });

    // Set the oscillation stabilization wait interrupt.
    clock.int_enr().modify(|_, w| w.mcse().set_bit());

    // Enable the main (external) clock oscillator.
    clock.scm_ctl().modify(|_, w| w.mosce().set_bit());

    // Checkk the main oscillator stabile bit.
    // TODO: anything to be done with a time-out here?
    while clock.scm_str().read().mordy() != true {}

    // Configure the master clock to use the main (external clock).
    clock.scm_ctl().modify(|_, w| unsafe { w.rcs().bits(0x1) });

    // Wait for master clock selection to take effect.
    // TODO: anything to be done with a time-out here?
    while clock.scm_str().read().rcm() != 0x1 {}
}

fn init_pins() {
    // FM3 (MB9BF61xT) UART pin options:
    // NOTE: UART channels 0-3 don't offer a FIFO, channels 4-7 do have a FIFO.
    //
    // * SIN0_0:P21, SIN1_0:P56, SIN2_0:P72, SIN3_0:P75, SIN4_0:PD2, SIN5_0:P60, SIN6_0:P53, SIN7_0:P59
    // * SOT0_0:P22, SOT1_0:P57, SOT2_0:P73, SOT3_0:P76, SOT4_0:PD1, SOT5_0:P61, SOT6_0:P54, SOT7_0:P5A
    //
    // This code uses UART channel 4.  On the CQ-FRK-FM3 board this uses connector CN2's pin 14 (SOT 4_2,
    // GPIO P06) and pin 11 (SIN 4_2, GPIO P05).
    //

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // Port Function Setting register (PFRx) - configure channel 4's UART pins to be a peripheral (not GPIO) function.
    // PFR0 controls GPIO pins P00 through P0F.  Configure P05 (SIN 4_2) and P06 (SOT 4_2) as peripheral pins.
    gpio.pfr0().modify(|_, w| w.p05().set_bit());
    gpio.pfr0().modify(|_, w| w.p06().set_bit());

    // Extended Pin Function Setting register (EPFR0x) - configure UART pin functions.
    // EPFR08 controls UART channels 4-7.
    let mut current_epfr08 = gpio.epfr08().read().bits();
    current_epfr08 &= !0x3FF; // Mask off channel 4 bits.
    current_epfr08 |= 0x0FC; // No RTS, CTS 4_2, SIN 4_2, SOT 4_2, no SCK.
    gpio.epfr08().write(|w| unsafe { w.bits(current_epfr08) });
}

fn print_banner() {
    println!("");
    println!("     _    ____ __  __  ");
    println!("    / \\  / ___|  \\/  |___  ___ _ __  ___  ___ ");
    println!("   / _ \\| |  _| |\\/| / __|/ _ \\ '_ \\/ __|/ _ \\");
    println!("  / ___ \\ |_| | |  | \\__ \\  __/ | | \\__ \\  __/");
    println!(" /_/   \\_\\____|_|  |_|___/\\___|_| |_|___/\\___|");
    println!("");
}

#[entry]
fn main() -> ! {
    // Disable the hardware watchdog timer.
    disable_wdg();

    // Initialize the master clock to use the main (external) clock.
    init_clock();

    // Initialize UART pins.
    init_pins();

    // Initialize the UART controller.
    let mut uart4 = Mb9bf61xtUart::new();
    uart4.init_uart();

    // Print banner and command list.
    print_banner();
    println!(" version 0.1, Jeff Glaum <jeffglaum@live.com>");
    println!("");

    loop {}
}
