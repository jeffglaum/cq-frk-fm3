#![no_std]
#![no_main]

use embedded_hal::i2c::I2c;
use panic_halt as _;

use cortex_m_rt::entry;
use mb9bf61xt;

mod serial;
pub use crate::serial::Mb9bf61xtUart;
mod i2c;
pub use crate::i2c::Mb9bf61xtI2c;

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

    // Set the base and accessor clock pre-scalers.
    //
    // NOTE: these pre-scalar values appear to be in violation of the datasheet (80MHz max base clock, 40MHz PLK2) but
    // they are in the sample code provide by Fujitsu and appear to be functional.
    clock.bsc_psr().modify(|_, w| unsafe { w.bsr().bits(0) }); // Base clock (BSC_PSR) is same as CLKPLL clock.
    clock
        .apbc0_psr()
        .modify(|_, w| unsafe { w.apbc0().bits(0x1) }); // PLK0 (APBC0_PSR) is 1/2 Base clock.
    clock
        .apbc1_psr()
        .modify(|_, w| unsafe { w.apbc1().bits(0x81) }); // PLK1 (APBC0_PSR) is 1/2 Base clock.
    clock
        .apbc2_psr()
        .modify(|_, w| unsafe { w.apbc2().bits(0x81) }); // PLK2 (APBC2_PSR) is 1/2 Base clock.

    // Force-clear all possible oscillization stabilization wait interrupt causes (anomolous freq, main, sub, and PLL).
    clock.int_clr().write(|w| unsafe { w.bits(0x27) });

    // Set the main oscillator stabilization wait time (~33ms).
    clock.csw_tmr().write(|w| unsafe { w.bits(0xC) });

    // Set the oscillation stabilization wait interrupt.
    clock.int_enr().modify(|_, w| w.mcse().set_bit());

    // Enable the main (external) clock oscillator.
    clock.scm_ctl().modify(|_, w| w.mosce().set_bit());

    // Check the main oscillator stable bit.
    // TODO: anything to be done with a time-out here?
    while clock.scm_str().read().mordy() != true {}

    // Set the PLL stabilization wait time.
    clock.psw_tmr().modify(|_, w| unsafe { w.powt().bits(0x2) }); // 512us wait time.
    clock.psw_tmr().modify(|_, w| w.pinc().clear_bit()); // Use main oscillator as source.

    // Set the PLL oscillation wait interrupt.
    clock.int_enr().modify(|_, w| w.pcse().set_bit()); // Enable PLL oscillation stabilization interrupt.

    // Set the PLL multiplication ratio.  4MHz base clock K=1 (PLLin=4MHz), N=36 (PLLout=288MHz), M=2 (CLKPLL=144MHz), denominators.
    // PllOut = PllIn * (M * N) = 4MHz * (2 * 36) = 288MHz
    // CLKPLL = PllOut / M = (288MHz / 2) = 144MHz
    clock
        .pll_ctl1()
        .modify(|_, w| unsafe { w.pllk().bits(0x0) }); // PLLK = 0, K = 1/(PLLK+1) = 1.
    clock
        .pll_ctl1()
        .modify(|_, w| unsafe { w.pllm().bits(0x1) }); // PLLM = 1, M = 1/(PLLM+1) = 1/2.
    clock
        .pll_ctl2()
        .modify(|_, w| unsafe { w.plln().bits(0x23) }); // PLLN = 35, N = 1/(PLLN+1) = 1/36.

    // Enable PLL oscillation.
    clock.scm_ctl().modify(|_, w| w.plle().set_bit());

    // Check the main oscillator stable bit.
    // TODO: anything to be done with a time-out here?
    while clock.scm_str().read().plrdy() != true {}

    // Configure the master clock to use the PLL clock (based on external xtal).
    clock.scm_ctl().modify(|_, w| unsafe { w.rcs().bits(0x2) });

    // Wait for master clock selection to take effect.
    // TODO: anything to be done with a time-out here?
    while clock.scm_str().read().rcm() != 0x2 {}
}

fn init_pins() {
    // FM3 (MB9BF61xT) MFS (UART & I2C) pin options:
    // NOTE: MFS (UART & I2C) channels 0-3 don't offer a FIFO, channels 4-7 do have a FIFO.
    //
    // * SIN0_0:P21, SIN1_0:P56, SIN2_0:P72, SIN3_0:P75, SIN4_0:PD2, SIN5_0:P60, SIN6_0:P53, SIN7_0:P59
    // * SOT0_0:P22, SOT1_0:P57, SOT2_0:P73, SOT3_0:P76, SOT4_0:PD1, SOT5_0:P61, SOT6_0:P54, SOT7_0:P5A
    //
    // This code uses MFS channel 4 for UART.  On the CQ-FRK-FM3 board this uses connector CN2's pin 14 (SOT 4_2,
    // GPIO P06) and pin 11 (SIN 4_2, GPIO P05).
    //
    // This code uses  MFS channel 6 for I2C.  On the CQ-FRK-FM3 board this uses connector CN3's pin 40 (SCK 6_1,
    // GPIO P31) and pin 41 (SDA 6_1, GPIO P32).
    //

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // ** UART **

    // Port Function Setting register (PFRx) - configure channel 4's MFS pins to be a peripheral (not GPIO) function.
    // PFR0 controls GPIO pins P00 through P0F.  Configure P05 (SIN 4_2) and P06 (SOT 4_2) as peripheral pins.
    gpio.pfr0().modify(|_, w| w.p05().set_bit());
    gpio.pfr0().modify(|_, w| w.p06().set_bit());

    // Extended Pin Function Setting register (EPFR0x) - configure MFS pin functions.
    // EPFR08 controls MFS channels 4-7.
    let mut current_epfr08 = gpio.epfr08().read().bits();
    current_epfr08 &= !0x3FF; // Mask off channel 4 bits.
    current_epfr08 |= 0x0FC; // No RTS, CTS 4_2, SIN 4_2, SOT 4_2, no SCK.
    gpio.epfr08().write(|w| unsafe { w.bits(current_epfr08) });

    // ** I2C **

    // Port Function Setting register (PFRx) - configure channel 6's MFS pins to be a peripheral (not GPIO) function.
    // PFR3 controls GPIO pins P30 through P3F.  Configure P31 (SCK 6_1) and P32 (SDA 6_1) as peripheral pins.
    gpio.pfr3().modify(|_, w| w.p31().set_bit());
    gpio.pfr3().modify(|_, w| w.p32().set_bit());

    // Extended Pin Function Setting register (EPFR0x) - configure MFS pin functions.
    // EPFR08 controls MFS channels 4-7.
    let mut current_epfr08 = gpio.epfr08().read().bits();
    current_epfr08 &= !0x3F0000; // Mask off channel 6 bits.
    current_epfr08 |= 0x2A0000; // SIN 6_1, SOT 6_1, SCK 6_1.
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

    // Initialize the I2C controller.
    let mut i2c6 = Mb9bf61xtI2c::new();
    i2c6.init_i2c();

    // Print banner and command list.
    print_banner();
    println!(" version 0.1, Jeff Glaum <jeffglaum@live.com>");
    println!("");

    // Try reading the MPU-9250A "Who am I?" register.  It should return 0x71 (possibly 0x68?).
    let mut wai: [u8; 1] = [0];
    let _result = i2c6.read(0x75, &mut wai).unwrap();
    println!("MPU-9250A WHO_AM_I value=0x{:x}", wai[0]);

    loop {}
}
