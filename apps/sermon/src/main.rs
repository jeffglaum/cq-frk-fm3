#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m;
use cortex_m_rt::{entry, interrupt};
use embedded_io::Write;
use mb9bf61xt;
use mb9bf61xt::Interrupt as interrupt;

const BAUD_RATE: u32 = 115200; // Baud Rate.
const MASTER_CLOCK_FREQ: u32 = 4000000; // Master clock (HCLK) is the main (external) clock.  CQ-FRK-FM3 uses a 4Mhz xtal.
const BUS_CLOCK_FREQ: u32 = MASTER_CLOCK_FREQ / 1; // Bus clock divisors are all 1:1 (HW reset) so PCLK = (HCLK/1).
const MAX_TX_FIFO_DEPTH: u8 = 16; // Maximum transmit (and receive) FIFO depth is 16 x 9 bits.

#[allow(unused_macros)]
macro_rules! print {
    ($($args:tt)*) => {
        _serial_printfmt(core::format_args!($($args)*));
    }
}

#[allow(unused_macros)]
macro_rules! println {
    ($($args:tt)*) => {
        _serial_printfmt(core::format_args!($($args)*));
        _serial_printfmt(core::format_args!("\n"));
    };
}

fn disablewdg() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let wdg = p.HWWDT;

    // Unlock the watchdog registers.
    wdg.wdg_lck().write(|w| unsafe { w.bits(0x1ACCE551) });
    wdg.wdg_lck().write(|w| unsafe { w.bits(0xE5331AAE) });

    // Disable the hardware watchdog timer.
    wdg.wdg_ctl().modify(|_, w| w.inten().clear_bit());
}

fn initclock() {
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

fn initpins() {
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

fn initgpio() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;

    // Set to GPIO mode.
    gpio.pfrf().write(|w| w.pf3().clear_bit());

    // Set to open-drain mode.
    gpio.pzrf().write(|w| w.pf3().set_bit());

    // Set to output.
    gpio.ddrf().write(|w| w.pf3().set_bit());

    // Set to high level.
    gpio.pdorf().write(|w| w.pf3().set_bit());
}

struct Mb9bf61xtUart;

impl Mb9bf61xtUart {
    fn inituart() {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let uart4 = p.MFS4;

        // Initialize internal UART state.
        uart4.uart_uart_scr().modify(|_, w| w.upcl().clear_bit()); // Programmable clear.

        // Serial Mode register (SMR).
        uart4
            .uart_uart_smr()
            .modify(|_, w| unsafe { w.md().bits(0) }); // Async normal mode.
        uart4.uart_uart_smr().modify(|_, w| w.soe().set_bit()); // Serial data output enable.
        uart4.uart_uart_smr().modify(|_, w| w.bds().clear_bit()); // LSB first.
        uart4.uart_uart_smr().modify(|_, w| w.sbl().clear_bit()); // Select either 1 or 3 stop bits (ESCR.ESBL decides).
        uart4.uart_uart_smr().modify(|_, w| w.wucr().clear_bit()); // Disable the wake-up function.

        // Baud Rate Generator registers (BGR0 and BGR1).
        // Baud rate formula: Reload Value = ((Bus Clock Frequency / Baud Rate) - 1).
        // NOTE: datasheet indicates that these must be handled as a single 16-bit write.
        let reload_value = (BUS_CLOCK_FREQ / BAUD_RATE) - 1;
        let bgr_value = (reload_value as u16) & !0x8000; // Clear upper bit (EXT) to indicate the internal clock should be used.
        uart4
            .uart_uart_bgr()
            .write(|w| unsafe { w.bits(bgr_value) });

        // Extended Communications Control register (ESCR).
        uart4
            .uart_uart_escr()
            .modify(|_, w| unsafe { w.l().bits(0) }); // 8 data bits.
        uart4.uart_uart_escr().modify(|_, w| w.p().clear_bit()); // Even parity.
        uart4.uart_uart_escr().modify(|_, w| w.pen().clear_bit()); // Disable parity.
        uart4.uart_uart_escr().modify(|_, w| w.inv().clear_bit()); // NRZ format.
        uart4.uart_uart_escr().modify(|_, w| w.esbl().clear_bit()); // 1 stop bit (works along with the SMR setting above).
        uart4.uart_uart_escr().modify(|_, w| w.flwen().clear_bit()); // Disable hardware flow control.

        // FIFO Control registers (FCR0 and FCR1).
        uart4.uart_uart_fcr1().modify(|_, w| w.fsel().set_bit()); // Transmit is FIFO2 and Receive is FIFO1.
        uart4.uart_uart_fcr1().modify(|_, w| w.ftie().clear_bit()); // Disable transmit FIFO interrupt.
        uart4.uart_uart_fcr1().modify(|_, w| w.friie().clear_bit()); // Disable receive FIFO idle detection.
        uart4.uart_uart_fcr1().modify(|_, w| w.flste().clear_bit()); // Disable data loss detection.
        uart4.uart_uart_fcr0().modify(|_, w| w.fcl1().set_bit()); // Reset FIFO1 (receive) state.
        uart4.uart_uart_fcr0().modify(|_, w| w.fcl2().set_bit()); // Reset FIFO2 (transmit) state.
        uart4.uart_uart_fcr0().modify(|_, w| w.fe1().set_bit()); // Enable FIFO1 (receive) operations.
        uart4.uart_uart_fcr0().modify(|_, w| w.fe2().set_bit()); // Enable FIFO2 (transmit) operations.

        // FIFO Byte register (FBYTE1 and FBYTE2) - set the receive FIFO level that generates a receive interrupt.
        // NOTE: a read-modify-write cannot be used for this register.
        // Reset the transmit FIFO FBYTE value.
        uart4.uart_uart_fbyte2().write(|w| unsafe { w.bits(0) });
        // Set the interrupt to trigger at the half-full point.
        // uart4
        //     .uart_uart_fbyte1()
        //     .write(|w| unsafe { w.bits(MAX_TX_FIFO_DEPTH / 2) });
        uart4.uart_uart_fbyte1().write(|w| unsafe { w.bits(1) });

        // Serial Control Register (SCR).
        uart4.uart_uart_scr().modify(|_, w| w.tbie().clear_bit()); // Disable transmit bus idle interrupt.
        uart4.uart_uart_scr().modify(|_, w| w.tie().clear_bit()); // Disable transmit interrupt.
        uart4.uart_uart_scr().modify(|_, w| w.rie().set_bit()); // Enable receive interrupt.
        uart4.uart_uart_scr().modify(|_, w| w.txe().set_bit()); // Enable transmitter.
        uart4.uart_uart_scr().modify(|_, w| w.rxe().set_bit()); // Enable receiver.
    }

    fn writeuartstring(buf: &[u8]) -> usize {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let uart4 = p.MFS4;

        // Per the datasheet, only write to the FIFO when it's empty.
        // TODO: write to buffer and use the tx interrupt to complete the transfer asynchronously.
        while uart4.uart_uart_ssr().read().tdre() == false {}

        let buf_len = buf.len();
        // TODO: how to get min without std?
        let xmit_len = if buf_len < MAX_TX_FIFO_DEPTH.into() {
            buf_len
        } else {
            MAX_TX_FIFO_DEPTH.into()
        };

        for i in 0..xmit_len {
            // Send the character.
            uart4
                .uart_uart_tdr()
                .write(|w| unsafe { w.bits(buf[i].into()) });
        }

        return xmit_len;
    }
}

fn print_banner() {
    println!("");
    println!("     _____           _       __   __  ___            _ __            ");
    println!("    / ___/___  _____(_)___ _/ /  /  |/  /___  ____  (_) /_____  _____");
    println!("    \\__ \\/ _ \\/ ___/ / __ `/ /  / /|_/ / __ \\/ __ \\/ / __/ __ \\/ ___/");
    println!("   ___/ /  __/ /  / / /_/ / /  / /  / / /_/ / / / / / /_/ /_/ / /    ");
    println!("  /____/\\___/_/  /_/\\__,_/_/  /_/  /_/\\____/_/ /_/_/\\__/\\____/_/     ");
    println!("");
}

#[entry]
fn main() -> ! {
    // Disable the hardware watchdog timer.
    disablewdg();

    // Initialize GPIO used for the LED.
    initgpio();

    // Initialize the master clock to use the main (external) clock.
    initclock();

    // Initialize UART pins.
    initpins();

    // Initialize the UART controller.
    Mb9bf61xtUart::inituart();

    // Enable the MFS4RX (UART RX) interrupt.
    unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS4RX) };

    // Print banner.
    print_banner();
    println!("Version 0.1, Jeff Glaum <jeffglaum@live.com>");
    println!("");
    print!("> ");

    loop {}
}

impl embedded_io::ErrorType for Mb9bf61xtUart {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for Mb9bf61xtUart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let wrote_len = Mb9bf61xtUart::writeuartstring(buf);
        Ok(wrote_len)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn _serial_print(bytes: &[u8]) {
    let _result = Mb9bf61xtUart.write_all(bytes);
}

fn _serial_printfmt(fmt: core::fmt::Arguments<'_>) {
    let _result = Mb9bf61xtUart.write_fmt(fmt);
}

#[interrupt]
fn MFS4RX() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let gpio = p.GPIO;
    let uart4 = p.MFS4;

    // Invert the LED output.
    let current_pf3_val = gpio.pdorf().read().pf3().bit_is_set();
    gpio.pdorf().write(|w| w.pf3().bit(!current_pf3_val));

    // Read everything out of the receive FIFO.
    // TODO: need to handle overrun, framing, and possibly parity errors.
    while uart4.uart_uart_ssr().read().rdrf() == true {
        let c = uart4.uart_uart_rdr().read().bits() as u8;
        _serial_print(&[c]);
    }
}
