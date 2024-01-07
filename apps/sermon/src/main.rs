#![no_std]
#![no_main]

// pick a panicking behavior
use panic_halt as _;

use cortex_m::asm;
use cortex_m_rt::entry;
use mb9bf61xt;


const BAUD_RATE: u32 = 115200;                          // Baud Rate.
const MASTER_CLOCK_FREQ: u32 = 4000000;                 // Master clock (HCLK) is the main (external) clock.
const BUS_CLOCK_FREQ: u32 = MASTER_CLOCK_FREQ / 1;      // Bus clock divisors are all 1:1 (HW reset) so PCLK = (HCLK/1).


fn initclock() {

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let clock = p.CRG;

    // Enable the main (external) clock oscillator.
    clock.scm_ctl().modify(|_,w| w.mosce().set_bit());

    // TBD: this is a hack to avoid having to set up the oscillator stabilization interrupt.
    asm::delay(5000);

    // Configure the master clock to use the main (external clock).
    clock.scm_ctl().modify(|_,w| unsafe {w.rcs().bits(0x1)});

    // TBD: this is a hack to avoid having to set up the oscillator stabilization interrupt.
    asm::delay(5000);
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
    gpio.pfr0().modify(|_,w| w.p05().set_bit());
    gpio.pfr0().modify(|_,w| w.p06().set_bit());

    // Extended Pin Function Setting register (EPFR0x) - configure UART pin functions.
    // EPFR08 controls UART channels 4-7.
    let mut current_epfr08 = gpio.epfr08().read().bits();
    current_epfr08 &= !0x3FF;       // Mask off channel 4 bits.
    current_epfr08 |= 0x0FC;        // No RTS, CTS 4_2, SIN 4_2, SOT 4_2, no SCK.
    gpio.epfr08().write(|w| unsafe {w.bits(current_epfr08)});

}

fn inituart() {

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let uart4 = p.MFS4;

    // Serial Mode register (SMR).
    uart4.uart_uart_smr().modify(|_,w| w.soe().set_bit());         // Serial data output enable.
    uart4.uart_uart_smr().modify(|_,w| w.bds().clear_bit());       // LSB first.
    uart4.uart_uart_smr().modify(|_,w| w.sbl().clear_bit());       // Select either 1 or 3 stop bits (ESCR.ESBL decides).
    uart4.uart_uart_smr().modify(|_,w| w.wucr().clear_bit());      // Disable the wake-up function.
    uart4.uart_uart_smr().modify(|_,w| unsafe {w.md().bits(0)});   // Async normal mode.

    // Baud Rate Generator registers (BGR0 and BGR1).
    // Baud rate formula: Reload Value = ((Bus Clock Frequency / Baud Rate) - 1).
    // NOTE: datasheet indicates that these must be handled as a single 16-bit write.
    let reload_value = (BUS_CLOCK_FREQ / BAUD_RATE) - 1;
    let bgr_value = (reload_value as u16) & !0x8000;        // Clear upper bit (EXT) to indicate the internal clock should be used.
    uart4.uart_uart_bgr().write(|w| unsafe {w.bits(bgr_value)});

    // Extended Communications Control register (ESCR).
    uart4.uart_uart_escr().modify(|_,w| unsafe {w.l().bits(0)});   // 8 data bits.
    uart4.uart_uart_escr().modify(|_,w| w.p().clear_bit());        // Even parity.
    uart4.uart_uart_escr().modify(|_,w| w.pen().clear_bit());      // Disable parity.
    uart4.uart_uart_escr().modify(|_,w| w.inv().clear_bit());      // NRZ format.
    uart4.uart_uart_escr().modify(|_,w| w.esbl().clear_bit());     // 1 stop bit (works along with the SMR setting above).
    uart4.uart_uart_escr().modify(|_,w| w.flwen().clear_bit());    // Disable hardware flow control.

    // FIFO Control registers (FCR0 and FCR1).
    uart4.uart_uart_fcr0().modify(|_,w| unsafe {w.bits(0)});       // Disable FIFOs.
    uart4.uart_uart_fcr1().modify(|_,w| unsafe {w.bits(0)});       //

    // TBD: Ignoring FBYTE1 and FBYTE2 registers for now since the FIFOs are disabled.

    // Serial Control Register (SCR).
    uart4.uart_uart_scr().modify(|_,w| w.tbie().clear_bit());      // Disable transmit bus idle interrupt.
    uart4.uart_uart_scr().modify(|_,w| w.tie().clear_bit());       // Disable transmit interrupt.
    uart4.uart_uart_scr().modify(|_,w| w.rie().clear_bit());       // Disable receive interrupt.
    uart4.uart_uart_scr().modify(|_,w| w.txe().set_bit());         // Enable transmitter.
    uart4.uart_uart_scr().modify(|_,w| w.rxe().set_bit());         // Enable receiver.

}

fn writeuart(c : u8) {

    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let uart4 = p.MFS4;

    //Send the character.
    uart4.uart_uart_tdr().write(|w| unsafe {w.bits(c.into())});
}

#[entry]
fn main() -> ! {

    // Initialize the master clock to use the main (external) clock.
    initclock();

    // Initialize UART pins.
    initpins();

    // Initialize the UART controller.
    inituart();

    loop {
        // Send characters over UART.
        writeuart(b'T');
        asm::delay(500);
        writeuart(b'e');
        asm::delay(500);
        writeuart(b's');
        asm::delay(500);
        writeuart(b't');
        asm::delay(500);
        writeuart(b' ');

        // Delay 5000 clock cycles. 
        // TBD: write routine should use FIFO and check for space instead of just waiting.
        asm::delay(5000);
    }
}
