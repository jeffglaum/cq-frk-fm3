use circular_buffer::CircularBuffer;
use cortex_m;
use cortex_m_rt::interrupt;
use embedded_io::Write;
use mb9bf61xt;
use mb9bf61xt::Interrupt as interrupt;

const MASTER_CLOCK_FREQ: u32 = 4000000; // Master clock (HCLK) is the main (external) clock.  CQ-FRK-FM3 uses a 4Mhz xtal.
const BUS_CLOCK_FREQ: u32 = MASTER_CLOCK_FREQ / 1; // Bus clock divisors are all 1:1 (HW reset) so PCLK = (HCLK/1).
const BAUD_RATE: u32 = 115200; // Baud Rate.
const MAX_TX_FIFO_DEPTH: u8 = 16; // Maximum transmit (and receive) FIFO depth is 16 x 9 bits.
const LINE_BUFFER_SIZE: usize = 32; // UART receive buffer size in bytes.

#[allow(unused_macros)]
#[macro_export]
macro_rules! print {
    ($($args:tt)*) => {
        serial::serial_printfmt(core::format_args!($($args)*));
    }
}

#[allow(unused_macros)]
#[macro_export]
macro_rules! println {
    ($($args:tt)*) => {
        serial::serial_printfmt(core::format_args!($($args)*));
        serial::serial_printfmt(core::format_args!("\n"));
    };
}

pub struct Mb9bf61xtUart;

static mut INPUT_BUFFER: CircularBuffer<LINE_BUFFER_SIZE, u8> =
    CircularBuffer::<LINE_BUFFER_SIZE, u8>::new();

impl Mb9bf61xtUart {
    pub fn new() -> Self {
        return Self {};
    }

    pub fn init_uart(&mut self) {
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

        // Enable the MFS4RX (UART RX) interrupt.
        unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS4RX) };
    }

    // NOTE: this is a blocking call.
    pub fn read_uart_bytes(buf: &mut [u8]) -> usize {
        // Disable the MFS4RX (UART RX) interrupt.
        // TODO: disable rx interrupts to avoid circular buffer race condition.
        //cortex_m::peripheral::NVIC::mask(interrupt::MFS4RX);

        // Wait until there's at least one character.
        unsafe {
            while INPUT_BUFFER.is_empty() {
                // NOP so Rust compiler doesn't optimize-away this loop.
                core::arch::asm!("nop");
            }
        }

        let mut i = 0;
        loop {
            let c = unsafe { INPUT_BUFFER.pop_back() };
            if c == None {
                break;
            }
            buf[i] = c.unwrap();
            i += 1;
        }

        // Enable the MFS4RX (UART RX) interrupt.
        //unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS4RX) };

        return i;
    }

    pub fn write_uart_string(buf: &[u8]) -> usize {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let uart4 = p.MFS4;

        // Per the datasheet, only write to the FIFO when it's empty.
        // TODO: write to buffer and use the tx interrupt to complete the transfer asynchronously.
        while uart4.uart_uart_ssr().read().tdre() == false {}

        let buf_len = buf.len();
        let xmit_len = core::cmp::min(buf_len, MAX_TX_FIFO_DEPTH.into());

        for i in 0..xmit_len {
            // Send the character.
            uart4
                .uart_uart_tdr()
                .write(|w| unsafe { w.bits(buf[i].into()) });
        }

        return xmit_len;
    }
}

impl embedded_io::ErrorType for Mb9bf61xtUart {
    type Error = core::convert::Infallible;
}

impl embedded_io::Write for Mb9bf61xtUart {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let wrote_len = Mb9bf61xtUart::write_uart_string(buf);
        Ok(wrote_len)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

impl embedded_io::Read for Mb9bf61xtUart {
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let read_len = Mb9bf61xtUart::read_uart_bytes(buf);
        Ok(read_len)
    }
}

pub fn serial_print(bytes: &[u8]) {
    let _result = Mb9bf61xtUart.write_all(bytes);
}

pub fn serial_printfmt(fmt: core::fmt::Arguments<'_>) {
    let _result = Mb9bf61xtUart.write_fmt(fmt);
}

#[interrupt]
fn MFS4RX() {
    let p = unsafe { mb9bf61xt::Peripherals::steal() };
    let uart4 = p.MFS4;

    // Read everything out of the receive FIFO.
    // TODO: need to handle overrun, framing, and possibly parity errors.
    while uart4.uart_uart_ssr().read().rdrf() == true {
        let c = uart4.uart_uart_rdr().read().bits() as u8;
        crate::serial::serial_print(&[c]);
        // TODO - why is this unsafe?
        unsafe { crate::serial::INPUT_BUFFER.push_front(c) };
    }
}
