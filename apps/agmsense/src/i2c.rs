use circular_buffer::CircularBuffer;
use cortex_m_rt::interrupt;
use embedded_hal::i2c::Operation;
use mb9bf61xt;
use mb9bf61xt::Interrupt as interrupt;

use crate::println;

// Helpful reference: https://github.com/fm3fan/uClinux/blob/master/drivers/i2c/busses/i2c-fm3.c
//
// NOTE: the FM3 datasheet indicates that in I2C mode (operation mode 4), the bus clock can't be operated at less than
// 8MHz and the I2C clock no faster than 400 kbps.
const MASTER_CLOCK_FREQ: u32 = 144000000; // Master clock (CLKPLL) is the PLL clock (see main.rs).
const HCLK_CLOCK_FREQ: u32 = MASTER_CLOCK_FREQ / 2; // Base clock divisor is 2 so HCLK = (CLKPLL/2).
const PLK2_CLOCK_FREQ: u32 = HCLK_CLOCK_FREQ / 2; // APB2 clock divisor is 2 so PCLK2 = (HCLK/2).
const I2C_BAUD_RATE: u32 = 400000; // I2C baud rate is 400kbps.
const MPU9250A_I2C_ADDRESS: u8 = 0x68; // MPU-9250A I2C bus address.
const TRANSMIT_QUEUE_DEPTH: usize = 4; // Transmit queue is 4 bytes deep.
const RECEIVE_DATA_DEPTH: usize = 4; // Receive data is 4 bytes deep.

pub struct Mb9bf61xtI2c;

// Queued transmit requests.
static mut TRANSMIT_QUEUE: CircularBuffer<TRANSMIT_QUEUE_DEPTH, u8> =
    CircularBuffer::<TRANSMIT_QUEUE_DEPTH, u8>::new();

// Data received from sensor.
static mut RECEIVE_DATA: CircularBuffer<RECEIVE_DATA_DEPTH, u8> =
    CircularBuffer::<RECEIVE_DATA_DEPTH, u8>::new();

impl Mb9bf61xtI2c {
    pub fn new() -> Self {
        return Self {};
    }

    // MFS channel 6 register base address: 0x4003.8600
    //
    pub fn init_i2c(&mut self) {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        // Serial Mode register (SMR).
        i2c6.i2c_i2c_smr().write(|w| unsafe { w.bits(0x80) }); // I2C mode (operation mode 4).

        // 7-bit Slave Address Mask Register (ISMK).
        i2c6.i2c_i2c_ismk().write(|w| unsafe { w.bits(0x7F) }); // Disable I2C controller.

        // Serial Status Register (SSR).
        i2c6.i2c_i2c_ssr().write(|w| unsafe { w.bits(0) });

        // Baud Rate Generator registers (BGR0 and BGR1).
        // Baud rate formula: Reload Value = ((Bus Clock Frequency / Baud Rate) - 1).
        // NOTE: datasheet indicates that these must be handled as a single 16-bit write.
        let reload_value = ((PLK2_CLOCK_FREQ / I2C_BAUD_RATE) - 1) as u16;
        i2c6.i2c_i2c_bgr()
            .write(|w| unsafe { w.bits(reload_value) });

        // // 7-bit Slave Address Register (ISBA).
        i2c6.i2c_i2c_isba().write(|w| unsafe { w.bits(0) }); // Disable slave mode.

        // 7-bit Slave Address Mask Register (ISMK).
        i2c6.i2c_i2c_ismk().modify(|_, w| w.en().set_bit()); // Enable I2C interface operations.

        // Clear the transmit queue and receive data queue.
        unsafe { TRANSMIT_QUEUE.clear() };
        unsafe { RECEIVE_DATA.clear() };

        // Enable the MFS6TX (I2C TX) and MFS6RX (I2C RX) interrupts.
        unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS6TX) };
        unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS6RX) };
    }

    // NOTE: this is a blocking call.
    pub fn read_i2c_bytes(address: u8, buf: &mut [u8]) -> usize {
        // Queue the transmit data.
        unsafe { TRANSMIT_QUEUE.push_front(MPU9250A_I2C_ADDRESS << 1 & !0x1) }; // Write.
        unsafe { TRANSMIT_QUEUE.push_front(address) };
        unsafe { TRANSMIT_QUEUE.push_front(MPU9250A_I2C_ADDRESS << 1 | 0x1) }; // Read.

        // Start the master transaction.
        Mb9bf61xtI2c::i2c_master_start();

        // TODO: Need lock.
        while unsafe { RECEIVE_DATA.is_empty() } {}

        // TODO: handle multiple bytes.
        buf[0] = unsafe { RECEIVE_DATA.pop_back().unwrap() };

        // TODO: return actual size.
        return 1;
    }

    fn i2c_write_tdr_byte(b: u8) {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;
        println!("Sending 0x{:X}", b);
        i2c6.i2c_i2c_tdr().write(|w| unsafe { w.bits(b as u16) });
    }

    fn i2c_master_data_rx() {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        if i2c6.i2c_i2c_ssr().read().ore() == true {
            // Overrun error.
            println!("OVERRUN error");
            // clear rx error interrupt */
            i2c6.i2c_i2c_ssr().modify(|_, w| w.rec().set_bit());
        } else if i2c6.i2c_i2c_ibsr().read().spc() == true {
            println!("STOP condition");
            i2c6.i2c_i2c_ibsr().modify(|_, w| w.spc().clear_bit());

            // stop condition interrupt disable, interrupt disable
            //i2c6.i2c_i2c_ibcr().modify(|_, w| w.cnde().clear_bit());
            i2c6.i2c_i2c_ibcr().modify(|_, w| w.inte().clear_bit());

            // clear IBSR:RACK
            i2c6.i2c_i2c_ismk().modify(|_, w| w.en().clear_bit());

            // restart
            i2c6.i2c_i2c_ismk().modify(|_, w| w.en().set_bit());
        } else if i2c6.i2c_i2c_ssr().read().rdrf() == true
            && i2c6.i2c_i2c_ibsr().read().fbt() == false
        {
            // TODO
            loop {
                if i2c6.i2c_i2c_ssr().read().rdrf() == false {
                    break;
                }
                let data = i2c6.i2c_i2c_rdr().read().bits();
                println!("INFO: Received data 0x{:X}.", data);
                unsafe { RECEIVE_DATA.push_front(data as u8) };
            }

            //i2c6.i2c_i2c_ibcr().modify(|_, w| w.mss().clear_bit());
            //i2c6.i2c_i2c_ibcr().modify(|_, w| w.acke().clear_bit());
            //i2c6.i2c_i2c_ibcr().modify(|_, w| w.cnde().set_bit());

            if i2c6.i2c_i2c_ibsr().read().rsc() == true {
                // clear restart condition
                i2c6.i2c_i2c_ibsr().modify(|_, w| w.rsc().clear_bit());
            }
        } else {
            // Do nothing
        }

        // clear interrupt
        //i2c6.i2c_i2c_ibcr().modify(|_, w| w.act_scc().clear_bit());
        i2c6.i2c_i2c_ibcr().modify(|_, w| w.int().clear_bit());
    }

    fn i2c_master_start() {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        if unsafe { TRANSMIT_QUEUE.is_empty() } == false {
            // I2C Bus Control Register (IBCR).
            i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0) }); // Clear.

            // Write slave address to TDR.
            Mb9bf61xtI2c::i2c_write_tdr_byte(unsafe { TRANSMIT_QUEUE.pop_back().unwrap() });

            // I2C Bus Control Register (IBCR).
            i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x85) }); // Enable master, enable interrupt, select interrupt.
        }
    }

    fn i2c_master_data_tx() {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        if i2c6.i2c_i2c_ibsr().read().rack() == true {
            // If SDA is high (true), it's a NACK.
            println!("NACK received!");
            // TODO
        } else if i2c6.i2c_i2c_ibcr().read().ber() == true {
            println!("BUS error");
            // TODO
        } else if i2c6.i2c_i2c_ibsr().read().al() == true {
            println!("ARBITRATION lost");
            // TODO
        } else if i2c6.i2c_i2c_ibsr().read().spc() == true {
            println!("STOP condition");
            // TODO
        } else {
            if i2c6.i2c_i2c_ibsr().read().trx() == true {
                // *** more data to transfer ***
                // Make sure the transmitter is empty.
                if i2c6.i2c_i2c_ssr().read().tdre() == true
                    && unsafe { TRANSMIT_QUEUE.is_empty() == false }
                {
                    // Write slave address.
                    Mb9bf61xtI2c::i2c_write_tdr_byte(unsafe { TRANSMIT_QUEUE.pop_back().unwrap() });

                    // I2C Bus Control Register (IBCR).
                    i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x84) }); // Master enable, interrupt enable, clear interrupt.
                }
            } else {
                println!("Read Data");
                // *** Data to read ***
                loop {
                    if i2c6.i2c_i2c_ssr().read().rdrf() == false {
                        break;
                    }
                    let data = i2c6.i2c_i2c_rdr().read().bits();
                    println!("INFO: Received data 0x{:X}.", data);
                    unsafe { RECEIVE_DATA.push_front(data as u8) };
                }
                // I2C Bus Control Register (IBCR).
                //let mut v = i2c6.i2c_i2c_ibcr().read().bits();
                //v &= !(0x25); // Clear ACK, interrupt enable, clear interrupt.
                //v |= 0x10; // Set WSEL.
                //i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(v) }); // Send ACK.

                // I2C Bus Control Register (IBCR).
                i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x20) }); // Stop condition.
            }

            // Clear the interrupt regardles of whether it was cleared above.
            i2c6.i2c_i2c_ibcr().modify(|_, w| w.int().clear_bit());
        }
    }

    // NOTE: this is a blocking call.
    pub fn write_i2c_bytes(address: u8, _buf: &mut &[u8]) -> usize {
        println!("INFO: I2C write to address 0x{:X}", address);
        return 0;
    }
}

impl embedded_hal::i2c::ErrorType for Mb9bf61xtI2c {
    type Error = core::convert::Infallible;
}

impl embedded_hal::i2c::I2c for Mb9bf61xtI2c {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [Operation<'_>],
    ) -> Result<(), Self::Error> {
        for (_i, operation) in operations.iter_mut().enumerate() {
            match operation {
                Operation::Read(buf) => {
                    let _result = Mb9bf61xtI2c::read_i2c_bytes(address, buf);
                }
                Operation::Write(buf) => {
                    let _result = Mb9bf61xtI2c::write_i2c_bytes(address, buf);
                }
            }
        }

        Ok(())
    }
}

#[interrupt]
fn MFS6TX() {
    println!("TX interrupt");
    Mb9bf61xtI2c::i2c_master_data_tx();
}

#[interrupt]
fn MFS6RX() {
    println!("RX interrupt");
    Mb9bf61xtI2c::i2c_master_data_rx();
}
