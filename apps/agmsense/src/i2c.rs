use cortex_m_rt::interrupt;
use embedded_hal::i2c::{NoAcknowledgeSource, Operation};
use mb9bf61xt;
use mb9bf61xt::Interrupt as interrupt;
use rtt_target::rprintln;

use crate::println;

// Helpful reference: https://github.com/fm3fan/uClinux/blob/master/drivers/i2c/busses/i2c-fm3.c
//
// NOTE: the FM3 datasheet indicates that in I2C mode (operation mode 4), the bus clock can't be operated at less than
// 8MHz and the I2C clock no faster than 400 kbps.
const MASTER_CLOCK_FREQ: u32 = 144000000; // Master clock (CLKPLL) is the PLL clock (see main.rs).
const HCLK_CLOCK_FREQ: u32 = MASTER_CLOCK_FREQ / 2; // Base clock divisor is 2 so HCLK = (CLKPLL/2).
const PLK2_CLOCK_FREQ: u32 = HCLK_CLOCK_FREQ / 2; // APB2 clock divisor is 2 so PCLK2 = (HCLK/2).
const I2C_BAUD_RATE: u32 = 400000; // I2C baud rate is 400kbps.
                                   //const MPU9250A_I2C_ADDRESS: u8 = 0x68; // MPU-9250A I2C bus address.

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[non_exhaustive]
pub enum Error {
    Bus,
    ArbitrationLoss,
    NoAcknowledge(NoAcknowledgeSource),
    Overrun,
    ReservedAddress,
    Other,
}

impl Error {
    pub(crate) fn nack_addr(self) -> Self {
        match self {
            Error::NoAcknowledge(NoAcknowledgeSource::Unknown) => {
                Error::NoAcknowledge(NoAcknowledgeSource::Address)
            }
            e => e,
        }
    }
    pub(crate) fn nack_data(self) -> Self {
        match self {
            Error::NoAcknowledge(NoAcknowledgeSource::Unknown) => {
                Error::NoAcknowledge(NoAcknowledgeSource::Data)
            }
            e => e,
        }
    }
}

pub struct Mb9bf61xtI2c;

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

        // Enable the MFS6TX (I2C TX) and MFS6RX (I2C RX) interrupts.
        // TODO
        //unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS6TX) };
        //unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS6RX) };
    }

    fn prepare_write(&self, address: u8) -> Result<(), Error> {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        // I2C Bus Control Register (IBCR).
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0) }); // Clear.

        // Write slave address to TDR.
        i2c6.i2c_i2c_tdr()
            .write(|w| unsafe { w.bits(address as u16) });

        // I2C Bus Control Register (IBCR).
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x85) }); // Enable master, enable interrupt, select interrupt.

        Ok(())
    }

    fn prepare_read(&self, address: u8) -> Result<(), Error> {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        // I2C Bus Control Register (IBCR).
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0) }); // Clear.

        // Write slave address to TDR.
        i2c6.i2c_i2c_tdr()
            .write(|w| unsafe { w.bits(address as u16) });

        // I2C Bus Control Register (IBCR).
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x85) }); // Enable master, enable interrupt, select interrupt.

        Ok(())
    }

    fn write_bytes(&self, wb: &[u8]) -> Result<(), Error> {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        let oi = wb.iter();
        for op in oi {
            let _ = self.check_and_clear_error_flags();

            // Make sure the transmitter is empty.
            while i2c6.i2c_i2c_ssr().read().tdre() == false {}

            i2c6.i2c_i2c_tdr().write(|w| unsafe { w.bits(*op as u16) });

            // I2C Bus Control Register (IBCR).
            i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x84) }); // Master enable, interrupt enable, clear interrupt.

            let _ = self.check_and_clear_error_flags();
        }
        Ok(())
    }

    fn read_bytes(&self, _rb: &mut [u8]) -> Result<(), Error> {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        while i2c6.i2c_i2c_ssr().read().rdrf() == true {
            let _ = self.check_and_clear_error_flags();
            let data = i2c6.i2c_i2c_rdr().read().bits();
            println!("INFO: Received data 0x{:X}.", data);
        }
        //i2c6.i2c_i2c_ibcr().modify(|_, w| w.mss().clear_bit());
        //i2c6.i2c_i2c_ibcr().modify(|_, w| w.acke().clear_bit());
        //i2c6.i2c_i2c_ibcr().modify(|_, w| w.cnde().set_bit());

        if i2c6.i2c_i2c_ibsr().read().rsc() == true {
            // clear restart condition
            i2c6.i2c_i2c_ibsr().modify(|_, w| w.rsc().clear_bit());
        }
        Ok(())
    }

    fn read_wo_prepare(&mut self, _rb: &mut [u8]) -> Result<(), Error> {
        Ok(())
    }

    fn write_wo_prepare(&mut self, _wb: &[u8]) -> Result<(), Error> {
        Ok(())
    }

    fn check_and_clear_error_flags(&self) -> Result<(), Error> {
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
        } else if i2c6.i2c_i2c_ibsr().read().rack() == true {
            // If SDA is high (true), it's a NACK.
            println!("NACK received!");
            // TODO
        } else if i2c6.i2c_i2c_ibcr().read().ber() == true {
            println!("BUS error");
            // TODO
        } else if i2c6.i2c_i2c_ibsr().read().al() == true {
            println!("ARBITRATION lost");
            // TODO
        }

        Ok(())
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
        let mut oi = operations.iter_mut();
        if let Some(mut prev_op) = oi.next() {
            // 1. generate START for operation
            let _ = match &prev_op {
                Operation::Read(_) => self.prepare_read(address),
                Operation::Write(_) => self.prepare_write(address),
            };
            for op in oi {
                // 2. execute previous operation
                let _ = match &mut prev_op {
                    Operation::Read(rb) => self.read_bytes(rb),
                    Operation::Write(wb) => self.write_bytes(wb),
                };
                // 3. if operation changes type we must generate a new START
                let _ = match (&prev_op, &op) {
                    (Operation::Read(_), Operation::Write(_)) => self.prepare_write(address),
                    (Operation::Write(_), Operation::Read(_)) => self.prepare_read(address),
                    _ => Ok(()),
                };
                prev_op = op;
            }
            // 4. here prev_op is teh last command, use variations that will generate stop
            let _ = match prev_op {
                Operation::Read(rb) => self.read_wo_prepare(rb),
                Operation::Write(wb) => self.write_wo_prepare(wb),
            };
        }
        Ok(())
    }
}

#[interrupt]
fn MFS6TX() {
    rprintln!("TX interrupt");
}

#[interrupt]
fn MFS6RX() {
    rprintln!("RX interrupt");
}
