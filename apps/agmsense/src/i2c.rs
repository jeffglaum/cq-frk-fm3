use cortex_m_rt::interrupt;
use embedded_hal::i2c::{NoAcknowledgeSource, Operation};
use mb9bf61xt;
use mb9bf61xt::Interrupt as interrupt;
use rtt_target::rprintln;

// Helpful reference: https://github.com/fm3fan/uClinux/blob/master/drivers/i2c/busses/i2c-fm3.c
//
// NOTE: the FM3 datasheet indicates that in i2c mode (operation mode 4), the bus clock can't be operated at less than
// 8MHz and the i2c clock no faster than 400 kbps.
const MASTER_CLOCK_FREQ: u32 = 144000000; // Master clock (CLKPLL) is the PLL clock (see main.rs).
const HCLK_CLOCK_FREQ: u32 = MASTER_CLOCK_FREQ / 2; // Base clock divisor is 2 so HCLK = (CLKPLL/2).
const PLK2_CLOCK_FREQ: u32 = HCLK_CLOCK_FREQ / 2; // APB2 clock divisor is 2 so PCLK2 = (HCLK/2).
const I2C_BAUD_RATE: u32 = 400000; // i2c baud rate is 400kbps.

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[non_exhaustive]
pub enum Error {
    _Bus,
    _ArbitrationLoss,
    _NoAcknowledge(NoAcknowledgeSource),
    _Overrun,
    _ReservedAddress,
    _Other,
}

impl Error {
    // pub(crate) fn nack_addr(self) -> Self {
    // match self {
    // Error::NoAcknowledge(NoAcknowledgeSource::Unknown) => {
    // Error::NoAcknowledge(NoAcknowledgeSource::Address)
    // }
    // e => e,
    // }
    // }
    // pub(crate) fn nack_data(self) -> Self {
    // match self {
    // Error::NoAcknowledge(NoAcknowledgeSource::Unknown) => {
    // Error::NoAcknowledge(NoAcknowledgeSource::Data)
    // }
    // e => e,
    // }
    // }
}

pub struct Mb9bf61xtI2c;

impl Mb9bf61xtI2c {
    pub fn new() -> Self {
        return Self {};
    }

    pub fn init(&mut self) -> Result<(), Error> {
        // MFS channel 6 register base address: 0x4003.8600
        let i2c6 = unsafe { mb9bf61xt::Peripherals::steal().MFS6 };

        // Serial Mode register (SMR) - set i2c mode (operation mode 4)
        i2c6.i2c_i2c_smr().write(|w| unsafe { w.bits(0x80) });

        // 7-bit Slave Address Mask Register (ISMK) - disable i2c controller
        i2c6.i2c_i2c_ismk().write(|w| unsafe { w.bits(0x7F) });

        // Serial Status Register (SSR)
        i2c6.i2c_i2c_ssr().write(|w| unsafe { w.bits(0) });

        // Baud Rate Generator registers (BGR0 and BGR1)
        // Baud rate formula: Reload Value = ((Bus Clock Frequency / Baud Rate) - 1)
        // NOTE: datasheet indicates that these must be handled as a single 16-bit write
        let reload_value = ((PLK2_CLOCK_FREQ / I2C_BAUD_RATE) - 1) as u16;
        i2c6.i2c_i2c_bgr()
            .write(|w| unsafe { w.bits(reload_value) });

        // // 7-bit Slave Address Register (ISBA) - disable slave mode
        i2c6.i2c_i2c_isba().write(|w| unsafe { w.bits(0) });

        // 7-bit Slave Address Mask Register (ISMK) - enable i2c interface operations
        i2c6.i2c_i2c_ismk().modify(|_, w| w.en().set_bit());

        // TODO - Enable the MFS6TX (i2c tx) and MFS6RX (i2c rx) interrupts
        //unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS6TX) };
        //unsafe { cortex_m::peripheral::NVIC::unmask(interrupt::MFS6RX) };

        Ok(())
    }

    fn prepare_write(&self, address: u8) -> Result<(), Error> {
        let i2c6 = unsafe { mb9bf61xt::Peripherals::steal().MFS6 };

        rprintln!(
            "INFO: [prepare_write] write to slave address {:#02x}, sending i2c start bit",
            address
        );

        // i2c Bus Control Register (IBCR) - clear to force idle
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0) });

        // Write slave address to TDR
        i2c6.i2c_i2c_tdr()
            .write(|w| unsafe { w.bits(address as u16) });

        // i2c Bus Control Register (IBCR) - enable master
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x80) });

        // TODO (needed?) - wait for transmit buffer to empty
        while i2c6.i2c_i2c_ssr().read().tdre() == false {}

        Ok(())
    }

    fn prepare_read(&self, address: u8) -> Result<(), Error> {
        let i2c6 = unsafe { mb9bf61xt::Peripherals::steal().MFS6 };

        rprintln!(
            "INFO: [prepare_read] write to slave address {:#02x}, sending i2c start bit",
            address
        );

        // i2c Bus Control Register (IBCR) - clear to force idle
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0) });

        // Write slave address to TDR
        i2c6.i2c_i2c_tdr()
            .write(|w| unsafe { w.bits(address as u16) });

        // i2c Bus Control Register (IBCR) - enable master
        i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x80) });

        Ok(())
    }

    fn write_bytes(&self, wb: &[u8]) -> Result<(), Error> {
        let i2c6 = unsafe { mb9bf61xt::Peripherals::steal().MFS6 };

        for op in wb.iter() {
            // Check for error condition
            match self.check_and_clear_error_flags() {
                Ok(()) => {}
                Err(e) => {
                    rprintln!("ERROR: failed to check and clear error flags [{:?}]", e);
                }
            };

            // Wait for the transmitter to become empty
            while i2c6.i2c_i2c_ssr().read().tdre() == false {}

            rprintln!(
                "INFO: [write_bytes] writing bytes to i2c device: {:#02x?}",
                wb
            );

            // Write a byte to TDR
            i2c6.i2c_i2c_tdr().write(|w| unsafe { w.bits(*op as u16) });

            // i2c Bus Control Register (IBCR).
            //i2c6.i2c_i2c_ibcr().write(|w| unsafe { w.bits(0x84) }); // Master enable, interrupt enable, clear interrupt.

            // Check for error condition
            match self.check_and_clear_error_flags() {
                Ok(()) => {}
                Err(e) => {
                    rprintln!("ERROR: failed to check and clear error flags [{:?}]", e);
                }
            };
        }
        Ok(())
    }

    fn read_bytes(&self, rb: &mut [u8]) -> Result<(), Error> {
        let i2c6 = unsafe { mb9bf61xt::Peripherals::steal().MFS6 };
        let mut i = 0;

        rprintln!("INFO: [read_bytes] reading bytes from i2c device");

        // Read as many bytes as are available (TODO - check for array overflow)
        while i2c6.i2c_i2c_ssr().read().rdrf() == true {
            // Check for error condition
            match self.check_and_clear_error_flags() {
                Ok(()) => {}
                Err(e) => {
                    rprintln!("ERROR: failed to check and clear error flags [{:?}]", e);
                }
            };
            // Read a byte from the i2c controller
            let data = i2c6.i2c_i2c_rdr().read().bits() as u8;

            rprintln!("INFO: received data 0x{:X}", data);
            rb[i] = data;
            i = i + 1;
        }
        //i2c6.i2c_i2c_ibcr().modify(|_, w| w.mss().clear_bit());
        //i2c6.i2c_i2c_ibcr().modify(|_, w| w.acke().clear_bit());
        //i2c6.i2c_i2c_ibcr().modify(|_, w| w.cnde().set_bit());

        // if i2c6.i2c_i2c_ibsr().read().rsc() == true {
        // clear restart condition
        //    i2c6.i2c_i2c_ibsr().modify(|_, w| w.rsc().clear_bit());
        //}
        Ok(())
    }

    fn write_wo_prepare(&mut self, _wb: &[u8]) -> Result<(), Error> {
        rprintln!("INFO: [write_wo_prepare] writing bytes to i2c device");
        Ok(())
    }

    fn read_wo_prepare(&mut self, rb: &mut [u8]) -> Result<(), Error> {
        let i2c6 = unsafe { mb9bf61xt::Peripherals::steal().MFS6 };
        let mut i = 0;

        rprintln!("INFO: [read_wo_prepare] reading bytes from i2c device");

        // Read as many bytes as are available (TODO - check for array overflow)
        while i2c6.i2c_i2c_ssr().read().rdrf() == true {
            // Check for error condition
            match self.check_and_clear_error_flags() {
                Ok(()) => {}
                Err(e) => {
                    rprintln!("ERROR: failed to check and clear error flags [{:?}]", e);
                }
            };
            let data = i2c6.i2c_i2c_rdr().read().bits() as u8;

            rprintln!("INFO: received data 0x{:X}", data);
            rb[i] = data;
            i = i + 1;
        }
        Ok(())
    }

    fn check_and_clear_error_flags(&self) -> Result<(), Error> {
        let i2c6 = unsafe { mb9bf61xt::Peripherals::steal().MFS6 };

        if i2c6.i2c_i2c_ssr().read().ore() == true {
            // overrun error
            rprintln!("ERROR: *** overrun error ***");
            // clear rx error interrupt
            i2c6.i2c_i2c_ssr().modify(|_, w| w.rec().set_bit());
        } else if i2c6.i2c_i2c_ibsr().read().spc() == true {
            rprintln!("INFO: stop condition");
            i2c6.i2c_i2c_ibsr().modify(|_, w| w.spc().clear_bit());

            // stop condition interrupt disable, interrupt disable
            //i2c6.i2c_i2c_ibcr().modify(|_, w| w.cnde().clear_bit());
            //i2c6.i2c_i2c_ibcr().modify(|_, w| w.inte().clear_bit());

            // clear IBSR:RACK
            i2c6.i2c_i2c_ismk().modify(|_, w| w.en().clear_bit());
            i2c6.i2c_i2c_ismk().modify(|_, w| w.en().set_bit());
        } else if i2c6.i2c_i2c_ibsr().read().rack() == true {
            // if sda is high (true), it's a nack
            rprintln!("INFO: *** nack received ***");
            // TODO
        } else if i2c6.i2c_i2c_ibcr().read().ber() == true {
            rprintln!("INFO: *** bus error ***");
            // TODO
        } else if i2c6.i2c_i2c_ibsr().read().al() == true {
            rprintln!("INFO: *** arbitration lost ***");
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
        let mut op_iter = operations.iter_mut();
        if let Some(mut prev_op) = op_iter.next() {
            // 1. generate START for operation
            let _ = match &prev_op {
                Operation::Read(_) => self.prepare_read(address),
                Operation::Write(_) => self.prepare_write(address),
            };
            for op in op_iter {
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
            // 4. here prev_op is the last command, use variations that will generate stop
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
    rprintln!("INFO: tx interrupt");
}

#[interrupt]
fn MFS6RX() {
    rprintln!("INFO: rx interrupt");
}
