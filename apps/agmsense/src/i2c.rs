use embedded_hal::i2c::Operation;
use mb9bf61xt;

use crate::println;

// NOTE: the fm3 datasheet indicates that in I2C mode (operation mode 4), the bus clock can't be operated at less than
// 8MHz and the I2C clock no faster than 400 kbps.
const MASTER_CLOCK_FREQ: u32 = 144000000; // Master clock (CLKPLL) is the PLL clock (see main.rs).
const BASE_CLOCK_FREQ: u32 = MASTER_CLOCK_FREQ / 1; // Base clock divisor is 1 so HCLK = (CLKPLL/1).
const PLK2_CLOCK_FREQ: u32 = BASE_CLOCK_FREQ / 2; // APB2 clock divisor is 2 so PCLK2 = (HCLK/2).
const BAUD_RATE: u32 = 400000; // I2C baud rate is 400kbps.
const MPU9250A_I2C_ADDRESS: u8 = 0x68; // MPU-9250A I2C bus address.

pub struct Mb9bf61xtI2c;

impl Mb9bf61xtI2c {
    pub fn new() -> Self {
        return Self {};
    }

    pub fn init_i2c(&mut self) {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        // Serial Mode register (SMR).
        i2c6.i2c_i2c_smr()
            .modify(|_, w| unsafe { w.md().bits(0x80) }); // I2C mode (operation mode 4).
        i2c6.i2c_i2c_smr().modify(|_, w| w.wucr().clear_bit()); // Disable wake-up control.
        i2c6.i2c_i2c_smr().modify(|_, w| w.rie().clear_bit()); // Disable receive interrupt.
        i2c6.i2c_i2c_smr().modify(|_, w| w.tie().clear_bit()); // Disable transmit interrupt.

        // Baud Rate Generator registers (BGR0 and BGR1).
        // Baud rate formula: Reload Value = ((Bus Clock Frequency / Baud Rate) - 1).
        // NOTE: datasheet indicates that these must be handled as a single 16-bit write.
        let reload_value = ((PLK2_CLOCK_FREQ / BAUD_RATE) - 1) as u16;
        i2c6.i2c_i2c_bgr()
            .write(|w| unsafe { w.bits(reload_value) });

        // FIFO Control registers (FCR0 and FCR1).

        // FIFO Byte register (FBYTE1 and FBYTE2) - set the receive FIFO level that generates a receive interrupt.
        // NOTE: a read-modify-write cannot be used for this register.

        // 7-bit Slave Address Register (ISBA).
        // Set MPU-9250A I2C slave address.
        i2c6.i2c_i2c_isba()
            .modify(|_, w| unsafe { w.sa().bits(MPU9250A_I2C_ADDRESS) });

        // 7-bit Slave Address Mask Register (ISMK).
        // Set I2C slave address mask.
        i2c6.i2c_i2c_ismk()
            .modify(|_, w| unsafe { w.sm().bits(0x7F) });
        i2c6.i2c_i2c_ismk().modify(|_, w| w.en().set_bit()); // Enable I2C interface operations.
    }

    // NOTE: this is a blocking call.
    pub fn read_i2c_bytes(address: u8, buf: &mut [u8]) -> usize {
        let p = unsafe { mb9bf61xt::Peripherals::steal() };
        let i2c6 = p.MFS6;

        let register_read: u16 = address as u16 | 0x1; // Lowest bit indicates a read operation.

        // Write the send (register) address to the TDR.
        i2c6.i2c_i2c_tdr()
            .write(|w| unsafe { w.bits(register_read) });

        // Set Master mode.
        i2c6.i2c_i2c_ibcr().modify(|_, w| w.mss().set_bit());

        // Wait for the interrupt bit to go high.
        while i2c6.i2c_i2c_ibcr().read().int() == false {}

        // Check for bus error.
        if i2c6.i2c_i2c_ibcr().read().ber() == true {
            // TODO: implement bus error handler.
            println!("ERROR: I2C bus error.");
        }

        // Check for arbitration lost signal.
        if i2c6.i2c_i2c_ibcr().read().act_scc() == false {
            // TODO: implement arbitration lost processing.
            println!("ERROR: I2C bus arbitration lost.");
        }

        // Check for reserved address access.
        if i2c6.i2c_i2c_ibsr().read().rsa() == true {
            // TODO: implement reserved address handler.
            println!("ERROR: I2C bus reserved address.");
        }

        // Check for RACK acknowledge.
        if i2c6.i2c_i2c_ibsr().read().rack() == true {
            // TODO: implement
            println!("ERROR: I2C RACK acknowledge.");
        }

        // Check for receive.
        if i2c6.i2c_i2c_ibsr().read().trx() == false {
            // Check first byte bit.
            if i2c6.i2c_i2c_ibsr().read().fbt() == false {
                println!("INFO: Found data on I2C bus.");
            }
        }
        // TODO: handle multiple bytes.
        for b in buf {
            *b = i2c6.i2c_i2c_rdr().read().bits() as u8;
        }

        // Set waiting.
        i2c6.i2c_i2c_ibcr().modify(|_, w| w.wsel().set_bit());

        // Send an ACK.
        i2c6.i2c_i2c_ibcr().modify(|_, w| w.acke().clear_bit());
        i2c6.i2c_i2c_ibcr().modify(|_, w| w.mss().clear_bit());
        i2c6.i2c_i2c_ibcr().modify(|_, w| w.acke().set_bit());

        // Clear interrupt flag.
        i2c6.i2c_i2c_ibcr().modify(|_, w| w.int().clear_bit());

        // TODO: return size.
        return 0;
    }
    // NOTE: this is a blocking call.
    pub fn write_i2c_bytes(address: u8, _buf: &mut &[u8]) -> usize {
        println!("INFO: I2C write to address 0x{:x}", address);
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
