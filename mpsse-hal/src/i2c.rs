use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use mpsse;
use std::io;
use std::result;

pub type Result<T> = result::Result<T, io::Error>;

struct SlaveAddr {
    addr: u8,
}

impl SlaveAddr {
    pub fn new(addr: u8) -> Self {
        Self { addr: addr << 1 }
    }
    pub fn w(&self) -> u8 {
        self.addr
    }
    pub fn r(&self) -> u8 {
        self.addr | 1
    }
}

pub struct I2c<'s> {
    ctx: mpsse::Mpsse<'s>,
}

impl<'s> I2c<'s> {
    pub fn new() -> Result<Self> {
        Ok(Self {
            ctx: mpsse::Mpsse::new(mpsse::Mode::I2C, 400_000, mpsse::Endianess::MSB)?,
        })
    }
    pub fn generate_from(ctx: mpsse::Mpsse<'s>) -> Result<Self> {
        Ok(Self {
            ctx,
        })
    }
    pub fn get_clock(&mut self) -> u32 {
        self.ctx.get_clock()
    }
    pub fn set_clock(&mut self, freq: u32) -> Result<()> {
        self.ctx.set_clock(freq)
    }
}

impl<'s> Read for I2c<'s> {
    type Error = io::Error;

    /// Reads enough bytes from slave with `address` to fill `buffer`
    ///
    /// # I2C Events (contract)
    ///
    /// ``` text
    /// Master: ST SAD+R        MAK    MAK ...    NMAK SP
    /// Slave:           SAK B0     B1     ... BN
    /// ```
    ///
    /// Where
    ///
    /// - `ST` = start condition
    /// - `SAD+R` = slave address followed by bit 1 to indicate reading
    /// - `SAK` = slave acknowledge
    /// - `Bi` = ith byte of data
    /// - `MAK` = master acknowledge
    /// - `NMAK` = master no acknowledge
    /// - `SP` = stop condition
    fn read(&mut self, address: u8, buffer: &mut [u8]) -> Result<()> {
        let slave_addr = SlaveAddr::new(address);

        self.ctx.start()?;
        self.ctx.send_acks();
        self.ctx.write(&mut (slave_addr.r().to_le_bytes()))?;
        if self.ctx.get_ack() == mpsse::I2cAck::Nack {
            //dbg!("faild to get ACK");
            return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "NACK"));
        }

        self.ctx.read(buffer)?;
        self.ctx.stop()?;

        Ok(())
    }
}

impl<'s> Write for I2c<'s> {
    type Error = io::Error;

    /// Sends bytes to slave with address `addr`
    ///
    /// # I2C Events (contract)
    ///
    /// ``` text
    /// Master: ST SAD+W     B0     B1     ... BN     SP
    /// Slave:           SAK    SAK    SAK ...    SAK
    /// ```
    ///
    /// Where
    ///
    /// - `ST` = start condition
    /// - `SAD+W` = slave address followed by bit 0 to indicate writing
    /// - `SAK` = slave acknowledge
    /// - `Bi` = ith byte of data
    /// - `SP` = stop condition
    fn write(&mut self, address: u8, bytes: &[u8]) -> Result<()> {
        let slave_addr = SlaveAddr::new(address);
        let mut w_bytes = bytes.to_vec();

        self.ctx.start()?;
        self.ctx.write(&mut (slave_addr.w().to_le_bytes()))?;
        if self.ctx.get_ack() == mpsse::I2cAck::Nack {
            //dbg!("faild to get ACK");
            return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "NACK"));
        }

        self.ctx.write(&mut w_bytes)?;
        self.ctx.stop()?;

        Ok(())
    }
}

impl<'s> WriteRead for I2c<'s> {
    type Error = io::Error;

    /// Sends bytes to slave with address `addr` and then reads enough bytes to fill `buffer` *in a
    /// single transaction*
    ///
    /// # I2C Events (contract)
    ///
    /// ``` text
    /// Master: ST SAD+W     O0     O1     ... OM     SR SAD+R        MAK    MAK ...    NMAK SP
    /// Slave:           SAK    SAK    SAK ...    SAK          SAK I0     I1     ... IN
    /// ```
    ///
    /// Where
    ///
    /// - `ST` = start condition
    /// - `SAD+W` = slave address followed by bit 0 to indicate writing
    /// - `SAK` = slave acknowledge
    /// - `Oi` = ith outgoing byte of data
    /// - `SR` = repeated start condition
    /// - `SAD+R` = slave address followed by bit 1 to indicate reading
    /// - `Ii` = ith incoming byte of data
    /// - `MAK` = master acknowledge
    /// - `NMAK` = master no acknowledge
    /// - `SP` = stop condition
    fn write_read(
        &mut self,
        address: u8,
        bytes: &[u8],
        buffer: &mut [u8],
    ) -> Result<()> {
        let slave_addr = SlaveAddr::new(address);
        let mut w_bytes = bytes.to_vec();

        self.ctx.start()?;
        self.ctx.write(&mut (slave_addr.w().to_le_bytes()))?;
        if self.ctx.get_ack() == mpsse::I2cAck::Nack {
            //dbg!("faild to get ACK");
            return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "NACK"));
        }
        self.ctx.write(&mut w_bytes)?;

        self.ctx.send_acks();
        self.ctx.start()?;
        self.ctx.write(&mut (slave_addr.r().to_le_bytes()))?;
        if self.ctx.get_ack() == mpsse::I2cAck::Nack {
            //dbg!("faild to get ACK");
            return Err(io::Error::new(io::ErrorKind::ConnectionAborted, "NACK"));
        }
        self.ctx.read(buffer)?;
        self.ctx.stop()?;

        Ok(())
    }
}
