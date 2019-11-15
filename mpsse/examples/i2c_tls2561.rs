use mpsse;
use std::io;
use std::io::{Error, ErrorKind};

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

#[allow(dead_code)]
#[repr(u8)]
enum Register {
    Control = 0x00u8,
    Timing = 0x01u8,
    ThreshLowLow = 0x02u8,
    ThreshLowHigh = 0x03u8,
    ThreshHighLow = 0x04u8,
    ThreshHighHigh = 0x05u8,
    Interrupt = 0x06u8,
    Crc = 0x08u8,
    Id = 0x0Au8,
    Data0Low = 0x0Cu8,
    Data0High = 0x0Du8,
    Data1Low = 0x0Eu8,
    Data1High = 0x0Fu8,
}

#[derive(PartialEq)]
enum Gain {
    X1,
    X16,
}

enum IntegTime {
    Time13ms,
    Time101ms,
    Time402ms,
}

enum ClipThresh {
    Clipping13ms = 4900,
    Clipping101ms = 37000,
    Clipping402ms = 65000,
}

const LUXSCALE: u32 = 14;
const RATIOSCALE: u32 = 9;
const CHSCALE: u32 = 1 << 10;
const CHSCALE_TINT0: u32 = CHSCALE * 322 / 11;
const CHSCALE_TINT1: u32 = CHSCALE * 322 / 81;

// CS package values
const K1C: u32 = 0x0043; // 0.130 * 2^RATIO_SCALE
const B1C: u32 = 0x0204; // 0.0315 * 2^LUX_SCALE
const M1C: u32 = 0x01ad; // 0.0262 * 2^LUX_SCALE
const K2C: u32 = 0x0085; // 0.260 * 2^RATIO_SCALE
const B2C: u32 = 0x0228; // 0.0337 * 2^LUX_SCALE
const M2C: u32 = 0x02c1; // 0.0430 * 2^LUX_SCALE
const K3C: u32 = 0x00c8; // 0.390 * 2^RATIO_SCALE
const B3C: u32 = 0x0253; // 0.0363 * 2^LUX_SCALE
const M3C: u32 = 0x0363; // 0.0529 * 2^LUX_SCALE
const K4C: u32 = 0x010a; // 0.520 * 2^RATIO_SCALE
const B4C: u32 = 0x0282; // 0.0392 * 2^LUX_SCALE
const M4C: u32 = 0x03df; // 0.0605 * 2^LUX_SCALE
const K5C: u32 = 0x014d; // 0.65 * 2^RATIO_SCALE
const B5C: u32 = 0x0177; // 0.0229 * 2^LUX_SCALE
const M5C: u32 = 0x01dd; // 0.0291 * 2^LUX_SCALE
const K6C: u32 = 0x019a; // 0.80 * 2^RATIO_SCALE
const B6C: u32 = 0x0101; // 0.0157 * 2^LUX_SCALE
const M6C: u32 = 0x0127; // 0.0180 * 2^LUX_SCALE
const K7C: u32 = 0x029a; // 1.3 * 2^RATIO_SCALE
const B7C: u32 = 0x0037; // 0.00338 * 2^LUX_SCALE
const M7C: u32 = 0x002b; // 0.00260 * 2^LUX_SCALE
const K8C: u32 = 0x029a; // 1.3 * 2^RATIO_SCALE
const B8C: u32 = 0x0000; // 0.000 * 2^LUX_SCALE
const M8C: u32 = 0x0000; // 0.000 * 2^LUX_SCALE

struct CommandCode {
    code: u8,
}

#[allow(dead_code)]
impl CommandCode {
    pub fn new(reg: Register) -> Self {
        Self { code: reg as u8 }
    }
    pub fn command_bit(&mut self) -> &mut Self {
        self.code |= 0x80;
        self
    }
    pub fn clear_bit(&mut self) -> &mut Self {
        self.code |= 0x40;
        self
    }
    pub fn word_bit(&mut self) -> &mut Self {
        self.code |= 0x20;
        self
    }
    pub fn block_bit(&mut self) -> &mut Self {
        self.code |= 0x10;
        self
    }
    pub fn build(&self) -> u8 {
        self.code
    }
}

fn power(i2c: &mut mpsse::Mpsse, on: bool) -> io::Result<()> {
    let addr = SlaveAddr::new(0x39);
    let cmd = CommandCode::new(Register::Control).command_bit().build();
    println!("command = {:x}", cmd);

    let power = if on { 0x03 as u8 } else { 0x00 as u8 };

    i2c.start()?;

    let mut wdata: [u8; 3] = [addr.w(), cmd, power];
    i2c.write(&mut wdata)?;
    if i2c.get_ack() == mpsse::I2cAck::Nack {
        dbg!("faild to get ACK");
        return Err(Error::new(ErrorKind::ConnectionAborted, "NACK"));
    }

    i2c.stop()?;

    println!("power {}", if on { "on" } else { "off" });

    Ok(())
}

fn read_id(i2c: &mut mpsse::Mpsse) -> io::Result<()> {
    let addr = SlaveAddr::new(0x39);
    let cmd = CommandCode::new(Register::Id).command_bit().build();
    println!("command = {:x}", cmd);

    i2c.start()?;
    i2c.send_acks();

    let mut wdata: [u8; 2] = [addr.w(), cmd];
    i2c.write(&mut wdata)?;
    if i2c.get_ack() == mpsse::I2cAck::Nack {
        dbg!("faild to get ACK");
        return Err(Error::new(ErrorKind::ConnectionAborted, "NACK"));
    }

    i2c.start()?;
    i2c.write(&mut (addr.r().to_le_bytes()))?;
    if i2c.get_ack() == mpsse::I2cAck::Nack {
        dbg!("faild to get ACK");
        return Err(Error::new(ErrorKind::ConnectionAborted, "NACK"));
    }

    let mut id = [0x00u8; 1];
    i2c.read(&mut id)?;

    // send NACK
    i2c.send_nacks();
    let mut _dummy: [u8; 1] = [0];
    let _ = i2c.read(&mut _dummy);

    i2c.stop()?;

    println!("id = {:x}", id[0]);

    Ok(())
}

#[allow(dead_code)]
fn reg_to_gain_and_integ(tim_reg: u8) -> (Gain, IntegTime) {
    (
        if tim_reg & 0x10 != 0 {
            Gain::X16
        } else {
            Gain::X1
        },
        match tim_reg & 0x03 {
            0x00 => IntegTime::Time13ms,
            0x01 => IntegTime::Time101ms,
            0x02 => IntegTime::Time402ms,
            _ => IntegTime::Time402ms,
        },
    )
}

fn caluculate_lux(broadband: u16, ir: u16, gain: Gain, integ: IntegTime) -> u32 {
    let clip_thresh: u16 = match integ {
        IntegTime::Time13ms => ClipThresh::Clipping13ms,
        IntegTime::Time101ms => ClipThresh::Clipping101ms,
        IntegTime::Time402ms => ClipThresh::Clipping402ms,
    } as u16;

    if broadband > clip_thresh || ir > clip_thresh {
        return 65536;
    }

    let ch_scale: u32 = match integ {
        IntegTime::Time13ms => CHSCALE_TINT0,
        IntegTime::Time101ms => CHSCALE_TINT1,
        IntegTime::Time402ms => CHSCALE,
    } * if gain == Gain::X16 { 16 } else { 1 };

    let ch0: u32 = (broadband as u32 * ch_scale) / CHSCALE;
    let ch1: u32 = (ir as u32 * ch_scale) / CHSCALE;

    let ratio1: u32 = if ch0 == 0 {
        0
    } else {
        (ch1 << (RATIOSCALE + 1)) / ch0
    };
    let ratio = (ratio1 + 1) >> 1;

    let (b, m) = if ratio <= K1C {
        (B1C, M1C)
    } else if ratio <= K2C {
        (B2C, M2C)
    } else if ratio <= K3C {
        (B3C, M3C)
    } else if ratio <= K4C {
        (B4C, M4C)
    } else if ratio <= K5C {
        (B5C, M5C)
    } else if ratio <= K6C {
        (B6C, M6C)
    } else if ratio <= K7C {
        (B7C, M7C)
    } else if ratio > K8C {
        (B8C, M8C)
    } else {
        (B8C, M8C)
    };

    let mut temp0: i32 = (ch0 * b) as i32 - (ch1 * m) as i32;
    if temp0 < 0 {
        temp0 = 0;
    }
    let temp1: u32 = temp0 as u32 + (1 << (LUXSCALE - 1));
    let lux = temp1 >> LUXSCALE;

    lux
}

fn read_data(i2c: &mut mpsse::Mpsse) -> io::Result<(u16, u16)> {
    let addr = SlaveAddr::new(0x39);
    let cmd = CommandCode::new(Register::Data0Low)
        .command_bit()
        .block_bit()
        .build();
    println!("command = {:x}", cmd);

    i2c.start()?;
    i2c.send_acks();

    let mut wdata: [u8; 2] = [addr.w(), cmd];
    i2c.write(&mut wdata)?;
    if i2c.get_ack() == mpsse::I2cAck::Nack {
        dbg!("failed to get ACK");
        return Err(Error::new(ErrorKind::ConnectionAborted, "NACK"));
    }

    i2c.start()?;
    i2c.write(&mut (addr.r().to_le_bytes()))?;
    if i2c.get_ack() == mpsse::I2cAck::Nack {
        dbg!("failed to get ACK");
        return Err(Error::new(ErrorKind::ConnectionAborted, "NACK"));
    }

    let mut data = [0x00u8; 4];
    i2c.read(&mut data)?;

    // send NACK
    i2c.send_nacks();
    let mut _dummy: [u8; 1] = [0];
    let _ = i2c.read(&mut _dummy);

    i2c.stop()?;

    println!("data = {:x?}", data);

    Ok(
        (
            data[0] as u16 | ((data[1] as u16) << 8),
            data[2] as u16 | ((data[3] as u16) << 8),
        )
    )
}

fn main() -> io::Result<()> {
    let mut i2c = mpsse::Mpsse::new(mpsse::Mode::I2C, 400_000, mpsse::Endianess::MSB)?;

    power(&mut i2c, true)?;
    read_id(&mut i2c)?;
    let (data0, data1) = read_data(&mut i2c)?;
    let lux = caluculate_lux(data0, data1, Gain::X1, IntegTime::Time402ms);

    println!("lux = {}", lux);

    Ok(())
}
