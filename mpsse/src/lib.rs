#![allow(dead_code)]

use libc;
use mpsse_sys;
use std::ffi::CStr;
use std::io;
use std::io::{Error, ErrorKind};
use std::ptr;

const MPSSE_OK: i32 = mpsse_sys::MPSSE_OK as i32;
const MPSSE_FAIL: i32 = mpsse_sys::MPSSE_FAIL as i32;

pub enum Mode {
    SPI0 = mpsse_sys::modes_SPI0 as isize,
    SPI1 = mpsse_sys::modes_SPI1 as isize,
    SPI2 = mpsse_sys::modes_SPI2 as isize,
    SPI3 = mpsse_sys::modes_SPI3 as isize,
    I2C = mpsse_sys::modes_I2C as isize,
    GPIO = mpsse_sys::modes_GPIO as isize,
    BITBANG = mpsse_sys::modes_BITBANG as isize,
}

pub enum Endianess {
    MSB = mpsse_sys::MSB as isize,
    LSB = mpsse_sys::LSB as isize,
}

pub struct Mpsse {
    context: *mut mpsse_sys::mpsse_context,
}

#[derive(Debug, PartialEq)]
pub enum PinLevel {
    Low = 0,
    High,
}

pub mod gpio_pin {
    pub const L0: u32 = mpsse_sys::gpio_pins_GPIOL0;
    pub const L1: u32 = mpsse_sys::gpio_pins_GPIOL1;
    pub const L2: u32 = mpsse_sys::gpio_pins_GPIOL2;
    pub const L3: u32 = mpsse_sys::gpio_pins_GPIOL3;
    pub const H0: u32 = mpsse_sys::gpio_pins_GPIOH0;
    pub const H1: u32 = mpsse_sys::gpio_pins_GPIOH1;
    pub const H2: u32 = mpsse_sys::gpio_pins_GPIOH2;
    pub const H3: u32 = mpsse_sys::gpio_pins_GPIOH3;
    pub const H4: u32 = mpsse_sys::gpio_pins_GPIOH4;
    pub const H5: u32 = mpsse_sys::gpio_pins_GPIOH5;
    pub const H6: u32 = mpsse_sys::gpio_pins_GPIOH6;
    pub const H7: u32 = mpsse_sys::gpio_pins_GPIOH7;
}

pub const CLOCK_100_KHZ: u32 = 100000;
pub const CLOCK_400_KHZ: u32 = 400000;
pub const CLOCK_1_MHZ: u32 = 1000000;
pub const CLOCK_2_MHZ: u32 = 2000000;
pub const CLOCK_5_MHZ: u32 = 5000000;
pub const CLOCK_6_MHZ: u32 = 6000000;
pub const CLOCK_10_MHZ: u32 = 10000000;
pub const CLOCK_12_MHZ: u32 = 12000000;
pub const CLOCK_15_MHZ: u32 = 15000000;
pub const CLOCK_30_MHZ: u32 = 30000000;
pub const CLOCK_60_MHZ: u32 = 60000000;

pub enum I2cAck {
    Ack = mpsse_sys::i2c_ack_ACK as isize,
    Nack = mpsse_sys::i2c_ack_NACK as isize,
}

pub enum Interface {
    Any = mpsse_sys::interface_IFACE_ANY as isize,
    A = mpsse_sys::interface_IFACE_A as isize,
    B = mpsse_sys::interface_IFACE_B as isize,
    C = mpsse_sys::interface_IFACE_C as isize,
    D = mpsse_sys::interface_IFACE_D as isize,
}

impl Mpsse {
    pub fn new(mode: Mode, freq: u32, endianess: Endianess) -> io::Result<Self> {
        let mut mpsse = Self {
            context: unsafe { mpsse_sys::MPSSE(mode as u32, freq as i32, endianess as i32) },
        };

        if mpsse.context.is_null() {
            Err(Error::new(
                ErrorKind::NotFound,
                "MPSSE cannot found supported device",
            ))
        } else if unsafe { (*mpsse.context).open } == 0 {
            Err(Error::new(ErrorKind::NotFound, mpsse.error_string()))
        } else {
            Ok(mpsse)
        }
    }

    // pub fn open(vid: i32, pid: i32, mode: Mode, freq: u32,
    //     endianess: Endianess,
    //     interface: Interface,
    //     description: Option<&str>,
    //     serial: Option<&str>,
    // ) -> io::Result<Self> {
    //     Ok( Self { context: ptr::null_mut() } )
    // }
    // pub fn open_index(vid: i32, pid: i32, mode: Mode, freq: u32,
    //     endianess: Endianess,
    //     interface: Interface,
    //     description: Option<&str>,
    //     serial: Option<&str>,
    //     index: i32,
    // ) -> io::Result<Self> {
    //     Ok( Self { context: ptr::null_mut() })
    // }

    fn error_string(&mut self) -> String {
        let c_str: &CStr = unsafe { CStr::from_ptr(mpsse_sys::ErrorString(self.context)) };
        c_str.to_str().unwrap_or("ffi internal error").to_owned()
    }

    pub fn set_mode(&mut self, endianess: Endianess) -> io::Result<()> {
        if unsafe { mpsse_sys::SetMode(self.context, endianess as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn enable_bit_mode(&mut self, enable_transfer: bool) {
        let tf: i32 = if enable_transfer { 1 } else { 0 };
        unsafe { mpsse_sys::EnableBitmode(self.context, tf) };
    }

    pub fn set_clock(&mut self, freq: u32) -> io::Result<()> {
        if unsafe { mpsse_sys::SetClock(self.context, freq) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn get_clock(&mut self) -> u32 {
        unsafe { mpsse_sys::GetClock(self.context) as u32 }
    }

    pub fn get_vid(&mut self) -> i32 {
        unsafe { mpsse_sys::GetVid(self.context) }
    }

    pub fn get_pid(&mut self) -> i32 {
        unsafe { mpsse_sys::GetPid(self.context) }
    }

    pub fn get_description(&mut self) -> String {
        let c_str: &CStr = unsafe { CStr::from_ptr(mpsse_sys::GetDescription(self.context)) };
        c_str.to_str().unwrap_or("ffi internal error").to_owned()
    }

    pub fn set_loopback(&mut self, enable: bool) -> io::Result<()> {
        let enable: i32 = if enable { 1 } else { 0 };
        if unsafe { mpsse_sys::SetLoopback(self.context, enable) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn set_cs_idle(&mut self, idle_state_level: PinLevel) {
        let idle: i32 = if idle_state_level == PinLevel::High {
            1
        } else {
            0
        };
        unsafe { mpsse_sys::SetCSIdle(self.context, idle) };
    }

    pub fn start(&mut self) -> io::Result<()> {
        if unsafe { mpsse_sys::Start(self.context) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn write(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if unsafe { mpsse_sys::Write(self.context, buf.as_mut_ptr() as *mut i8, buf.len() as i32) }
            == MPSSE_OK
        {
            Ok(buf.len())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn stop(&mut self) -> io::Result<()> {
        if unsafe { mpsse_sys::Stop(self.context) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn get_ack(&mut self) -> I2cAck {
        if unsafe { mpsse_sys::GetAck(self.context) } == I2cAck::Ack as i32 {
            I2cAck::Ack
        } else {
            I2cAck::Nack
        }
    }

    pub fn set_ack(&mut self, ack: I2cAck) {
        unsafe { mpsse_sys::SetAck(self.context, ack as i32) };
    }

    pub fn send_acks(&mut self) {
        unsafe { mpsse_sys::SendAcks(self.context) };
    }

    pub fn send_nacks(&mut self) {
        unsafe { mpsse_sys::SendNacks(self.context) };
    }

    pub fn flush_after_read(&mut self, enable_flush: bool) {
        let flush: i32 = if enable_flush { 1 } else { 0 };
        unsafe { mpsse_sys::FlushAfterRead(self.context, flush) };
    }

    pub fn tristate(&mut self) -> io::Result<()> {
        if unsafe { mpsse_sys::Tristate(self.context) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn version() -> u8 {
        unsafe { mpsse_sys::Version() as u8 }
    }

    pub fn pin_high(&mut self, pin: u32) -> io::Result<()> {
        if unsafe { mpsse_sys::PinHigh(self.context, pin as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn pin_low(&mut self, pin: u32) -> io::Result<()> {
        if unsafe { mpsse_sys::PinLow(self.context, pin as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn pin_state(&mut self, pin: u32) -> PinLevel {
        if unsafe { mpsse_sys::PinState(self.context, pin as i32, -1) } == MPSSE_OK {
            PinLevel::Low
        } else {
            PinLevel::High
        }
    }

    pub fn set_direction(&mut self, direction: u8) -> io::Result<()> {
        if unsafe { mpsse_sys::SetDirection(self.context, direction) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn write_bits(&mut self, bits: u8, size: usize) -> io::Result<()> {
        if unsafe { mpsse_sys::WriteBits(self.context, bits as i8, size as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn read_bits(&mut self, size: usize) -> u8 {
        unsafe { mpsse_sys::ReadBits(self.context, size as i32) as u8 }
    }

    pub fn write_pins(&mut self, data: u8) -> io::Result<()> {
        if unsafe { mpsse_sys::WritePins(self.context, data) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn reqd_pins(&mut self) -> u8 {
        unsafe { mpsse_sys::ReadPins(self.context) as u8 }
    }

    pub fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let raw_ptr: *mut u8 =
            unsafe { mpsse_sys::Read(self.context, buf.len() as i32) as *mut u8 };

        if raw_ptr.is_null() {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        } else {
            unsafe {
                ptr::copy_nonoverlapping(raw_ptr, buf.as_mut_ptr(), buf.len());
                libc::free(raw_ptr as *mut libc::c_void);
            }
            Ok(buf.len())
        }
    }

    pub fn transfer<'a>(&mut self, buf: &'a mut [u8]) -> io::Result<&'a [u8]> {
        let raw_ptr: *mut u8 = unsafe {
            mpsse_sys::Transfer(self.context, buf.as_mut_ptr() as *mut i8, buf.len() as i32)
                as *mut u8
        };

        if raw_ptr.is_null() {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        } else {
            unsafe {
                ptr::copy_nonoverlapping(raw_ptr, buf.as_mut_ptr(), buf.len());
                libc::free(raw_ptr as *mut libc::c_void);
            }
            Ok(buf)
        }
    }

    pub fn fast_write(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if unsafe {
            mpsse_sys::FastWrite(self.context, buf.as_mut_ptr() as *mut i8, buf.len() as i32)
        } == MPSSE_OK
        {
            Ok(buf.len())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn fast_read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if unsafe {
            mpsse_sys::FastRead(self.context, buf.as_mut_ptr() as *mut i8, buf.len() as i32)
        } == MPSSE_OK
        {
            Ok(buf.len())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    pub fn fast_transfer<'a>(&mut self, buf: &'a mut [u8]) -> io::Result<&'a [u8]> {
        let mut wbuf = buf.to_vec();
        if unsafe {
            mpsse_sys::FastTransfer(
                self.context,
                wbuf.as_mut_ptr() as *mut i8,
                buf.as_mut_ptr() as *mut i8,
                buf.len() as i32,
            )
        } == MPSSE_OK
        {
            Ok(buf)
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }
}

impl Drop for Mpsse {
    fn drop(&mut self) {
        println!("drop! close");
        unsafe { mpsse_sys::Close(self.context) };
    }
}
