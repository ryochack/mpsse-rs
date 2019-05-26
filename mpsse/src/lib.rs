//! Wrappers for libmpsse for FTDI's FT2232 familly of USB chips.
#![allow(dead_code)]

use libc;
use mpsse_sys;
use std::ffi::CStr;
use std::io;
use std::io::{Error, ErrorKind};
use std::ptr;

const MPSSE_OK: i32 = mpsse_sys::MPSSE_OK as i32;
const MPSSE_FAIL: i32 = mpsse_sys::MPSSE_FAIL as i32;

/// Enumeration of MPSSE modes.
#[derive(Clone, Copy)]
pub enum Mode {
    SPI0 = mpsse_sys::modes_SPI0 as isize,
    SPI1 = mpsse_sys::modes_SPI1 as isize,
    SPI2 = mpsse_sys::modes_SPI2 as isize,
    SPI3 = mpsse_sys::modes_SPI3 as isize,
    I2C = mpsse_sys::modes_I2C as isize,
    GPIO = mpsse_sys::modes_GPIO as isize,
    BITBANG = mpsse_sys::modes_BITBANG as isize,
}

/// Enumeration of MPSSE byte orders.
#[derive(Clone, Copy)]
pub enum Endianess {
    MSB = mpsse_sys::MSB as isize,
    LSB = mpsse_sys::LSB as isize,
}

/// Mpsse builder.
pub struct MpsseBuilder<'s> {
    vid: i32,
    pid: i32,
    mode: Mode,
    freq: u32,
    endianess: Endianess,
    interface: FtdiInterface,
    description: Option<&'s str>,
    serial: Option<&'s str>,
    index: i32,
}

/// Mpsse structure.
/// Mpsse has an instance of libmpsse
pub struct Mpsse<'s> {
    context: *mut mpsse_sys::mpsse_context,
    description: Option<&'s str>,
    serial: Option<&'s str>,
}

/// The GPIO pin level
#[derive(Clone, Copy)]
pub enum PinLevel {
    Low = 0,
    High,
}

/// The input/output direction of all pins. For use in BITBANG mode only.
#[derive(Clone, Copy)]
pub enum Direction {
    Input = 0,
    Output,
}

pub const GPIOL0: u32 = mpsse_sys::gpio_pins_GPIOL0;
pub const GPIOL1: u32 = mpsse_sys::gpio_pins_GPIOL1;
pub const GPIOL2: u32 = mpsse_sys::gpio_pins_GPIOL2;
pub const GPIOL3: u32 = mpsse_sys::gpio_pins_GPIOL3;
pub const GPIOH0: u32 = mpsse_sys::gpio_pins_GPIOH0;
pub const GPIOH1: u32 = mpsse_sys::gpio_pins_GPIOH1;
pub const GPIOH2: u32 = mpsse_sys::gpio_pins_GPIOH2;
pub const GPIOH3: u32 = mpsse_sys::gpio_pins_GPIOH3;
pub const GPIOH4: u32 = mpsse_sys::gpio_pins_GPIOH4;
pub const GPIOH5: u32 = mpsse_sys::gpio_pins_GPIOH5;
pub const GPIOH6: u32 = mpsse_sys::gpio_pins_GPIOH6;
pub const GPIOH7: u32 = mpsse_sys::gpio_pins_GPIOH7;

pub const CLOCK_100_KHZ: u32 = 100_000;
pub const CLOCK_400_KHZ: u32 = 400_000;
pub const CLOCK_1_MHZ: u32 = 1_000_000;
pub const CLOCK_2_MHZ: u32 = 2_000_000;
pub const CLOCK_5_MHZ: u32 = 5_000_000;
pub const CLOCK_6_MHZ: u32 = 6_000_000;
pub const CLOCK_10_MHZ: u32 = 10_000_000;
pub const CLOCK_12_MHZ: u32 = 12_000_000;
pub const CLOCK_15_MHZ: u32 = 15_000_000;
pub const CLOCK_30_MHZ: u32 = 30_000_000;
pub const CLOCK_60_MHZ: u32 = 60_000_000;

#[derive(Clone, Copy)]
pub enum I2cAck {
    Ack = mpsse_sys::i2c_ack_ACK as isize,
    Nack = mpsse_sys::i2c_ack_NACK as isize,
}

/// FTDI Interface
#[derive(Clone, Copy)]
pub enum FtdiInterface {
    Any = mpsse_sys::interface_IFACE_ANY as isize,
    A = mpsse_sys::interface_IFACE_A as isize,
    B = mpsse_sys::interface_IFACE_B as isize,
    C = mpsse_sys::interface_IFACE_C as isize,
    D = mpsse_sys::interface_IFACE_D as isize,
}

pub struct SupportedDevice {
    vid: i32,
    pid: i32,
    description: &'static str,
}

pub static SUPPORTED_DEVICES: &'static [SupportedDevice] = &[
    SupportedDevice {
        vid: 0x0403,
        pid: 0x6010,
        description: "FT2232 Future Technology Devices International, Ltd",
    },
    SupportedDevice {
        vid: 0x0403,
        pid: 0x6011,
        description: "FT4232 Future Technology Devices International, Ltd",
    },
    SupportedDevice {
        vid: 0x0403,
        pid: 0x6014,
        description: "FT232H Future Technology Devices International, Ltd",
    },
    /* These devices are based on FT2232 chips, but have not been tested. */
    SupportedDevice {
        vid: 0x0403,
        pid: 0x8878,
        description: "Bus Blaster v2 (channel A)",
    },
    SupportedDevice {
        vid: 0x0403,
        pid: 0x8879,
        description: "Bus Blaster v2 (channel B)",
    },
    SupportedDevice {
        vid: 0x0403,
        pid: 0xBDC8,
        description: "Turtelizer JTAG/RS232 Adapter A",
    },
    SupportedDevice {
        vid: 0x0403,
        pid: 0xCFF8,
        description: "Amontec JTAGkey",
    },
    SupportedDevice {
        vid: 0x0403,
        pid: 0x8A98,
        description: "TIAO Multi Protocol Adapter",
    },
    SupportedDevice {
        vid: 0x15BA,
        pid: 0x0003,
        description: "Olimex Ltd. OpenOCD JTAG",
    },
    SupportedDevice {
        vid: 0x15BA,
        pid: 0x0004,
        description: "Olimex Ltd. OpenOCD JTAG TINY",
    },
];

/// Mpsse Builder.
/// Open device by VID/PID/index
impl<'s> MpsseBuilder<'s> {
    /// Creates a new Mpsse Builder.
    ///
    /// `mode` is MPSSE mode, one of enum modes.
    /// `freq` is Clock frequency to use for the specified mode.
    /// `endianess` is Specifies how data is clocked in/out (MSB, LSB).
    pub fn new(mode: Mode, freq: u32, endianess: Endianess) -> Self {
        MpsseBuilder {
            vid: 0,
            pid: 0,
            mode,
            freq,
            endianess,
            interface: FtdiInterface::A,
            description: None,
            serial: None,
            index: 0,
        }
    }
    /// Set device vendor ID.
    pub fn set_vid(&mut self, vid: i32) -> &mut Self {
        self.vid = vid;
        self
    }
    /// Set device product ID.
    pub fn set_pid(&mut self, pid: i32) -> &mut Self {
        self.pid = pid;
        self
    }
    /// Set FTDI interface to use (IFACE_A - IFACE_D).
    pub fn set_interface(&mut self, interface: FtdiInterface) -> &mut Self {
        self.interface = interface;
        self
    }
    /// Set device product description if needed.
    pub fn set_description(&mut self, description: &'s str) -> &mut Self {
        self.description = Some(description);
        self
    }
    /// Set device serial number if needed.
    pub fn set_serial(&mut self, serial: &'s str) -> &mut Self {
        self.serial = Some(serial);
        self
    }
    /// Set device index (set to 0 if not needed).
    pub fn set_index(&mut self, index: i32) -> &mut Self {
        self.index = index;
        self
    }
    pub fn build(&self) -> io::Result<Mpsse> {
        let mut mpsse = Mpsse {
            context: unsafe {
                mpsse_sys::OpenIndex(
                    self.vid,
                    self.pid,
                    self.mode as u32,
                    self.freq as i32,
                    self.endianess as i32,
                    self.interface as i32,
                    self.description.map_or(std::ptr::null(), |s| s.as_ptr() as *const i8),
                    self.serial.map_or(std::ptr::null(), |s| s.as_ptr() as *const i8),
                    self.index,
                )
            },
            description: self.description,
            serial: self.serial,
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
}

impl<'s> Mpsse<'s> {
    /// Creates a new Mpsse.
    /// Opens and initializes the first FTDI device found.
    ///
    /// `mode` is MPSSE mode, one of enum modes.
    /// `freq` is Clock frequency to use for the specified mode.
    /// `endianess` is Specifies how data is clocked in/out (MSB, LSB).
    ///
    /// # Examples
    ///
    /// ```no_run
    /// let mut mpsse = mpsse::Mpsse::new(mpsse::Mode::GPIO, 0, mpsse::Endianess::MSB)?;
    /// ```
    pub fn new(mode: Mode, freq: u32, endianess: Endianess) -> io::Result<Self> {
        let mut mpsse = Self {
            context: unsafe { mpsse_sys::MPSSE(mode as u32, freq as i32, endianess as i32) },
            description: None,
            serial: None,
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

    /// Retrieves the last error string from libftdi.
    fn error_string(&mut self) -> String {
        let c_str: &CStr = unsafe { CStr::from_ptr(mpsse_sys::ErrorString(self.context)) };
        c_str.to_str().unwrap_or("ffi internal error").to_owned()
    }

    /// Sets the appropriate transmit and receive commands based on the requested mode and byte order.
    pub fn set_mode(&mut self, endianess: Endianess) -> io::Result<()> {
        if unsafe { mpsse_sys::SetMode(self.context, endianess as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Enables bit-wise data transfers.
    pub fn enable_bit_mode(&mut self, enable_transfer: bool) {
        let tf: i32 = if enable_transfer { 1 } else { 0 };
        unsafe { mpsse_sys::EnableBitmode(self.context, tf) };
    }

    /// Sets the appropriate divisor for the desired clock frequency.
    pub fn set_clock(&mut self, freq: u32) -> io::Result<()> {
        if unsafe { mpsse_sys::SetClock(self.context, freq) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Gets the currently configured clock rate.
    pub fn get_clock(&mut self) -> u32 {
        unsafe { mpsse_sys::GetClock(self.context) as u32 }
    }

    /// Returns the vendor ID of the FTDI chip.
    pub fn get_vid(&mut self) -> i32 {
        unsafe { mpsse_sys::GetVid(self.context) }
    }

    /// Returns the product ID of the FTDI chip.
    pub fn get_pid(&mut self) -> i32 {
        unsafe { mpsse_sys::GetPid(self.context) }
    }

    /// Returns the description of the FTDI chip, if any.
    pub fn get_description(&mut self) -> Option<String> {
        let c_str: &CStr = unsafe { CStr::from_ptr(mpsse_sys::GetDescription(self.context)) };
        c_str.to_str().and_then(|s: &str| Ok(s.to_owned())).ok()
    }

    /// Enable / disable internal loopback.
    /// Set `enable` Zero to disable loopback, 1 to enable loopback.
    pub fn set_loopback(&mut self, enable: bool) -> io::Result<()> {
        let enable: i32 = if enable { 1 } else { 0 };
        if unsafe { mpsse_sys::SetLoopback(self.context, enable) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Sets the idle state of the chip select pin. CS idles high by default.
    pub fn set_cs_idle(&mut self, idle_state_level: PinLevel) {
        unsafe { mpsse_sys::SetCSIdle(self.context, idle_state_level as i32) };
    }

    /// Send data start condition.
    pub fn start(&mut self) -> io::Result<()> {
        if unsafe { mpsse_sys::Start(self.context) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Send data out via the selected serial protocol.
    /// `data` is buffer of data to send.
    pub fn write(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if unsafe { mpsse_sys::Write(self.context, buf.as_mut_ptr() as *mut i8, buf.len() as i32) }
            == MPSSE_OK
        {
            Ok(buf.len())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Send data stop condition.
    pub fn stop(&mut self) -> io::Result<()> {
        if unsafe { mpsse_sys::Stop(self.context) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Returns the last received ACK bit.
    pub fn get_ack(&mut self) -> I2cAck {
        if unsafe { mpsse_sys::GetAck(self.context) } == I2cAck::Ack as i32 {
            I2cAck::Ack
        } else {
            I2cAck::Nack
        }
    }

    /// Sets the transmitted ACK bit.
    pub fn set_ack(&mut self, ack: I2cAck) {
        unsafe { mpsse_sys::SetAck(self.context, ack as i32) };
    }

    /// Causes libmpsse to send ACKs after each read byte in I2C mode.
    pub fn send_acks(&mut self) {
        unsafe { mpsse_sys::SendAcks(self.context) };
    }

    /// Causes libmpsse to send NACKs after each read byte in I2C mode.
    pub fn send_nacks(&mut self) {
        unsafe { mpsse_sys::SendNacks(self.context) };
    }

    ///* Enables or disables flushing of the FTDI chip's RX buffers after each read operation.
    ///* Flushing is disable by default.
    pub fn flush_after_read(&mut self, enable_flush: bool) {
        let flush: i32 = if enable_flush { 1 } else { 0 };
        unsafe { mpsse_sys::FlushAfterRead(self.context, flush) };
    }

    /// Places all I/O pins into a tristate mode.
    pub fn tristate(&mut self) -> io::Result<()> {
        if unsafe { mpsse_sys::Tristate(self.context) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Returns the libmpsse version number.
    /// High nibble is major version, low nibble is minor version.
    pub fn version() -> u8 {
        unsafe { mpsse_sys::Version() as u8 }
    }

    /// Sets the specified pin high.
    pub fn pin_high(&mut self, pin: u32) -> io::Result<()> {
        if unsafe { mpsse_sys::PinHigh(self.context, pin as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Sets the specified pin low.
    pub fn pin_low(&mut self, pin: u32) -> io::Result<()> {
        if unsafe { mpsse_sys::PinLow(self.context, pin as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Checks if a specific pin is high or low. For use in BITBANG mode only.
    pub fn pin_state(&mut self, pin: u32) -> PinLevel {
        // int PinState(struct mpsse_context *mpsse, int pin, int state)
        // @state - The state of the pins, as returned by ReadPins.
        //          If set to -1, ReadPins will automatically be called.
        if unsafe { mpsse_sys::PinState(self.context, pin as i32, -1) } == MPSSE_OK {
            PinLevel::Low
        } else {
            PinLevel::High
        }
    }

    /// Sets the input/output direction of all pins. For use in BITBANG mode only.
    pub fn set_direction(&mut self, direction: Direction) -> io::Result<()> {
        if unsafe { mpsse_sys::SetDirection(self.context, direction as u8) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Performs a bit-wise write of up to 8 bits at a time.
    /// `bits` is a byte containing the desired bits to write.
    /// `size` is the number of bits from the 'bits' byte to write.
    pub fn write_bits(&mut self, bits: u8, size: usize) -> io::Result<()> {
        if unsafe { mpsse_sys::WriteBits(self.context, bits as i8, size as i32) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Performs a bit-wise read of up to 8 bits.
    /// Returns an 8-bit byte containing the read bits.
    /// `size` is number of bits to read.
    pub fn read_bits(&mut self, size: usize) -> u8 {
        unsafe { mpsse_sys::ReadBits(self.context, size as i32) as u8 }
    }

    /// Sets the input/output value of all pins. For use in BITBANG mode only.
    /// `data` is byte indicating bit hi/low value of each bit.
    pub fn write_pins(&mut self, data: u8) -> io::Result<()> {
        if unsafe { mpsse_sys::WritePins(self.context, data) } == MPSSE_OK {
            Ok(())
        } else {
            Err(Error::new(ErrorKind::Other, self.error_string()))
        }
    }

    /// Reads the state of the chip's pins. For use in BITBANG mode only.
    /// Returns a byte with the corresponding pin's bits set to 1 or 0.
    pub fn reqd_pins(&mut self) -> u8 {
        unsafe { mpsse_sys::ReadPins(self.context) as u8 }
    }

    /// Reads data over the selected serial protocol.
    /// `buf` is buffer to read.
    /// Returns a number of the read data on success.
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

    /// Reads and writes data over the selected serial protocol (SPI only).
    /// `buf` is buffer containing bytes to write.
    /// Returns a buffer to the read data on success. This buffer is same buffer of `buf` is given
    /// as argment, `transfer` rewrite `buf`.
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

    /// Function for performing fast writes in MPSSE.
    /// `buf` is the data to write.
    /// Returns a number of the write data on success.
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

    /// Function for performing fast reads in MPSSE.
    /// `buf` is the destination buffer to read data into.
    /// Returns a number of the read data on success.
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

    /// Function to perform fast transfers in MPSSE.
    /// `buf` is buffer containing bytes to write.
    /// Returns a buffer to the read data on success. This buffer is same buffer of `buf` is given
    /// as argment, `transfer` rewrite `buf`.
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

impl<'s> Drop for Mpsse<'s> {
    fn drop(&mut self) {
        unsafe { mpsse_sys::Close(self.context) };
    }
}
