use mpsse_sys;
use std::ffi::{CStr, CString, c_void};
use libc;

fn main() {
    let io: *mut mpsse_sys::mpsse_context = unsafe {
        mpsse_sys::MPSSE(mpsse_sys::modes_SPI0, mpsse_sys::clock_rates_ONE_HUNDRED_KHZ as i32, mpsse_sys::MSB as i32)
    };
    if !io.is_null() {
        unsafe {
            let desc_msg = CStr::from_ptr(mpsse_sys::GetDescription(io));
            println!("{} initialized at {}Hz (SPI mode 0)", desc_msg.to_str().unwrap(), mpsse_sys::GetClock(io));
            mpsse_sys::Start(io);
            let data = mpsse_sys::Transfer(io, CString::new(b"\x01\x02\x03\x04".to_vec()).unwrap().into_raw(), 4);
            mpsse_sys::Stop(io);

            if !data.is_null() {
                // if connect D0(MOSI) and D1(MISO), data includes [1, 2, 3, 4]
                let data_str = CStr::from_ptr(data);
                println!("read: {:?}", data_str.to_bytes());
            }

            libc::free(data as *mut c_void);
        }
    } else {
        unsafe {
            let err_msg = CStr::from_ptr(mpsse_sys::ErrorString(io));
            println!("Failed to open MPSSE: {}", err_msg.to_str().unwrap());
        }
    }

    unsafe { mpsse_sys::Close(io) };
}

