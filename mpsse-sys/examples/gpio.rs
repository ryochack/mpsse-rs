use mpsse_sys;
use std::{thread, time};
use std::ffi::CStr;

fn main() {
    let io: *mut mpsse_sys::mpsse_context = unsafe {
        mpsse_sys::MPSSE(mpsse_sys::modes_GPIO, 0, 0)
    };
    if !io.is_null() {
        for _ in 0..10 {
            unsafe { mpsse_sys::PinHigh(io, mpsse_sys::gpio_pins_GPIOL0 as i32) };
            println!(
                "GPIOL0 State: {}",
                unsafe { mpsse_sys::PinState(io, mpsse_sys::gpio_pins_GPIOL0 as i32, -1) }
            );
            thread::sleep(time::Duration::from_secs(1));

            unsafe { mpsse_sys::PinLow(io, mpsse_sys::gpio_pins_GPIOL0 as i32) };
            println!(
                "GPIOL0 State: {}",
                unsafe { mpsse_sys::PinState(io, mpsse_sys::gpio_pins_GPIOL0 as i32, -1) }
            );
            thread::sleep(time::Duration::from_secs(1));
        }
    } else {
        unsafe {
            let err_msg = CStr::from_ptr(mpsse_sys::ErrorString(io));
            println!("Failed to open MPSSE: {}", err_msg.to_str().unwrap());
        }
    }

    unsafe { mpsse_sys::Close(io) };
}
