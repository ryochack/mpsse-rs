use mpsse;
use std::{thread, time};
use std::ffi::CStr;

fn main() {
    let io: *mut mpsse::mpsse_context = unsafe {
        mpsse::MPSSE(mpsse::modes_GPIO, 0, 0)
    };
    if !io.is_null() {
        for _ in 0..10 {
            unsafe { mpsse::PinHigh(io, mpsse::gpio_pins_GPIOL0 as i32) };
            println!(
                "GPIOL0 State: {}",
                unsafe { mpsse::PinState(io, mpsse::gpio_pins_GPIOL0 as i32, -1) }
            );
            thread::sleep(time::Duration::from_secs(1));

            unsafe { mpsse::PinLow(io, mpsse::gpio_pins_GPIOL0 as i32) };
            println!(
                "GPIOL0 State: {}",
                unsafe { mpsse::PinState(io, mpsse::gpio_pins_GPIOL0 as i32, -1) }
            );
            thread::sleep(time::Duration::from_secs(1));
        }
    } else {
        unsafe {
            let err_msg = CStr::from_ptr(mpsse::ErrorString(io));
            println!("Failed to open MPSSE: {}", err_msg.to_str().unwrap());
        }
    }

    unsafe { mpsse::Close(io) };
}
