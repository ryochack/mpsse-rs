use mpsse;
use std::io;
use std::{thread, time};

fn main() -> io::Result<()> {
    let mut mpsse = mpsse::Mpsse::new(mpsse::Mode::GPIO, 0, mpsse::Endianess::MSB)?;

    mpsse.pin_high(mpsse::gpio_pin::L0)?;
    println!("GPIOL0 State {:?}", mpsse.pin_state(mpsse::gpio_pin::L0));
    thread::sleep(time::Duration::from_secs(1));

    mpsse.pin_low(mpsse::gpio_pin::L0)?;
    println!("GPIOL0 State {:?}", mpsse.pin_state(mpsse::gpio_pin::L0));
    thread::sleep(time::Duration::from_secs(1));

    Ok(())
}

