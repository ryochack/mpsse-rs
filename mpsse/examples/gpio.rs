use mpsse;
use std::io;
use std::{thread, time};

fn main() -> io::Result<()> {
    let mut mpsse = mpsse::Mpsse::new(mpsse::Mode::GPIO, 0, mpsse::Endianess::MSB)?;

    println!("GPIO example start...");

    for pin in mpsse::GPIOL0..=mpsse::GPIOH7 {
        println!("pin_high:{}", pin);
        mpsse.pin_high(pin)?;
        thread::sleep(time::Duration::from_millis(500));
    }

    for pin in mpsse::GPIOL0..=mpsse::GPIOH7 {
        println!("pin_low:{}", pin);
        mpsse.pin_low(pin)?;
        thread::sleep(time::Duration::from_millis(500));
    }

    Ok(())
}

