#![allow(unused_imports)]
#![allow(clippy::single_component_path_imports)]
//#![feature(backtrace)]

#![feature(try_blocks)]

#[cfg(all(feature = "qemu", not(esp32)))]
compile_error!("The `qemu` feature can only be built for the `xtensa-esp32-espidf` target.");

#[cfg(all(feature = "ip101", not(esp32)))]
compile_error!("The `ip101` feature can only be built for the `xtensa-esp32-espidf` target.");

#[cfg(all(feature = "kaluga", not(esp32s2)))]
compile_error!("The `kaluga` feature can only be built for the `xtensa-esp32s2-espidf` target.");

#[cfg(all(feature = "ttgo", not(esp32)))]
compile_error!("The `ttgo` feature can only be built for the `xtensa-esp32-espidf` target.");

#[cfg(all(feature = "heltec", not(esp32s3)))]
compile_error!("The `heltec` feature can only be built for the `xtensa-esp32-espidf` target.");

#[cfg(all(feature = "esp32s3_usb_otg", not(esp32s3)))]
compile_error!(
    "The `esp32s3_usb_otg` feature can only be built for the `xtensa-esp32s3-espidf` target."
);

use core::cell::RefCell;
use core::ffi;
use core::sync::atomic::*;

use std::{fs, mem, result};
use std::io::{Read as _, Write as _};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
use std::os::fd::{AsRawFd, IntoRawFd};
use std::path::PathBuf;
use std::{env, sync::Arc, thread, time::*};

use anyhow::{bail, Result};

use async_io::{Async, Timer};
use esp_idf_svc::http::server::{EspHttpConnection, Request};
use esp_idf_svc::io::Write;
use log::*;

use esp_idf_svc::sys::EspError;

use esp_idf_svc::hal::adc;
use esp_idf_svc::hal::delay::{self, Delay};
use esp_idf_svc::hal::gpio;
use esp_idf_svc::hal::i2c;
use esp_idf_svc::hal::peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::spi;

use esp_idf_svc::eventloop::*;
use esp_idf_svc::ipv4;
use esp_idf_svc::mqtt::client::*;
use esp_idf_svc::ping;
use esp_idf_svc::sntp;
use esp_idf_svc::systime::EspSystemTime;
use esp_idf_svc::timer::*;
use esp_idf_svc::wifi::*;

use display_interface_spi::SPIInterfaceNoCS;

use embedded_graphics::mono_font::{ascii::FONT_10X20, MonoTextStyle};
use embedded_graphics::pixelcolor::*;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::*;
use embedded_graphics::text::*;

use mipidsi;
use sensor_scd30::Scd30;
use ssd1306;
use ssd1306::mode::DisplayConfig;


use byteorder::{ByteOrder, LittleEndian};

use epd_waveshare::{epd4in2::*, graphics::VarDisplay, prelude::*};

use esp32_nimble::{uuid128, BLEDevice, NimbleProperties};
//use esp_idf_sys as _;
use std::format;

#[allow(dead_code)]
#[cfg(not(feature = "qemu"))]
const SSID: &str = ""; //env!("RUST_ESP32_STD_DEMO_WIFI_SSID");
#[allow(dead_code)]
#[cfg(not(feature = "qemu"))]
const PASS: &str = ""; // env!("RUST_ESP32_STD_DEMO_WIFI_PASS");

#[cfg(esp32s2)]
include!(env!("EMBUILD_GENERATED_SYMBOLS_FILE"));


type MyDisplay = ssd1306::Ssd1306<ssd1306::prelude::I2CInterface<i2c::I2cDriver<'static>>, ssd1306::prelude::DisplaySize128x64, ssd1306::mode::BufferedGraphicsMode<ssd1306::prelude::DisplaySize128x64>>;

//type Mutex<T> = embedded_svc::utils::mutex::Mutex<embedded_svc::utils::mutex::RawMutex, T>;

struct SendWrapper<T>(T);
//unsafe impl<T> Send for SendWrapper<T>  {}
//unsafe impl<T> Sync for SendWrapper<T>  {}

fn main() -> Result<()> {
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Get backtraces from anyhow; only works for Xtensa arch currently
    // TODO: No longer working with ESP-IDF 4.3.1+
    //#[cfg(target_arch = "xtensa")]
    //env::set_var("RUST_BACKTRACE", "1");

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let mut display = init_display(pins.gpio21, peripherals.i2c0, pins.gpio17, pins.gpio18)?;

    setStatus(&mut display, "Initialized display, initializing i2c ...")?;

    let i2c_scd30 = i2c::I2cDriver::new(
        peripherals.i2c1,
        pins.gpio46,
        pins.gpio45,
        &i2c::I2cConfig::new()
                .baudrate(50.kHz().into())
                .scl_enable_pullup(true)
                .sda_enable_pullup(true)
    )?;

    let mut scd30: Scd30<i2c::I2cDriver<'_>, Delay, i2c::I2cError> = Scd30::new(i2c_scd30, Delay::new(0))
        .map_err(|e| anyhow::anyhow!("SCD30 error: {:?}", e))?;
    

    setStatus(&mut display, "Reading SCD30 ...")?;

    esp_idf_hal::delay::FreeRtos::delay_ms(1000 as u32);

    scd30.soft_reset()
        .map_err(|e| anyhow::anyhow!("SCD30 error: {:?}", e))?;

    esp_idf_hal::delay::FreeRtos::delay_ms(1000 as u32);

    let firmware_version = scd30.firmware_version()
        .map_err(|e| anyhow::anyhow!("SCD30 error: {:?}", e))?;

    scd30.set_alt_offset(600);
    scd30.set_afc(false);
    scd30.set_measurement_interval(2);
    scd30.set_temp_offset(0.0);

    setStatus(&mut display, format!("FV: {firmware_version}").as_str())?;

    esp_idf_hal::delay::FreeRtos::delay_ms(1000 as u32);

    scd30.start_continuous(0)
        .map_err(|e| anyhow::anyhow!("SCD30 error: {:?}", e))?;

    esp_idf_hal::delay::FreeRtos::delay_ms(1000 as u32);

    // read once
    let mut measurement = scd30.read_data()
        .map_err(|e| anyhow::anyhow!("SCD30 error: {:?}", e))?;


    // put it into an Arc
    let scd30 = Arc::new(std::sync::Mutex::new(SendWrapper(scd30)));

    // ------------------


    let ble_device = BLEDevice::take();

    let server = ble_device.get_server();


    let ble_advertising = ble_device.get_advertising();

    server.on_connect(|server, desc| {
      ::log::info!("Client connected");
  
      server
        .update_conn_params(desc.conn_handle, 24, 48, 0, 60)
        .unwrap();
  
      //::log::info!("Multi-connect support: start advertising");
      //ble_device.get_advertising().start().unwrap();
    });
    server.on_disconnect(|_desc, reason| {
      ::log::info!("Client disconnected ({:X})", reason);
    });
    let service = server.create_service(uuid128!("fafafafa-fafa-fafa-fafa-fafafafafafa"));
  
    // CO2
    let co2_characteristic = service.lock().create_characteristic(
      uuid128!("d4e0e0d0-1a2b-11e9-ab14-d663bd873d93"),
      NimbleProperties::READ | NimbleProperties::NOTIFY,
    );
    co2_characteristic
      .lock()
      .set_value(&measurement.co2.to_le_bytes());
  
    // Temperature
    let temp_characteristic = service.lock().create_characteristic(
      uuid128!("a3c87500-8ed3-4bdf-8a39-a01bebede295"),
      NimbleProperties::READ | NimbleProperties::NOTIFY,
    );
    temp_characteristic
      .lock()
      .set_value(&measurement.temp.to_le_bytes());
  
    // Forced Recalibration
    let calibrate_characteristic = service.lock().create_characteristic(
      uuid128!("3c9a3f00-8ed3-4bdf-8a39-a01bebede295"),
      NimbleProperties::WRITE,
    );

    let copyOfScd30 = Arc::clone(&scd30);

    calibrate_characteristic
      .lock()
      .on_write(move |args| {
        let recalibration_value = LittleEndian::read_u16(args.recv_data);
        ::log::info!("Recalibrating to: {:?}ppm", recalibration_value);
        copyOfScd30.lock().unwrap().0.set_frc(recalibration_value).unwrap();

        //setStatus(&mut display, format!("Calibrated: {recalibrationValue}").as_str());
      });
  
    ble_advertising
      .name("ESP32-CO2-Sensor")
      .add_service_uuid(uuid128!("fafafafa-fafa-fafa-fafa-fafafafafafa"));
  
    ble_advertising.start().unwrap();

        // -------
    
    let mut counter = 0;

    loop {
        let result: Result<()> = try {

            measurement = scd30.lock().map(|mut x| x.0.read_data())//.0.read_data()
              .map_err(|e| anyhow::anyhow!("SCD30 error: {:?}", e))?
              .map_err(|e| anyhow::anyhow!("SCD30 error: {:?}", e))?;

            let co2 = measurement.co2;
            let rh = measurement.rh;
            let temp = measurement.temp;

            // update bluetooth
            co2_characteristic
                .lock()
                .set_value(&co2.to_le_bytes())
                .notify();

            temp_characteristic
              .lock()
              .set_value(&temp.to_le_bytes());

            // update oled

            setStatus(&mut display, format!("rh: {rh}").as_str())?;

            esp_idf_hal::delay::FreeRtos::delay_ms(1000);

            setStatus(&mut display, format!("temp: {temp}").as_str())?;

            esp_idf_hal::delay::FreeRtos::delay_ms(1000);

            setStatus(&mut display, format!("co2: {co2}").as_str())?;

            esp_idf_hal::delay::FreeRtos::delay_ms(2000);
        };

        if let Err(e) = result {

            setStatus(&mut display, format!("ERR: {e}").as_str())?;

            delay::Ets::delay_ms(10000 as u32);
        }
    }

  

    // let mut outputDriver = gpio::PinDriver::output(pins.gpio35)?;
    // while (true) {
    //     outputDriver.set_low();
    //     thread::sleep(Duration::from_millis(1000));
    //     outputDriver.set_high();
    //     thread::sleep(Duration::from_millis(1000));
    // }

    Ok(())
}

fn init_display(
    rst: gpio::Gpio21,
    i2c: i2c::I2C0,
    sda: gpio::Gpio17,
    scl: gpio::Gpio18,
) -> Result<MyDisplay> {
    info!("About to initialize the Heltec SSD1306 I2C LED driver");

    let di = ssd1306::I2CDisplayInterface::new(i2c::I2cDriver::new(
        i2c,
        sda,
        scl,
        &i2c::I2cConfig::new().baudrate(400.kHz().into()),
    )?);

    let mut reset = gpio::PinDriver::output(rst)?;

    reset.set_high()?;
    delay::Ets::delay_ms(1 as u32);

    reset.set_low()?;
    delay::Ets::delay_ms(10 as u32);

    reset.set_high()?;

    // PinDriver has a Drop implementation that resets the pin, which would turn off the display
    mem::forget(reset);


    let mut display: MyDisplay = ssd1306::Ssd1306::new(
        di,
        ssd1306::size::DisplaySize128x64,
        ssd1306::rotation::DisplayRotation::Rotate0,
    )
    .into_buffered_graphics_mode();

    display
        .init()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;
    
    writeText(&mut display, "Hello Rust!",BinaryColor::Off, BinaryColor::On, BinaryColor::Off, BinaryColor::On)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;


    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    Ok(display)
}


fn writeText<D>(
    display: &mut D,
    text: &str,
    bg: D::Color,
    fg: D::Color,
    fill: D::Color,
    stroke: D::Color,
) -> Result<(), D::Error>
where
    D: DrawTarget + Dimensions,
{
    display.clear(bg)?;

    Rectangle::new(display.bounding_box().top_left, display.bounding_box().size)
        .into_styled(
            PrimitiveStyleBuilder::new()
                .fill_color(fill)
                .stroke_color(stroke)
                .stroke_width(1)
                .build(),
        )
        .draw(display)?;

    Text::new(
        &text,
        Point::new(10, (display.bounding_box().size.height - 10) as i32 / 2),
        MonoTextStyle::new(&FONT_10X20, fg),
    )
    .draw(display)?;

    info!("LED rendering done");

    Ok(())
}

fn setStatus(
    display: &mut MyDisplay,
    text: &str
) -> Result<()>
{
    writeText(display, &text, BinaryColor::Off, BinaryColor::On, BinaryColor::Off, BinaryColor::On)
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Display error: {:?}", e))?;

    return Ok(());
}
