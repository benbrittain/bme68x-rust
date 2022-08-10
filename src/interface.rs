use libc;
use libc::*;

use crate::bme68x::*;
use std::process::Command;

#[no_mangle]
pub unsafe extern "C" fn bme68x_i2c_read(
    mut reg_addr: uint8_t,
    mut reg_data: *mut uint8_t,
    mut len: uint32_t,
    mut intf_ptr: *mut libc::c_void,
) -> int8_t {
    let mut dev_addr_0: uint8_t = *(intf_ptr as *mut uint8_t);
    return -(1 as libc::c_int) as int8_t;
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_i2c_write(
    mut reg_addr: uint8_t,
    mut reg_data: *const uint8_t,
    mut len: uint32_t,
    mut intf_ptr: *mut libc::c_void,
) -> int8_t {
    let mut dev_addr_0: uint8_t = *(intf_ptr as *mut uint8_t);
    return -(1 as libc::c_int) as int8_t;
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_spi_read(
    mut reg_addr: u8,
    mut reg_data: *mut u8,
    mut len: u32,
    mut intf_ptr: *mut libc::c_void,
) -> i8 {
    let reg_slice: &mut [u8] = &mut *std::ptr::slice_from_raw_parts_mut(reg_data, len as usize);
    let cmd = format!("s w 0x{:x} r {len} u", reg_addr);
    let output = Command::new("/home/ben/workspace/spidriver/c/build/spicl")
        .arg("/dev/ttyUSB1")
        .args(cmd.split(" "))
        .output()
        .unwrap();

    let return_bytes: Vec<u8> = std::str::from_utf8(&output.stdout)
        .unwrap()
        .trim()
        .split(",")
        .map(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).unwrap())
        .collect();
    reg_slice.copy_from_slice(&return_bytes[..len as usize]);
    return 0;
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_spi_write(
    reg_addr: u8,
    reg_data: *const u8,
    len: u32,
    _intf_ptr: *mut libc::c_void,
) -> i8 {
    let reg_slice: &[u8] = &*std::ptr::slice_from_raw_parts(reg_data, len as usize);
    let data: String = reg_slice.iter().map(|b| format!("0x{:x},", b)).collect();
    let cmd = format!("s w 0x{:x} w {} u", reg_addr, data);
    let output = Command::new("/home/ben/workspace/spidriver/c/build/spicl")
        .arg("/dev/ttyUSB1")
        .args(cmd.split(" "))
        .output()
        .unwrap();
    return 0;
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_delay_us(period: u32, intf_ptr: *mut libc::c_void) {
    let delay = std::time::Duration::from_micros(period as u64);
    std::thread::sleep(delay);
}

#[derive(Debug)]
pub enum Error {
    NullPointer(String),
    CommunicationFailure(String),
    IncorrectLengthParameter(String),
    DeviceNotFound(String),
    SelfTestError(String),
    NoNewDataFound(String),
    Unknown(String),
}

pub fn check_rslt(api_name: String, rslt: i8) -> Result<(), Error> {
    match rslt {
        0 => Ok(()),
        -1 => Err(Error::NullPointer(api_name)),
        -2 => Err(Error::CommunicationFailure(api_name)),
        -3 => Err(Error::DeviceNotFound(api_name)),
        -4 => Err(Error::IncorrectLengthParameter(api_name)),
        -5 => Err(Error::SelfTestError(api_name)),
        2 => Err(Error::NoNewDataFound(api_name)),
        _ => Err(Error::Unknown(api_name)),
    }
}

pub trait Interface {
    /// Communication to the bme68x device occurs over SPI or I2C.
    fn interface_type() -> CommInterface;

    /// Function for writing the sensor's registers through I2C bus.
    fn i2c_read(_reg_addr: u8, _reg_data: &mut [u8]) {
        unimplemented!("The interface over the I2C device must implement read. This is only called when the interface type is set to CommInterface::I2C.");
    }

    /// Function for writing the sensor's registers through I2C bus.
    fn i2c_write(_reg_addr: u8, _reg_data: &[u8]) {
        unimplemented!("The interface over the I2C device must implement read. This is only called when the interface type is set to CommInterface::I2C.");
    }

    /// Function for reading the sensor's registers through SPI bus.
    fn spi_read(_reg_addr: u8, _reg_data: &mut [u8]) {
        unimplemented!("The interface over the SPI device must implement read. This is only called when the interface type is set to CommInterface::SPI.");
    }

    /// Function for writing the sensor's registers through SPI bus.
    fn spi_write(_reg_addr: u8, _reg_data: &[u8]) {
        unimplemented!("The interface over the SPI device must implement write. This is only called when the interface type is set to CommInterface::SPI.");
    }

    /// Function for delaying in Microseconds.
    fn delay(_us: u64) {
        unimplemented!("The interface over the SPI/I2C device must implement a delay.")
    }
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_interface_init(
    mut bme: *mut Device,
    mut intf: CommInterface,
) -> int8_t {
    let ref mut fresh0 = (*bme).read;
    *fresh0 = Some(
        bme68x_spi_read
            as unsafe extern "C" fn(uint8_t, *mut uint8_t, uint32_t, *mut libc::c_void) -> int8_t,
    );
    let ref mut fresh1 = (*bme).write;
    *fresh1 = Some(
        bme68x_spi_write
            as unsafe extern "C" fn(uint8_t, *const uint8_t, uint32_t, *mut libc::c_void) -> int8_t,
    );
    (*bme).intf = CommInterface::SPI;
    let ref mut fresh2 = (*bme).delay_us;
    *fresh2 = Some(bme68x_delay_us as unsafe extern "C" fn(uint32_t, *mut libc::c_void) -> ());
    (*bme).amb_temp = 25;
    return 0;
}
