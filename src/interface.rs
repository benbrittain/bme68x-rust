use libc;
use libc::*;

use crate::bme68x::*;
use std::process::Command;

pub type bme68x_intf = libc::c_uint;
pub const BME68X_I2C_INTF: bme68x_intf = 1;
pub const BME68X_SPI_INTF: bme68x_intf = 0;

#[derive(Copy, Clone)]
#[repr(C)]
pub struct bme68x_calib_data {
    pub par_h1: uint16_t,
    pub par_h2: uint16_t,
    pub par_h3: int8_t,
    pub par_h4: int8_t,
    pub par_h5: int8_t,
    pub par_h6: uint8_t,
    pub par_h7: int8_t,
    pub par_gh1: int8_t,
    pub par_gh2: int16_t,
    pub par_gh3: int8_t,
    pub par_t1: uint16_t,
    pub par_t2: int16_t,
    pub par_t3: int8_t,
    pub par_p1: uint16_t,
    pub par_p2: int16_t,
    pub par_p3: int8_t,
    pub par_p4: int16_t,
    pub par_p5: int16_t,
    pub par_p6: int8_t,
    pub par_p7: int8_t,
    pub par_p8: int16_t,
    pub par_p9: int16_t,
    pub par_p10: uint8_t,
    pub t_fine: libc::c_float,
    pub res_heat_range: uint8_t,
    pub res_heat_val: int8_t,
    pub range_sw_err: int8_t,
}
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
    // println!("READ 0x{:x} {:?}", reg_addr, reg_slice);
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
    // println!("WRITE 0x{:x} {:X?}", reg_addr, reg_slice);
    let data: String = reg_slice.iter().map(|b| format!("0x{:x},", b)).collect();
    let cmd = format!("s w 0x{:x} w {} u", reg_addr, data);
    let output = Command::new("/home/ben/workspace/spidriver/c/build/spicl")
        .arg("/dev/ttyUSB1")
        .args(cmd.split(" "))
        .output()
        .unwrap();

    let return_bytes = std::str::from_utf8(&output.stdout);
    //
    //        .unwrap()
    //        .trim()
    //        .split(",")
    //        .map(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).unwrap())
    //        .collect();
    //    println!("{:x?}", return_bytes);
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

#[no_mangle]
pub unsafe extern "C" fn bme68x_interface_init(mut bme: *mut Device, mut intf: uint8_t) -> int8_t {
    let mut rslt: int8_t = 0 as libc::c_int as int8_t;
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
    (*bme).intf = BME68X_SPI_INTF;
    let ref mut fresh2 = (*bme).delay_us;
    *fresh2 = Some(bme68x_delay_us as unsafe extern "C" fn(uint32_t, *mut libc::c_void) -> ());
    (*bme).amb_temp = 25 as libc::c_int as int8_t;
    return rslt;
}
