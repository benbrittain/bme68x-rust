use libc;
use libc::*;

use crate::bme68x::*;
use std::process::Command;

extern "C" {
    fn printf(_: *const libc::c_char, _: ...) -> libc::c_int;
}
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
    println!("READ 0x{:x} {:?}", reg_addr, reg_slice);
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
    mut reg_addr: uint8_t,
    mut reg_data: *const uint8_t,
    mut len: uint32_t,
    mut intf_ptr: *mut libc::c_void,
) -> int8_t {
    todo!();
    return -(1 as libc::c_int) as int8_t;
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_delay_us(period: u32, intf_ptr: *mut libc::c_void) {
    let delay = std::time::Duration::from_micros(period as u64);
    std::thread::sleep(delay);
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_check_rslt(mut api_name: *const libc::c_char, mut rslt: int8_t) {
    match rslt as libc::c_int {
        0 => {}
        -1 => {
            printf(
                b"API name [%s]  Error [%d] : Null pointer\r\n\0" as *const u8
                    as *const libc::c_char,
                api_name,
                rslt as libc::c_int,
            );
        }
        -2 => {
            printf(
                b"API name [%s]  Error [%d] : Communication failure\r\n\0" as *const u8
                    as *const libc::c_char,
                api_name,
                rslt as libc::c_int,
            );
        }
        -4 => {
            printf(
                b"API name [%s]  Error [%d] : Incorrect length parameter\r\n\0" as *const u8
                    as *const libc::c_char,
                api_name,
                rslt as libc::c_int,
            );
        }
        -3 => {
            printf(
                b"API name [%s]  Error [%d] : Device not found\r\n\0" as *const u8
                    as *const libc::c_char,
                api_name,
                rslt as libc::c_int,
            );
        }
        -5 => {
            printf(
                b"API name [%s]  Error [%d] : Self test error\r\n\0" as *const u8
                    as *const libc::c_char,
                api_name,
                rslt as libc::c_int,
            );
        }
        2 => {
            printf(
                b"API name [%s]  Warning [%d] : No new data found\r\n\0" as *const u8
                    as *const libc::c_char,
                api_name,
                rslt as libc::c_int,
            );
        }
        _ => {
            printf(
                b"API name [%s]  Error [%d] : Unknown error code\r\n\0" as *const u8
                    as *const libc::c_char,
                api_name,
                rslt as libc::c_int,
            );
        }
    };
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_interface_init(
    mut bme: *mut bme68x_dev,
    mut intf: uint8_t,
) -> int8_t {
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
