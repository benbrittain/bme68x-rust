use libc;
use libc::*;

pub type bme68x_read_fptr_t =
    Option<unsafe extern "C" fn(uint8_t, *mut uint8_t, uint32_t, *mut libc::c_void) -> int8_t>;
pub type bme68x_write_fptr_t =
    Option<unsafe extern "C" fn(uint8_t, *const uint8_t, uint32_t, *mut libc::c_void) -> int8_t>;
pub type bme68x_delay_us_fptr_t = Option<unsafe extern "C" fn(uint32_t, *mut libc::c_void) -> ()>;
pub type bme68x_intf = libc::c_uint;
pub const BME68X_I2C_INTF: bme68x_intf = 1;
pub const BME68X_SPI_INTF: bme68x_intf = 0;
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bme68x_data {
    pub status: uint8_t,
    pub gas_index: uint8_t,
    pub meas_index: uint8_t,
    pub res_heat: uint8_t,
    pub idac: uint8_t,
    pub gas_wait: uint8_t,
    pub temperature: libc::c_float,
    pub pressure: libc::c_float,
    pub humidity: libc::c_float,
    pub gas_resistance: libc::c_float,
}
#[derive(Debug, Copy, Clone)]
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
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bme68x_conf {
    pub os_hum: uint8_t,
    pub os_temp: uint8_t,
    pub os_pres: uint8_t,
    pub filter: uint8_t,
    pub odr: uint8_t,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct bme68x_heatr_conf {
    pub enable: uint8_t,
    pub heatr_temp: uint16_t,
    pub heatr_dur: uint16_t,
    pub heatr_temp_prof: *mut uint16_t,
    pub heatr_dur_prof: *mut uint16_t,
    pub profile_len: uint8_t,
    pub shared_heatr_dur: uint16_t,
}
#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct bme68x_dev {
    pub chip_id: uint8_t,
    pub intf_ptr: *mut libc::c_void,
    pub variant_id: uint32_t,
    pub intf: bme68x_intf,
    pub mem_page: uint8_t,
    pub amb_temp: int8_t,
    pub calib: bme68x_calib_data,
    pub read: bme68x_read_fptr_t,
    pub write: bme68x_write_fptr_t,
    pub delay_us: bme68x_delay_us_fptr_t,
    pub intf_rslt: int8_t,
    pub info_msg: uint8_t,
}

impl Default for bme68x_dev {
    fn default() -> Self {
        Self {
            chip_id: 0,
            intf_ptr: 0 as *mut libc::c_void,
            variant_id: 0,
            intf: BME68X_SPI_INTF,
            mem_page: 0,
            amb_temp: 0,
            calib: bme68x_calib_data {
                par_h1: 0,
                par_h2: 0,
                par_h3: 0,
                par_h4: 0,
                par_h5: 0,
                par_h6: 0,
                par_h7: 0,
                par_gh1: 0,
                par_gh2: 0,
                par_gh3: 0,
                par_t1: 0,
                par_t2: 0,
                par_t3: 0,
                par_p1: 0,
                par_p2: 0,
                par_p3: 0,
                par_p4: 0,
                par_p5: 0,
                par_p6: 0,
                par_p7: 0,
                par_p8: 0,
                par_p9: 0,
                par_p10: 0,
                t_fine: 0.,
                res_heat_range: 0,
                res_heat_val: 0,
                range_sw_err: 0,
            },
            read: None,
            write: None,
            delay_us: None,
            intf_rslt: 0,
            info_msg: 0,
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn bme68x_init(mut dev: *mut bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    rslt = bme68x_soft_reset(dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0xd0 as libc::c_int as uint8_t,
            &mut (*dev).chip_id,
            1 as libc::c_int as uint32_t,
            dev,
        );
        if rslt as libc::c_int == 0 as libc::c_int {
            if (*dev).chip_id as libc::c_int == 0x61 as libc::c_int {
                rslt = read_variant_id(dev);
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = get_calib_data(dev);
                }
            } else {
                rslt = -(3 as libc::c_int) as int8_t;
            }
        }
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_set_regs(
    mut reg_addr: *const uint8_t,
    mut reg_data: *const uint8_t,
    mut len: uint32_t,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut tmp_buff: [uint8_t; 20] = [
        0 as libc::c_int as uint8_t,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ];
    let mut index: uint16_t = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int && !reg_addr.is_null() && !reg_data.is_null() {
        if len > 0 as libc::c_int as libc::c_uint
            && len <= (20 as libc::c_int / 2 as libc::c_int) as libc::c_uint
        {
            index = 0 as libc::c_int as uint16_t;
            while (index as libc::c_uint) < len {
                if (*dev).intf as libc::c_uint == BME68X_SPI_INTF as libc::c_int as libc::c_uint {
                    rslt = set_mem_page(*reg_addr.offset(index as isize), dev);
                    tmp_buff[(2 as libc::c_int * index as libc::c_int) as usize] =
                        (*reg_addr.offset(index as isize) as libc::c_int & 0x7f as libc::c_int)
                            as uint8_t;
                } else {
                    tmp_buff[(2 as libc::c_int * index as libc::c_int) as usize] =
                        *reg_addr.offset(index as isize);
                }
                tmp_buff[(2 as libc::c_int * index as libc::c_int + 1 as libc::c_int) as usize] =
                    *reg_data.offset(index as isize);
                index = index.wrapping_add(1);
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                (*dev).intf_rslt = ((*dev).write).expect("non-null function pointer")(
                    tmp_buff[0 as libc::c_int as usize],
                    &mut *tmp_buff.as_mut_ptr().offset(1 as libc::c_int as isize),
                    (2 as libc::c_int as libc::c_uint)
                        .wrapping_mul(len)
                        .wrapping_sub(1 as libc::c_int as libc::c_uint),
                    (*dev).intf_ptr,
                );
                if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
                    rslt = -(2 as libc::c_int) as int8_t;
                }
            }
        } else {
            rslt = -(4 as libc::c_int) as int8_t;
        }
    } else {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_get_regs(
    mut reg_addr: uint8_t,
    mut reg_data: *mut uint8_t,
    mut len: uint32_t,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int && !reg_data.is_null() {
        if (*dev).intf as libc::c_uint == BME68X_SPI_INTF as libc::c_int as libc::c_uint {
            rslt = set_mem_page(reg_addr, dev);
            if rslt as libc::c_int == 0 as libc::c_int {
                reg_addr = (reg_addr as libc::c_int | 0x80 as libc::c_int) as uint8_t;
            }
        }
        (*dev).intf_rslt = ((*dev).read).expect("non-null function pointer")(
            reg_addr,
            reg_data,
            len,
            (*dev).intf_ptr,
        );
        if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
            rslt = -(2 as libc::c_int) as int8_t;
        }
    } else {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_soft_reset(mut dev: *mut bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut reg_addr: uint8_t = 0xe0 as libc::c_int as uint8_t;
    let mut soft_rst_cmd: uint8_t = 0xb6 as libc::c_int as uint8_t;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        if (*dev).intf as libc::c_uint == BME68X_SPI_INTF as libc::c_int as libc::c_uint {
            rslt = get_mem_page(dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = bme68x_set_regs(
                &mut reg_addr,
                &mut soft_rst_cmd,
                1 as libc::c_int as uint32_t,
                dev,
            );
            ((*dev).delay_us).expect("non-null function pointer")(
                10000 as libc::c_uint,
                (*dev).intf_ptr,
            );
            if rslt as libc::c_int == 0 as libc::c_int {
                if (*dev).intf as libc::c_uint == BME68X_SPI_INTF as libc::c_int as libc::c_uint {
                    rslt = get_mem_page(dev);
                }
            }
        }
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_set_conf(
    mut conf: *mut bme68x_conf,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut odr20: uint8_t = 0 as libc::c_int as uint8_t;
    let mut odr3: uint8_t = 1 as libc::c_int as uint8_t;
    let mut current_op_mode: uint8_t = 0;
    let mut reg_array: [uint8_t; 5] = [
        0x71 as libc::c_int as uint8_t,
        0x72 as libc::c_int as uint8_t,
        0x73 as libc::c_int as uint8_t,
        0x74 as libc::c_int as uint8_t,
        0x75 as libc::c_int as uint8_t,
    ];
    let mut data_array: [uint8_t; 5] = [0 as libc::c_int as uint8_t, 0, 0, 0, 0];
    rslt = bme68x_get_op_mode(&mut current_op_mode, dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_set_op_mode(0 as libc::c_int as uint8_t, dev);
    }
    if conf.is_null() {
        rslt = -(1 as libc::c_int) as int8_t;
    } else if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            reg_array[0 as libc::c_int as usize],
            data_array.as_mut_ptr(),
            5 as libc::c_int as uint32_t,
            dev,
        );
        (*dev).info_msg = 0 as libc::c_int as uint8_t;
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).filter, 7 as libc::c_int as uint8_t, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).os_temp, 5 as libc::c_int as uint8_t, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).os_pres, 5 as libc::c_int as uint8_t, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).os_hum, 5 as libc::c_int as uint8_t, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).odr, 8 as libc::c_int as uint8_t, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            data_array[4 as libc::c_int as usize] =
                (data_array[4 as libc::c_int as usize] as libc::c_int & !(0x1c as libc::c_int)
                    | ((*conf).filter as libc::c_int) << 2 as libc::c_int & 0x1c as libc::c_int)
                    as uint8_t;
            data_array[3 as libc::c_int as usize] =
                (data_array[3 as libc::c_int as usize] as libc::c_int & !(0xe0 as libc::c_int)
                    | ((*conf).os_temp as libc::c_int) << 5 as libc::c_int & 0xe0 as libc::c_int)
                    as uint8_t;
            data_array[3 as libc::c_int as usize] =
                (data_array[3 as libc::c_int as usize] as libc::c_int & !(0x1c as libc::c_int)
                    | ((*conf).os_pres as libc::c_int) << 2 as libc::c_int & 0x1c as libc::c_int)
                    as uint8_t;
            data_array[1 as libc::c_int as usize] =
                (data_array[1 as libc::c_int as usize] as libc::c_int & !(0x7 as libc::c_int)
                    | (*conf).os_hum as libc::c_int & 0x7 as libc::c_int)
                    as uint8_t;
            if (*conf).odr as libc::c_int != 8 as libc::c_int {
                odr20 = (*conf).odr;
                odr3 = 0 as libc::c_int as uint8_t;
            }
            data_array[4 as libc::c_int as usize] =
                (data_array[4 as libc::c_int as usize] as libc::c_int & !(0xe0 as libc::c_int)
                    | (odr20 as libc::c_int) << 5 as libc::c_int & 0xe0 as libc::c_int)
                    as uint8_t;
            data_array[0 as libc::c_int as usize] =
                (data_array[0 as libc::c_int as usize] as libc::c_int & !(0x80 as libc::c_int)
                    | (odr3 as libc::c_int) << 7 as libc::c_int & 0x80 as libc::c_int)
                    as uint8_t;
        }
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_set_regs(
            reg_array.as_mut_ptr(),
            data_array.as_mut_ptr(),
            5 as libc::c_int as uint32_t,
            dev,
        );
    }
    if current_op_mode as libc::c_int != 0 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int
    {
        rslt = bme68x_set_op_mode(current_op_mode, dev);
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_get_conf(
    mut conf: *mut bme68x_conf,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut reg_addr: uint8_t = 0x71 as libc::c_int as uint8_t;
    let mut data_array: [uint8_t; 5] = [0; 5];
    rslt = bme68x_get_regs(
        reg_addr,
        data_array.as_mut_ptr(),
        5 as libc::c_int as uint32_t,
        dev,
    );
    if conf.is_null() {
        rslt = -(1 as libc::c_int) as int8_t;
    } else if rslt as libc::c_int == 0 as libc::c_int {
        (*conf).os_hum =
            (data_array[1 as libc::c_int as usize] as libc::c_int & 0x7 as libc::c_int) as uint8_t;
        (*conf).filter = ((data_array[4 as libc::c_int as usize] as libc::c_int
            & 0x1c as libc::c_int)
            >> 2 as libc::c_int) as uint8_t;
        (*conf).os_temp = ((data_array[3 as libc::c_int as usize] as libc::c_int
            & 0xe0 as libc::c_int)
            >> 5 as libc::c_int) as uint8_t;
        (*conf).os_pres = ((data_array[3 as libc::c_int as usize] as libc::c_int
            & 0x1c as libc::c_int)
            >> 2 as libc::c_int) as uint8_t;
        if (data_array[0 as libc::c_int as usize] as libc::c_int & 0x80 as libc::c_int)
            >> 7 as libc::c_int
            != 0
        {
            (*conf).odr = 8 as libc::c_int as uint8_t;
        } else {
            (*conf).odr = ((data_array[4 as libc::c_int as usize] as libc::c_int
                & 0xe0 as libc::c_int)
                >> 5 as libc::c_int) as uint8_t;
        }
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_set_op_mode(op_mode: uint8_t, mut dev: *mut bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut tmp_pow_mode: uint8_t = 0;
    let mut pow_mode: uint8_t = 0 as libc::c_int as uint8_t;
    let mut reg_addr: uint8_t = 0x74 as libc::c_int as uint8_t;
    loop {
        rslt = bme68x_get_regs(
            0x74 as libc::c_int as uint8_t,
            &mut tmp_pow_mode,
            1 as libc::c_int as uint32_t,
            dev,
        );
        if rslt as libc::c_int == 0 as libc::c_int {
            pow_mode = (tmp_pow_mode as libc::c_int & 0x3 as libc::c_int) as uint8_t;
            if pow_mode as libc::c_int != 0 as libc::c_int {
                tmp_pow_mode = (tmp_pow_mode as libc::c_int & !(0x3 as libc::c_int)) as uint8_t;
                rslt = bme68x_set_regs(
                    &mut reg_addr,
                    &mut tmp_pow_mode,
                    1 as libc::c_int as uint32_t,
                    dev,
                );
                ((*dev).delay_us).expect("non-null function pointer")(
                    10000 as libc::c_uint,
                    (*dev).intf_ptr,
                );
            }
        }
        if !(pow_mode as libc::c_int != 0 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int)
        {
            break;
        }
    }
    if op_mode as libc::c_int != 0 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
        tmp_pow_mode = (tmp_pow_mode as libc::c_int & !(0x3 as libc::c_int)
            | op_mode as libc::c_int & 0x3 as libc::c_int) as uint8_t;
        rslt = bme68x_set_regs(
            &mut reg_addr,
            &mut tmp_pow_mode,
            1 as libc::c_int as uint32_t,
            dev,
        );
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_get_op_mode(
    mut op_mode: *mut uint8_t,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut mode: uint8_t = 0;
    if !op_mode.is_null() {
        rslt = bme68x_get_regs(
            0x74 as libc::c_int as uint8_t,
            &mut mode,
            1 as libc::c_int as uint32_t,
            dev,
        );
        *op_mode = (mode as libc::c_int & 0x3 as libc::c_int) as uint8_t;
    } else {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_get_meas_dur(
    op_mode: uint8_t,
    mut conf: *mut bme68x_conf,
    mut dev: *mut bme68x_dev,
) -> uint32_t {
    let mut rslt: int8_t = 0;
    let mut meas_dur: uint32_t = 0 as libc::c_int as uint32_t;
    let mut meas_cycles: uint32_t = 0;
    let mut os_to_meas_cycles: [uint8_t; 6] = [
        0 as libc::c_int as uint8_t,
        1 as libc::c_int as uint8_t,
        2 as libc::c_int as uint8_t,
        4 as libc::c_int as uint8_t,
        8 as libc::c_int as uint8_t,
        16 as libc::c_int as uint8_t,
    ];
    if !conf.is_null() {
        rslt = boundary_check(&mut (*conf).os_temp, 5 as libc::c_int as uint8_t, dev);
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).os_pres, 5 as libc::c_int as uint8_t, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).os_hum, 5 as libc::c_int as uint8_t, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            meas_cycles = os_to_meas_cycles[(*conf).os_temp as usize] as uint32_t;
            meas_cycles = (meas_cycles as libc::c_uint)
                .wrapping_add(os_to_meas_cycles[(*conf).os_pres as usize] as libc::c_uint)
                as uint32_t as uint32_t;
            meas_cycles = (meas_cycles as libc::c_uint)
                .wrapping_add(os_to_meas_cycles[(*conf).os_hum as usize] as libc::c_uint)
                as uint32_t as uint32_t;
            meas_dur = meas_cycles.wrapping_mul(1963 as libc::c_uint);
            meas_dur = (meas_dur as libc::c_uint)
                .wrapping_add((477 as libc::c_int as libc::c_uint).wrapping_mul(4 as libc::c_uint))
                as uint32_t as uint32_t;
            meas_dur = (meas_dur as libc::c_uint)
                .wrapping_add((477 as libc::c_int as libc::c_uint).wrapping_mul(5 as libc::c_uint))
                as uint32_t as uint32_t;
            if op_mode as libc::c_int != 2 as libc::c_int {
                meas_dur = (meas_dur as libc::c_uint).wrapping_add(1000 as libc::c_uint) as uint32_t
                    as uint32_t;
            }
        }
    }
    return meas_dur;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_get_data(
    mut op_mode: uint8_t,
    mut data: *mut bme68x_data,
    mut n_data: *mut uint8_t,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    let mut j: uint8_t = 0 as libc::c_int as uint8_t;
    let mut new_fields: uint8_t = 0 as libc::c_int as uint8_t;
    let mut field_ptr: [*mut bme68x_data; 3] = [
        0 as *mut bme68x_data,
        0 as *mut bme68x_data,
        0 as *mut bme68x_data,
    ];
    let mut field_data: [bme68x_data; 3] = [
        {
            let mut init = bme68x_data {
                status: 0 as libc::c_int as uint8_t,
                gas_index: 0,
                meas_index: 0,
                res_heat: 0,
                idac: 0,
                gas_wait: 0,
                temperature: 0.,
                pressure: 0.,
                humidity: 0.,
                gas_resistance: 0.,
            };
            init
        },
        bme68x_data {
            status: 0,
            gas_index: 0,
            meas_index: 0,
            res_heat: 0,
            idac: 0,
            gas_wait: 0,
            temperature: 0.,
            pressure: 0.,
            humidity: 0.,
            gas_resistance: 0.,
        },
        bme68x_data {
            status: 0,
            gas_index: 0,
            meas_index: 0,
            res_heat: 0,
            idac: 0,
            gas_wait: 0,
            temperature: 0.,
            pressure: 0.,
            humidity: 0.,
            gas_resistance: 0.,
        },
    ];
    field_ptr[0 as libc::c_int as usize] =
        &mut *field_data.as_mut_ptr().offset(0 as libc::c_int as isize) as *mut bme68x_data;
    field_ptr[1 as libc::c_int as usize] =
        &mut *field_data.as_mut_ptr().offset(1 as libc::c_int as isize) as *mut bme68x_data;
    field_ptr[2 as libc::c_int as usize] =
        &mut *field_data.as_mut_ptr().offset(2 as libc::c_int as isize) as *mut bme68x_data;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int && !data.is_null() {
        if op_mode as libc::c_int == 1 as libc::c_int {
            rslt = read_field_data(0 as libc::c_int as uint8_t, data, dev);
            if rslt as libc::c_int == 0 as libc::c_int {
                if (*data).status as libc::c_int & 0x80 as libc::c_int != 0 {
                    new_fields = 1 as libc::c_int as uint8_t;
                } else {
                    new_fields = 0 as libc::c_int as uint8_t;
                    rslt = 2 as libc::c_int as int8_t;
                }
            }
        } else if op_mode as libc::c_int == 2 as libc::c_int
            || op_mode as libc::c_int == 3 as libc::c_int
        {
            rslt = read_all_field_data(field_ptr.as_mut_ptr() as *const *mut bme68x_data, dev);
            new_fields = 0 as libc::c_int as uint8_t;
            i = 0 as libc::c_int as uint8_t;
            while (i as libc::c_int) < 3 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
                if (*field_ptr[i as usize]).status as libc::c_int & 0x80 as libc::c_int != 0 {
                    new_fields = new_fields.wrapping_add(1);
                }
                i = i.wrapping_add(1);
            }
            i = 0 as libc::c_int as uint8_t;
            while (i as libc::c_int) < 2 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
                j = (i as libc::c_int + 1 as libc::c_int) as uint8_t;
                while (j as libc::c_int) < 3 as libc::c_int {
                    sort_sensor_data(i, j, field_ptr.as_mut_ptr());
                    j = j.wrapping_add(1);
                }
                i = i.wrapping_add(1);
            }
            i = 0 as libc::c_int as uint8_t;
            while (i as libc::c_int) < 3 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
                *data.offset(i as isize) = *field_ptr[i as usize];
                i = i.wrapping_add(1);
            }
            if new_fields as libc::c_int == 0 as libc::c_int {
                rslt = 2 as libc::c_int as int8_t;
            }
        } else {
            rslt = 1 as libc::c_int as int8_t;
        }
        if n_data.is_null() {
            rslt = -(1 as libc::c_int) as int8_t;
        } else {
            *n_data = new_fields;
        }
    } else {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_set_heatr_conf(
    mut op_mode: uint8_t,
    mut conf: *const bme68x_heatr_conf,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut nb_conv: uint8_t = 0 as libc::c_int as uint8_t;
    let mut hctrl: uint8_t = 0;
    let mut run_gas: uint8_t = 0 as libc::c_int as uint8_t;
    let mut ctrl_gas_data: [uint8_t; 2] = [0; 2];
    let mut ctrl_gas_addr: [uint8_t; 2] = [
        0x70 as libc::c_int as uint8_t,
        0x71 as libc::c_int as uint8_t,
    ];
    if !conf.is_null() {
        rslt = bme68x_set_op_mode(0 as libc::c_int as uint8_t, dev);
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = set_conf(conf, op_mode, &mut nb_conv, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = bme68x_get_regs(
                0x70 as libc::c_int as uint8_t,
                ctrl_gas_data.as_mut_ptr(),
                2 as libc::c_int as uint32_t,
                dev,
            );
            if rslt as libc::c_int == 0 as libc::c_int {
                if (*conf).enable as libc::c_int == 0x1 as libc::c_int {
                    hctrl = 0 as libc::c_int as uint8_t;
                    if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
                        run_gas = 0x2 as libc::c_int as uint8_t;
                    } else {
                        run_gas = 0x1 as libc::c_int as uint8_t;
                    }
                } else {
                    hctrl = 0x1 as libc::c_int as uint8_t;
                    run_gas = 0 as libc::c_int as uint8_t;
                }
                ctrl_gas_data[0 as libc::c_int as usize] = (ctrl_gas_data[0 as libc::c_int as usize]
                    as libc::c_int
                    & !(0x8 as libc::c_int)
                    | (hctrl as libc::c_int) << 3 as libc::c_int & 0x8 as libc::c_int)
                    as uint8_t;
                ctrl_gas_data[1 as libc::c_int as usize] = (ctrl_gas_data[1 as libc::c_int as usize]
                    as libc::c_int
                    & !(0xf as libc::c_int)
                    | nb_conv as libc::c_int & 0xf as libc::c_int)
                    as uint8_t;
                ctrl_gas_data[1 as libc::c_int as usize] = (ctrl_gas_data[1 as libc::c_int as usize]
                    as libc::c_int
                    & !(0x30 as libc::c_int)
                    | (run_gas as libc::c_int) << 4 as libc::c_int & 0x30 as libc::c_int)
                    as uint8_t;
                rslt = bme68x_set_regs(
                    ctrl_gas_addr.as_mut_ptr(),
                    ctrl_gas_data.as_mut_ptr(),
                    2 as libc::c_int as uint32_t,
                    dev,
                );
            }
        }
    } else {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_get_heatr_conf(
    mut conf: *const bme68x_heatr_conf,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut data_array: [uint8_t; 10] = [0 as libc::c_int as uint8_t, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let mut i: uint8_t = 0;
    rslt = bme68x_get_regs(
        0x5a as libc::c_int as uint8_t,
        data_array.as_mut_ptr(),
        10 as libc::c_int as uint32_t,
        dev,
    );
    if rslt as libc::c_int == 0 as libc::c_int {
        if !conf.is_null()
            && !((*conf).heatr_dur_prof).is_null()
            && !((*conf).heatr_temp_prof).is_null()
        {
            i = 0 as libc::c_int as uint8_t;
            while (i as libc::c_int) < 10 as libc::c_int {
                *((*conf).heatr_temp_prof).offset(i as isize) = data_array[i as usize] as uint16_t;
                i = i.wrapping_add(1);
            }
            rslt = bme68x_get_regs(
                0x64 as libc::c_int as uint8_t,
                data_array.as_mut_ptr(),
                10 as libc::c_int as uint32_t,
                dev,
            );
            if rslt as libc::c_int == 0 as libc::c_int {
                i = 0 as libc::c_int as uint8_t;
                while (i as libc::c_int) < 10 as libc::c_int {
                    *((*conf).heatr_dur_prof).offset(i as isize) =
                        data_array[i as usize] as uint16_t;
                    i = i.wrapping_add(1);
                }
            }
        } else {
            rslt = -(1 as libc::c_int) as int8_t;
        }
    }
    return rslt;
}
#[no_mangle]
pub unsafe extern "C" fn bme68x_selftest_check(mut dev: *const bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut n_fields: uint8_t = 0;
    let mut i: uint8_t = 0 as libc::c_int as uint8_t;
    let mut data: [bme68x_data; 6] = [
        {
            let mut init = bme68x_data {
                status: 0 as libc::c_int as uint8_t,
                gas_index: 0,
                meas_index: 0,
                res_heat: 0,
                idac: 0,
                gas_wait: 0,
                temperature: 0.,
                pressure: 0.,
                humidity: 0.,
                gas_resistance: 0.,
            };
            init
        },
        bme68x_data {
            status: 0,
            gas_index: 0,
            meas_index: 0,
            res_heat: 0,
            idac: 0,
            gas_wait: 0,
            temperature: 0.,
            pressure: 0.,
            humidity: 0.,
            gas_resistance: 0.,
        },
        bme68x_data {
            status: 0,
            gas_index: 0,
            meas_index: 0,
            res_heat: 0,
            idac: 0,
            gas_wait: 0,
            temperature: 0.,
            pressure: 0.,
            humidity: 0.,
            gas_resistance: 0.,
        },
        bme68x_data {
            status: 0,
            gas_index: 0,
            meas_index: 0,
            res_heat: 0,
            idac: 0,
            gas_wait: 0,
            temperature: 0.,
            pressure: 0.,
            humidity: 0.,
            gas_resistance: 0.,
        },
        bme68x_data {
            status: 0,
            gas_index: 0,
            meas_index: 0,
            res_heat: 0,
            idac: 0,
            gas_wait: 0,
            temperature: 0.,
            pressure: 0.,
            humidity: 0.,
            gas_resistance: 0.,
        },
        bme68x_data {
            status: 0,
            gas_index: 0,
            meas_index: 0,
            res_heat: 0,
            idac: 0,
            gas_wait: 0,
            temperature: 0.,
            pressure: 0.,
            humidity: 0.,
            gas_resistance: 0.,
        },
    ];
    let mut t_dev: bme68x_dev = bme68x_dev {
        chip_id: 0,
        intf_ptr: 0 as *mut libc::c_void,
        variant_id: 0,
        intf: BME68X_SPI_INTF,
        mem_page: 0,
        amb_temp: 0,
        calib: bme68x_calib_data {
            par_h1: 0,
            par_h2: 0,
            par_h3: 0,
            par_h4: 0,
            par_h5: 0,
            par_h6: 0,
            par_h7: 0,
            par_gh1: 0,
            par_gh2: 0,
            par_gh3: 0,
            par_t1: 0,
            par_t2: 0,
            par_t3: 0,
            par_p1: 0,
            par_p2: 0,
            par_p3: 0,
            par_p4: 0,
            par_p5: 0,
            par_p6: 0,
            par_p7: 0,
            par_p8: 0,
            par_p9: 0,
            par_p10: 0,
            t_fine: 0.,
            res_heat_range: 0,
            res_heat_val: 0,
            range_sw_err: 0,
        },
        read: None,
        write: None,
        delay_us: None,
        intf_rslt: 0,
        info_msg: 0,
    };
    let mut conf: bme68x_conf = bme68x_conf {
        os_hum: 0,
        os_temp: 0,
        os_pres: 0,
        filter: 0,
        odr: 0,
    };
    let mut heatr_conf: bme68x_heatr_conf = bme68x_heatr_conf {
        enable: 0,
        heatr_temp: 0,
        heatr_dur: 0,
        heatr_temp_prof: 0 as *mut uint16_t,
        heatr_dur_prof: 0 as *mut uint16_t,
        profile_len: 0,
        shared_heatr_dur: 0,
    };
    t_dev.amb_temp = 25 as libc::c_int as int8_t;
    t_dev.read = (*dev).read;
    t_dev.write = (*dev).write;
    t_dev.intf = (*dev).intf;
    t_dev.delay_us = (*dev).delay_us;
    t_dev.intf_ptr = (*dev).intf_ptr;
    rslt = bme68x_init(&mut t_dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        conf.os_hum = 1 as libc::c_int as uint8_t;
        conf.os_pres = 5 as libc::c_int as uint8_t;
        conf.os_temp = 2 as libc::c_int as uint8_t;
        heatr_conf.enable = 0x1 as libc::c_int as uint8_t;
        heatr_conf.heatr_dur = 1000 as libc::c_int as uint16_t;
        heatr_conf.heatr_temp = 350 as libc::c_int as uint16_t;
        rslt = bme68x_set_heatr_conf(1 as libc::c_int as uint8_t, &mut heatr_conf, &mut t_dev);
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = bme68x_set_conf(&mut conf, &mut t_dev);
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = bme68x_set_op_mode(1 as libc::c_int as uint8_t, &mut t_dev);
                if rslt as libc::c_int == 0 as libc::c_int {
                    (t_dev.delay_us).expect("non-null function pointer")(
                        1000000 as libc::c_uint,
                        t_dev.intf_ptr,
                    );
                    rslt = bme68x_get_data(
                        1 as libc::c_int as uint8_t,
                        &mut *data.as_mut_ptr().offset(0 as libc::c_int as isize),
                        &mut n_fields,
                        &mut t_dev,
                    );
                    if rslt as libc::c_int == 0 as libc::c_int {
                        if data[0 as libc::c_int as usize].idac as libc::c_int != 0 as libc::c_int
                            && data[0 as libc::c_int as usize].idac as libc::c_int
                                != 0xff as libc::c_int
                            && data[0 as libc::c_int as usize].status as libc::c_int
                                & 0x20 as libc::c_int
                                != 0
                        {
                            rslt = 0 as libc::c_int as int8_t;
                        } else {
                            rslt = -(5 as libc::c_int) as int8_t;
                        }
                    }
                }
            }
        }
        heatr_conf.heatr_dur = 2000 as libc::c_int as uint16_t;
        while rslt as libc::c_int == 0 as libc::c_int && (i as libc::c_int) < 6 as libc::c_int {
            if i as libc::c_int % 2 as libc::c_int == 0 as libc::c_int {
                heatr_conf.heatr_temp = 350 as libc::c_int as uint16_t;
            } else {
                heatr_conf.heatr_temp = 150 as libc::c_int as uint16_t;
            }
            rslt = bme68x_set_heatr_conf(1 as libc::c_int as uint8_t, &mut heatr_conf, &mut t_dev);
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = bme68x_set_conf(&mut conf, &mut t_dev);
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = bme68x_set_op_mode(1 as libc::c_int as uint8_t, &mut t_dev);
                    if rslt as libc::c_int == 0 as libc::c_int {
                        (t_dev.delay_us).expect("non-null function pointer")(
                            2000000 as libc::c_uint,
                            t_dev.intf_ptr,
                        );
                        rslt = bme68x_get_data(
                            1 as libc::c_int as uint8_t,
                            &mut *data.as_mut_ptr().offset(i as isize),
                            &mut n_fields,
                            &mut t_dev,
                        );
                    }
                }
            }
            i = i.wrapping_add(1);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = analyze_sensor_data(data.as_mut_ptr(), 6 as libc::c_int as uint8_t);
        }
    }
    return rslt;
}
unsafe extern "C" fn calc_temperature(
    mut temp_adc: uint32_t,
    mut dev: *mut bme68x_dev,
) -> libc::c_float {
    let mut var1: libc::c_float = 0.;
    let mut var2: libc::c_float = 0.;
    let mut calc_temp: libc::c_float = 0.;
    var1 = (temp_adc as libc::c_float / 16384.0f32
        - (*dev).calib.par_t1 as libc::c_float / 1024.0f32)
        * (*dev).calib.par_t2 as libc::c_float;
    var2 = (temp_adc as libc::c_float / 131072.0f32
        - (*dev).calib.par_t1 as libc::c_float / 8192.0f32)
        * (temp_adc as libc::c_float / 131072.0f32
            - (*dev).calib.par_t1 as libc::c_float / 8192.0f32)
        * ((*dev).calib.par_t3 as libc::c_float * 16.0f32);
    (*dev).calib.t_fine = var1 + var2;
    calc_temp = (*dev).calib.t_fine / 5120.0f32;
    return calc_temp;
}
unsafe extern "C" fn calc_pressure(
    mut pres_adc: uint32_t,
    mut dev: *const bme68x_dev,
) -> libc::c_float {
    let mut var1: libc::c_float = 0.;
    let mut var2: libc::c_float = 0.;
    let mut var3: libc::c_float = 0.;
    let mut calc_pres: libc::c_float = 0.;
    var1 = (*dev).calib.t_fine / 2.0f32 - 64000.0f32;
    var2 = var1 * var1 * ((*dev).calib.par_p6 as libc::c_float / 131072.0f32);
    var2 = var2 + var1 * (*dev).calib.par_p5 as libc::c_float * 2.0f32;
    var2 = var2 / 4.0f32 + (*dev).calib.par_p4 as libc::c_float * 65536.0f32;
    var1 = ((*dev).calib.par_p3 as libc::c_float * var1 * var1 / 16384.0f32
        + (*dev).calib.par_p2 as libc::c_float * var1)
        / 524288.0f32;
    var1 = (1.0f32 + var1 / 32768.0f32) * (*dev).calib.par_p1 as libc::c_float;
    calc_pres = 1048576.0f32 - pres_adc as libc::c_float;
    if var1 as libc::c_int != 0 as libc::c_int {
        calc_pres = (calc_pres - var2 / 4096.0f32) * 6250.0f32 / var1;
        var1 = (*dev).calib.par_p9 as libc::c_float * calc_pres * calc_pres / 2147483648.0f32;
        var2 = calc_pres * ((*dev).calib.par_p8 as libc::c_float / 32768.0f32);
        var3 = calc_pres / 256.0f32
            * (calc_pres / 256.0f32)
            * (calc_pres / 256.0f32)
            * ((*dev).calib.par_p10 as libc::c_int as libc::c_float / 131072.0f32);
        calc_pres = calc_pres
            + (var1 + var2 + var3 + (*dev).calib.par_p7 as libc::c_float * 128.0f32) / 16.0f32;
    } else {
        calc_pres = 0 as libc::c_int as libc::c_float;
    }
    return calc_pres;
}
unsafe extern "C" fn calc_humidity(
    mut hum_adc: uint16_t,
    mut dev: *const bme68x_dev,
) -> libc::c_float {
    let mut calc_hum: libc::c_float = 0.;
    let mut var1: libc::c_float = 0.;
    let mut var2: libc::c_float = 0.;
    let mut var3: libc::c_float = 0.;
    let mut var4: libc::c_float = 0.;
    let mut temp_comp: libc::c_float = 0.;
    temp_comp = (*dev).calib.t_fine / 5120.0f32;
    var1 = hum_adc as libc::c_float
        - ((*dev).calib.par_h1 as libc::c_float * 16.0f32
            + (*dev).calib.par_h3 as libc::c_float / 2.0f32 * temp_comp);
    var2 = var1
        * ((*dev).calib.par_h2 as libc::c_float / 262144.0f32
            * (1.0f32
                + (*dev).calib.par_h4 as libc::c_float / 16384.0f32 * temp_comp
                + (*dev).calib.par_h5 as libc::c_float / 1048576.0f32 * temp_comp * temp_comp));
    var3 = (*dev).calib.par_h6 as libc::c_float / 16384.0f32;
    var4 = (*dev).calib.par_h7 as libc::c_float / 2097152.0f32;
    calc_hum = var2 + (var3 + var4 * temp_comp) * var2 * var2;
    if calc_hum > 100.0f32 {
        calc_hum = 100.0f32;
    } else if calc_hum < 0.0f32 {
        calc_hum = 0.0f32;
    }
    return calc_hum;
}
unsafe extern "C" fn calc_gas_resistance_low(
    mut gas_res_adc: uint16_t,
    mut gas_range: uint8_t,
    mut dev: *const bme68x_dev,
) -> libc::c_float {
    let mut calc_gas_res: libc::c_float = 0.;
    let mut var1: libc::c_float = 0.;
    let mut var2: libc::c_float = 0.;
    let mut var3: libc::c_float = 0.;
    let mut gas_res_f: libc::c_float = gas_res_adc as libc::c_float;
    let mut gas_range_f: libc::c_float =
        ((1 as libc::c_uint) << gas_range as libc::c_int) as libc::c_float;
    let lookup_k1_range: [libc::c_float; 16] = [
        0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32, -1.0f32, 0.0f32, -0.8f32, 0.0f32, 0.0f32, -0.2f32,
        -0.5f32, 0.0f32, -1.0f32, 0.0f32, 0.0f32,
    ];
    let lookup_k2_range: [libc::c_float; 16] = [
        0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.1f32, 0.7f32, 0.0f32, -0.8f32, -0.1f32, 0.0f32, 0.0f32,
        0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32,
    ];
    var1 = 1340.0f32 + 5.0f32 * (*dev).calib.range_sw_err as libc::c_int as libc::c_float;
    var2 = var1 * (1.0f32 + lookup_k1_range[gas_range as usize] / 100.0f32);
    var3 = 1.0f32 + lookup_k2_range[gas_range as usize] / 100.0f32;
    calc_gas_res =
        1.0f32 / (var3 * 0.000000125f32 * gas_range_f * ((gas_res_f - 512.0f32) / var2 + 1.0f32));
    return calc_gas_res;
}
unsafe extern "C" fn calc_gas_resistance_high(
    mut gas_res_adc: uint16_t,
    mut gas_range: uint8_t,
) -> libc::c_float {
    let mut calc_gas_res: libc::c_float = 0.;
    let mut var1: uint32_t = 262144 as libc::c_uint >> gas_range as libc::c_int;
    let mut var2: int32_t = gas_res_adc as int32_t - 512 as libc::c_int;
    var2 *= 3 as libc::c_int;
    var2 = 4096 as libc::c_int + var2;
    calc_gas_res = 1000000.0f32 * var1 as libc::c_float / var2 as libc::c_float;
    return calc_gas_res;
}
unsafe extern "C" fn calc_res_heat(mut temp: uint16_t, mut dev: *const bme68x_dev) -> uint8_t {
    let mut var1: libc::c_float = 0.;
    let mut var2: libc::c_float = 0.;
    let mut var3: libc::c_float = 0.;
    let mut var4: libc::c_float = 0.;
    let mut var5: libc::c_float = 0.;
    let mut res_heat: uint8_t = 0;
    if temp as libc::c_int > 400 as libc::c_int {
        temp = 400 as libc::c_int as uint16_t;
    }
    var1 = (*dev).calib.par_gh1 as libc::c_float / 16.0f32 + 49.0f32;
    var2 = (*dev).calib.par_gh2 as libc::c_float / 32768.0f32 * 0.0005f32 + 0.00235f32;
    var3 = (*dev).calib.par_gh3 as libc::c_float / 1024.0f32;
    var4 = var1 * (1.0f32 + var2 * temp as libc::c_float);
    var5 = var4 + var3 * (*dev).amb_temp as libc::c_float;
    res_heat = (3.4f32
        * (var5
            * (4 as libc::c_int as libc::c_float
                / (4 as libc::c_int as libc::c_float
                    + (*dev).calib.res_heat_range as libc::c_float))
            * (1 as libc::c_int as libc::c_float
                / (1 as libc::c_int as libc::c_float
                    + (*dev).calib.res_heat_val as libc::c_float * 0.002f32))
            - 25 as libc::c_int as libc::c_float)) as uint8_t;
    return res_heat;
}
unsafe extern "C" fn calc_gas_wait(mut dur: uint16_t) -> uint8_t {
    let mut factor: uint8_t = 0 as libc::c_int as uint8_t;
    let mut durval: uint8_t = 0;
    if dur as libc::c_int >= 0xfc0 as libc::c_int {
        durval = 0xff as libc::c_int as uint8_t;
    } else {
        while dur as libc::c_int > 0x3f as libc::c_int {
            dur = (dur as libc::c_int / 4 as libc::c_int) as uint16_t;
            factor = (factor as libc::c_int + 1 as libc::c_int) as uint8_t;
        }
        durval = (dur as libc::c_int + factor as libc::c_int * 64 as libc::c_int) as uint8_t;
    }
    return durval;
}
unsafe extern "C" fn read_field_data(
    mut index: uint8_t,
    mut data: *mut bme68x_data,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0 as libc::c_int as int8_t;
    let mut buff: [uint8_t; 17] = [
        0 as libc::c_int as uint8_t,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ];
    let mut gas_range_l: uint8_t = 0;
    let mut gas_range_h: uint8_t = 0;
    let mut adc_temp: uint32_t = 0;
    let mut adc_pres: uint32_t = 0;
    let mut adc_hum: uint16_t = 0;
    let mut adc_gas_res_low: uint16_t = 0;
    let mut adc_gas_res_high: uint16_t = 0;
    let mut tries: uint8_t = 5 as libc::c_int as uint8_t;
    while tries as libc::c_int != 0 && rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            (0x1d as libc::c_int + index as libc::c_int * 17 as libc::c_int) as uint8_t,
            buff.as_mut_ptr(),
            17 as libc::c_int as uint16_t as uint32_t,
            dev,
        );
        if data.is_null() {
            rslt = -(1 as libc::c_int) as int8_t;
            break;
        } else {
            (*data).status =
                (buff[0 as libc::c_int as usize] as libc::c_int & 0x80 as libc::c_int) as uint8_t;
            (*data).gas_index =
                (buff[0 as libc::c_int as usize] as libc::c_int & 0xf as libc::c_int) as uint8_t;
            (*data).meas_index = buff[1 as libc::c_int as usize];
            adc_pres = (buff[2 as libc::c_int as usize] as uint32_t)
                .wrapping_mul(4096 as libc::c_int as libc::c_uint)
                | (buff[3 as libc::c_int as usize] as uint32_t)
                    .wrapping_mul(16 as libc::c_int as libc::c_uint)
                | (buff[4 as libc::c_int as usize] as uint32_t)
                    .wrapping_div(16 as libc::c_int as libc::c_uint);
            adc_temp = (buff[5 as libc::c_int as usize] as uint32_t)
                .wrapping_mul(4096 as libc::c_int as libc::c_uint)
                | (buff[6 as libc::c_int as usize] as uint32_t)
                    .wrapping_mul(16 as libc::c_int as libc::c_uint)
                | (buff[7 as libc::c_int as usize] as uint32_t)
                    .wrapping_div(16 as libc::c_int as libc::c_uint);
            adc_hum = ((buff[8 as libc::c_int as usize] as uint32_t)
                .wrapping_mul(256 as libc::c_int as libc::c_uint)
                | buff[9 as libc::c_int as usize] as uint32_t) as uint16_t;
            adc_gas_res_low = ((buff[13 as libc::c_int as usize] as uint32_t)
                .wrapping_mul(4 as libc::c_int as libc::c_uint)
                | (buff[14 as libc::c_int as usize] as uint32_t)
                    .wrapping_div(64 as libc::c_int as libc::c_uint))
                as uint16_t;
            adc_gas_res_high = ((buff[15 as libc::c_int as usize] as uint32_t)
                .wrapping_mul(4 as libc::c_int as libc::c_uint)
                | (buff[16 as libc::c_int as usize] as uint32_t)
                    .wrapping_div(64 as libc::c_int as libc::c_uint))
                as uint16_t;
            gas_range_l =
                (buff[14 as libc::c_int as usize] as libc::c_int & 0xf as libc::c_int) as uint8_t;
            gas_range_h =
                (buff[16 as libc::c_int as usize] as libc::c_int & 0xf as libc::c_int) as uint8_t;
            if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
                let ref mut fresh0 = (*data).status;
                *fresh0 = (*fresh0 as libc::c_int
                    | buff[16 as libc::c_int as usize] as libc::c_int & 0x20 as libc::c_int)
                    as uint8_t;
                let ref mut fresh1 = (*data).status;
                *fresh1 = (*fresh1 as libc::c_int
                    | buff[16 as libc::c_int as usize] as libc::c_int & 0x10 as libc::c_int)
                    as uint8_t;
            } else {
                let ref mut fresh2 = (*data).status;
                *fresh2 = (*fresh2 as libc::c_int
                    | buff[14 as libc::c_int as usize] as libc::c_int & 0x20 as libc::c_int)
                    as uint8_t;
                let ref mut fresh3 = (*data).status;
                *fresh3 = (*fresh3 as libc::c_int
                    | buff[14 as libc::c_int as usize] as libc::c_int & 0x10 as libc::c_int)
                    as uint8_t;
            }
            if (*data).status as libc::c_int & 0x80 as libc::c_int != 0
                && rslt as libc::c_int == 0 as libc::c_int
            {
                rslt = bme68x_get_regs(
                    (0x5a as libc::c_int + (*data).gas_index as libc::c_int) as uint8_t,
                    &mut (*data).res_heat,
                    1 as libc::c_int as uint32_t,
                    dev,
                );
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = bme68x_get_regs(
                        (0x50 as libc::c_int + (*data).gas_index as libc::c_int) as uint8_t,
                        &mut (*data).idac,
                        1 as libc::c_int as uint32_t,
                        dev,
                    );
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = bme68x_get_regs(
                        (0x64 as libc::c_int + (*data).gas_index as libc::c_int) as uint8_t,
                        &mut (*data).gas_wait,
                        1 as libc::c_int as uint32_t,
                        dev,
                    );
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    (*data).temperature = calc_temperature(adc_temp, dev);
                    (*data).pressure = calc_pressure(adc_pres, dev);
                    (*data).humidity = calc_humidity(adc_hum, dev);
                    if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
                        (*data).gas_resistance =
                            calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
                    } else {
                        (*data).gas_resistance =
                            calc_gas_resistance_low(adc_gas_res_low, gas_range_l, dev);
                    }
                    break;
                }
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                ((*dev).delay_us).expect("non-null function pointer")(
                    10000 as libc::c_uint,
                    (*dev).intf_ptr,
                );
            }
            tries = tries.wrapping_sub(1);
        }
    }
    return rslt;
}
unsafe extern "C" fn read_all_field_data(
    mut data: *const *mut bme68x_data,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0 as libc::c_int as int8_t;
    let mut buff: [uint8_t; 51] = [
        0 as libc::c_int as uint8_t,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ];
    let mut gas_range_l: uint8_t = 0;
    let mut gas_range_h: uint8_t = 0;
    let mut adc_temp: uint32_t = 0;
    let mut adc_pres: uint32_t = 0;
    let mut adc_hum: uint16_t = 0;
    let mut adc_gas_res_low: uint16_t = 0;
    let mut adc_gas_res_high: uint16_t = 0;
    let mut off: uint8_t = 0;
    let mut set_val: [uint8_t; 30] = [
        0 as libc::c_int as uint8_t,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ];
    let mut i: uint8_t = 0;
    if (*data.offset(0 as libc::c_int as isize)).is_null()
        && (*data.offset(1 as libc::c_int as isize)).is_null()
        && (*data.offset(2 as libc::c_int as isize)).is_null()
    {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0x1d as libc::c_int as uint8_t,
            buff.as_mut_ptr(),
            (17 as libc::c_int as uint32_t).wrapping_mul(3 as libc::c_int as libc::c_uint),
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0x50 as libc::c_int as uint8_t,
            set_val.as_mut_ptr(),
            30 as libc::c_int as uint32_t,
            dev,
        );
    }
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < 3 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
        off = (i as libc::c_int * 17 as libc::c_int) as uint8_t;
        (**data.offset(i as isize)).status =
            (buff[off as usize] as libc::c_int & 0x80 as libc::c_int) as uint8_t;
        (**data.offset(i as isize)).gas_index =
            (buff[off as usize] as libc::c_int & 0xf as libc::c_int) as uint8_t;
        (**data.offset(i as isize)).meas_index =
            buff[(off as libc::c_int + 1 as libc::c_int) as usize];
        adc_pres = (buff[(off as libc::c_int + 2 as libc::c_int) as usize] as uint32_t)
            .wrapping_mul(4096 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 3 as libc::c_int) as usize] as uint32_t)
                .wrapping_mul(16 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 4 as libc::c_int) as usize] as uint32_t)
                .wrapping_div(16 as libc::c_int as libc::c_uint);
        adc_temp = (buff[(off as libc::c_int + 5 as libc::c_int) as usize] as uint32_t)
            .wrapping_mul(4096 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 6 as libc::c_int) as usize] as uint32_t)
                .wrapping_mul(16 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 7 as libc::c_int) as usize] as uint32_t)
                .wrapping_div(16 as libc::c_int as libc::c_uint);
        adc_hum = ((buff[(off as libc::c_int + 8 as libc::c_int) as usize] as uint32_t)
            .wrapping_mul(256 as libc::c_int as libc::c_uint)
            | buff[(off as libc::c_int + 9 as libc::c_int) as usize] as uint32_t)
            as uint16_t;
        adc_gas_res_low = ((buff[(off as libc::c_int + 13 as libc::c_int) as usize] as uint32_t)
            .wrapping_mul(4 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 14 as libc::c_int) as usize] as uint32_t)
                .wrapping_div(64 as libc::c_int as libc::c_uint))
            as uint16_t;
        adc_gas_res_high = ((buff[(off as libc::c_int + 15 as libc::c_int) as usize] as uint32_t)
            .wrapping_mul(4 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 16 as libc::c_int) as usize] as uint32_t)
                .wrapping_div(64 as libc::c_int as libc::c_uint))
            as uint16_t;
        gas_range_l = (buff[(off as libc::c_int + 14 as libc::c_int) as usize] as libc::c_int
            & 0xf as libc::c_int) as uint8_t;
        gas_range_h = (buff[(off as libc::c_int + 16 as libc::c_int) as usize] as libc::c_int
            & 0xf as libc::c_int) as uint8_t;
        if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
            let ref mut fresh4 = (**data.offset(i as isize)).status;
            *fresh4 = (*fresh4 as libc::c_int
                | buff[(off as libc::c_int + 16 as libc::c_int) as usize] as libc::c_int
                    & 0x20 as libc::c_int) as uint8_t;
            let ref mut fresh5 = (**data.offset(i as isize)).status;
            *fresh5 = (*fresh5 as libc::c_int
                | buff[(off as libc::c_int + 16 as libc::c_int) as usize] as libc::c_int
                    & 0x10 as libc::c_int) as uint8_t;
        } else {
            let ref mut fresh6 = (**data.offset(i as isize)).status;
            *fresh6 = (*fresh6 as libc::c_int
                | buff[(off as libc::c_int + 14 as libc::c_int) as usize] as libc::c_int
                    & 0x20 as libc::c_int) as uint8_t;
            let ref mut fresh7 = (**data.offset(i as isize)).status;
            *fresh7 = (*fresh7 as libc::c_int
                | buff[(off as libc::c_int + 14 as libc::c_int) as usize] as libc::c_int
                    & 0x10 as libc::c_int) as uint8_t;
        }
        (**data.offset(i as isize)).idac = set_val[(**data.offset(i as isize)).gas_index as usize];
        (**data.offset(i as isize)).res_heat = set_val
            [(10 as libc::c_int + (**data.offset(i as isize)).gas_index as libc::c_int) as usize];
        (**data.offset(i as isize)).gas_wait = set_val
            [(20 as libc::c_int + (**data.offset(i as isize)).gas_index as libc::c_int) as usize];
        (**data.offset(i as isize)).temperature = calc_temperature(adc_temp, dev);
        (**data.offset(i as isize)).pressure = calc_pressure(adc_pres, dev);
        (**data.offset(i as isize)).humidity = calc_humidity(adc_hum, dev);
        if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
            (**data.offset(i as isize)).gas_resistance =
                calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
        } else {
            (**data.offset(i as isize)).gas_resistance =
                calc_gas_resistance_low(adc_gas_res_low, gas_range_l, dev);
        }
        i = i.wrapping_add(1);
    }
    return rslt;
}
unsafe extern "C" fn set_mem_page(mut reg_addr: uint8_t, mut dev: *mut bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut reg: uint8_t = 0;
    let mut mem_page: uint8_t = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        if reg_addr as libc::c_int > 0x7f as libc::c_int {
            mem_page = 0 as libc::c_int as uint8_t;
        } else {
            mem_page = 0x10 as libc::c_int as uint8_t;
        }
        if mem_page as libc::c_int != (*dev).mem_page as libc::c_int {
            (*dev).mem_page = mem_page;
            (*dev).intf_rslt = ((*dev).read).expect("non-null function pointer")(
                (0xf3 as libc::c_int | 0x80 as libc::c_int) as uint8_t,
                &mut reg,
                1 as libc::c_int as uint32_t,
                (*dev).intf_ptr,
            );
            if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
                rslt = -(2 as libc::c_int) as int8_t;
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                reg = (reg as libc::c_int & !(0x10 as libc::c_int)) as uint8_t;
                reg = (reg as libc::c_int | (*dev).mem_page as libc::c_int & 0x10 as libc::c_int)
                    as uint8_t;
                (*dev).intf_rslt = ((*dev).write).expect("non-null function pointer")(
                    (0xf3 as libc::c_int & 0x7f as libc::c_int) as uint8_t,
                    &mut reg,
                    1 as libc::c_int as uint32_t,
                    (*dev).intf_ptr,
                );
                if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
                    rslt = -(2 as libc::c_int) as int8_t;
                }
            }
        }
    }
    return rslt;
}
unsafe extern "C" fn get_mem_page(mut dev: *mut bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut reg: uint8_t = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        (*dev).intf_rslt = ((*dev).read).expect("non-null function pointer")(
            (0xf3 as libc::c_int | 0x80 as libc::c_int) as uint8_t,
            &mut reg,
            1 as libc::c_int as uint32_t,
            (*dev).intf_ptr,
        );
        if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
            rslt = -(2 as libc::c_int) as int8_t;
        } else {
            (*dev).mem_page = (reg as libc::c_int & 0x10 as libc::c_int) as uint8_t;
        }
    }
    return rslt;
}
unsafe extern "C" fn boundary_check(
    mut value: *mut uint8_t,
    mut max: uint8_t,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0;
    rslt = null_ptr_check(dev);
    if !value.is_null() && rslt as libc::c_int == 0 as libc::c_int {
        if *value as libc::c_int > max as libc::c_int {
            *value = max;
            let ref mut fresh8 = (*dev).info_msg;
            *fresh8 = (*fresh8 as libc::c_int | 1 as libc::c_int) as uint8_t;
        }
    } else {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    return rslt;
}
unsafe extern "C" fn null_ptr_check(mut dev: *const bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0 as libc::c_int as int8_t;
    if dev.is_null()
        || ((*dev).read).is_none()
        || ((*dev).write).is_none()
        || ((*dev).delay_us).is_none()
    {
        rslt = -(1 as libc::c_int) as int8_t;
    }
    return rslt;
}
unsafe extern "C" fn set_conf(
    mut conf: *const bme68x_heatr_conf,
    mut op_mode: uint8_t,
    mut nb_conv: *mut uint8_t,
    mut dev: *mut bme68x_dev,
) -> int8_t {
    let mut rslt: int8_t = 0 as libc::c_int as int8_t;
    let mut i: uint8_t = 0;
    let mut shared_dur: uint8_t = 0;
    let mut write_len: uint8_t = 0 as libc::c_int as uint8_t;
    let mut heater_dur_shared_addr: uint8_t = 0x6e as libc::c_int as uint8_t;
    let mut rh_reg_addr: [uint8_t; 10] = [
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
    ];
    let mut rh_reg_data: [uint8_t; 10] = [
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
    ];
    let mut gw_reg_addr: [uint8_t; 10] = [
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
    ];
    let mut gw_reg_data: [uint8_t; 10] = [
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
        0 as libc::c_int as uint8_t,
    ];
    match op_mode as libc::c_int {
        1 => {
            rh_reg_addr[0 as libc::c_int as usize] = 0x5a as libc::c_int as uint8_t;
            rh_reg_data[0 as libc::c_int as usize] = calc_res_heat((*conf).heatr_temp, dev);
            gw_reg_addr[0 as libc::c_int as usize] = 0x64 as libc::c_int as uint8_t;
            gw_reg_data[0 as libc::c_int as usize] = calc_gas_wait((*conf).heatr_dur);
            *nb_conv = 0 as libc::c_int as uint8_t;
            write_len = 1 as libc::c_int as uint8_t;
        }
        3 => {
            if ((*conf).heatr_dur_prof).is_null() || ((*conf).heatr_temp_prof).is_null() {
                rslt = -(1 as libc::c_int) as int8_t;
            } else {
                i = 0 as libc::c_int as uint8_t;
                while (i as libc::c_int) < (*conf).profile_len as libc::c_int {
                    rh_reg_addr[i as usize] = (0x5a as libc::c_int + i as libc::c_int) as uint8_t;
                    rh_reg_data[i as usize] =
                        calc_res_heat(*((*conf).heatr_temp_prof).offset(i as isize), dev);
                    gw_reg_addr[i as usize] = (0x64 as libc::c_int + i as libc::c_int) as uint8_t;
                    gw_reg_data[i as usize] =
                        calc_gas_wait(*((*conf).heatr_dur_prof).offset(i as isize));
                    i = i.wrapping_add(1);
                }
                *nb_conv = (*conf).profile_len;
                write_len = (*conf).profile_len;
            }
        }
        2 => {
            if ((*conf).heatr_dur_prof).is_null() || ((*conf).heatr_temp_prof).is_null() {
                rslt = -(1 as libc::c_int) as int8_t;
            } else {
                if (*conf).shared_heatr_dur as libc::c_int == 0 as libc::c_int {
                    rslt = 3 as libc::c_int as int8_t;
                }
                i = 0 as libc::c_int as uint8_t;
                while (i as libc::c_int) < (*conf).profile_len as libc::c_int {
                    rh_reg_addr[i as usize] = (0x5a as libc::c_int + i as libc::c_int) as uint8_t;
                    rh_reg_data[i as usize] =
                        calc_res_heat(*((*conf).heatr_temp_prof).offset(i as isize), dev);
                    gw_reg_addr[i as usize] = (0x64 as libc::c_int + i as libc::c_int) as uint8_t;
                    gw_reg_data[i as usize] =
                        *((*conf).heatr_dur_prof).offset(i as isize) as uint8_t;
                    i = i.wrapping_add(1);
                }
                *nb_conv = (*conf).profile_len;
                write_len = (*conf).profile_len;
                shared_dur = calc_heatr_dur_shared((*conf).shared_heatr_dur);
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = bme68x_set_regs(
                        &mut heater_dur_shared_addr,
                        &mut shared_dur,
                        1 as libc::c_int as uint32_t,
                        dev,
                    );
                }
            }
        }
        _ => {
            rslt = 1 as libc::c_int as int8_t;
        }
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_set_regs(
            rh_reg_addr.as_mut_ptr(),
            rh_reg_data.as_mut_ptr(),
            write_len as uint32_t,
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_set_regs(
            gw_reg_addr.as_mut_ptr(),
            gw_reg_data.as_mut_ptr(),
            write_len as uint32_t,
            dev,
        );
    }
    return rslt;
}
unsafe extern "C" fn calc_heatr_dur_shared(mut dur: uint16_t) -> uint8_t {
    let mut factor: uint8_t = 0 as libc::c_int as uint8_t;
    let mut heatdurval: uint8_t = 0;
    if dur as libc::c_int >= 0x783 as libc::c_int {
        heatdurval = 0xff as libc::c_int as uint8_t;
    } else {
        dur = (dur as uint32_t)
            .wrapping_mul(1000 as libc::c_int as libc::c_uint)
            .wrapping_div(477 as libc::c_int as libc::c_uint) as uint16_t;
        while dur as libc::c_int > 0x3f as libc::c_int {
            dur = (dur as libc::c_int >> 2 as libc::c_int) as uint16_t;
            factor = (factor as libc::c_int + 1 as libc::c_int) as uint8_t;
        }
        heatdurval = (dur as libc::c_int + factor as libc::c_int * 64 as libc::c_int) as uint8_t;
    }
    return heatdurval;
}
unsafe extern "C" fn sort_sensor_data(
    mut low_index: uint8_t,
    mut high_index: uint8_t,
    mut field: *mut *mut bme68x_data,
) {
    let mut meas_index1: int16_t = 0;
    let mut meas_index2: int16_t = 0;
    meas_index1 = (**field.offset(low_index as isize)).meas_index as int16_t;
    meas_index2 = (**field.offset(high_index as isize)).meas_index as int16_t;
    if (**field.offset(low_index as isize)).status as libc::c_int & 0x80 as libc::c_int != 0
        && (**field.offset(high_index as isize)).status as libc::c_int & 0x80 as libc::c_int != 0
    {
        let mut diff: int16_t =
            (meas_index2 as libc::c_int - meas_index1 as libc::c_int) as int16_t;
        if diff as libc::c_int > -(3 as libc::c_int) && (diff as libc::c_int) < 0 as libc::c_int
            || diff as libc::c_int > 2 as libc::c_int
        {
            swap_fields(low_index, high_index, field);
        }
    } else if (**field.offset(high_index as isize)).status as libc::c_int & 0x80 as libc::c_int != 0
    {
        swap_fields(low_index, high_index, field);
    }
}
unsafe extern "C" fn swap_fields(
    mut index1: uint8_t,
    mut index2: uint8_t,
    mut field: *mut *mut bme68x_data,
) {
    let mut temp: *mut bme68x_data = 0 as *mut bme68x_data;
    temp = *field.offset(index1 as isize);
    let ref mut fresh9 = *field.offset(index1 as isize);
    *fresh9 = *field.offset(index2 as isize);
    let ref mut fresh10 = *field.offset(index2 as isize);
    *fresh10 = temp;
}
unsafe extern "C" fn analyze_sensor_data(
    mut data: *const bme68x_data,
    mut n_meas: uint8_t,
) -> int8_t {
    let mut rslt: int8_t = 0 as libc::c_int as int8_t;
    let mut self_test_failed: uint8_t = 0 as libc::c_int as uint8_t;
    let mut i: uint8_t = 0;
    let mut cent_res: uint32_t = 0 as libc::c_int as uint32_t;
    if (*data.offset(0 as libc::c_int as isize)).temperature < 0 as libc::c_int as libc::c_float
        || (*data.offset(0 as libc::c_int as isize)).temperature
            > 60 as libc::c_int as libc::c_float
    {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    if (*data.offset(0 as libc::c_int as isize)).pressure < 90000 as libc::c_uint as libc::c_float
        || (*data.offset(0 as libc::c_int as isize)).pressure
            > 110000 as libc::c_uint as libc::c_float
    {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    if (*data.offset(0 as libc::c_int as isize)).humidity < 20 as libc::c_uint as libc::c_float
        || (*data.offset(0 as libc::c_int as isize)).humidity > 80 as libc::c_uint as libc::c_float
    {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    i = 0 as libc::c_int as uint8_t;
    while (i as libc::c_int) < n_meas as libc::c_int {
        if (*data.offset(i as isize)).status as libc::c_int & 0x20 as libc::c_int == 0 {
            self_test_failed = self_test_failed.wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
    if n_meas as libc::c_int >= 6 as libc::c_int {
        cent_res = (5 as libc::c_int as libc::c_float
            * ((*data.offset(3 as libc::c_int as isize)).gas_resistance
                + (*data.offset(5 as libc::c_int as isize)).gas_resistance)
            / (2 as libc::c_int as libc::c_float
                * (*data.offset(4 as libc::c_int as isize)).gas_resistance))
            as uint32_t;
    }
    if cent_res < 6 as libc::c_int as libc::c_uint {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    if self_test_failed != 0 {
        rslt = -(5 as libc::c_int) as int8_t;
    }
    return rslt;
}
unsafe extern "C" fn get_calib_data(mut dev: *mut bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut coeff_array: [uint8_t; 42] = [0; 42];
    rslt = bme68x_get_regs(
        0x8a as libc::c_int as uint8_t,
        coeff_array.as_mut_ptr(),
        23 as libc::c_int as uint32_t,
        dev,
    );
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0xe1 as libc::c_int as uint8_t,
            &mut *coeff_array.as_mut_ptr().offset(23 as libc::c_int as isize),
            14 as libc::c_int as uint32_t,
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0 as libc::c_int as uint8_t,
            &mut *coeff_array
                .as_mut_ptr()
                .offset((23 as libc::c_int + 14 as libc::c_int) as isize),
            5 as libc::c_int as uint32_t,
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        (*dev).calib.par_t1 = ((coeff_array[32 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[31 as libc::c_int as usize] as uint16_t as libc::c_int)
            as uint16_t;
        (*dev).calib.par_t2 = ((coeff_array[1 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[0 as libc::c_int as usize] as uint16_t as libc::c_int)
            as int16_t;
        (*dev).calib.par_t3 = coeff_array[2 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_p1 = ((coeff_array[5 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[4 as libc::c_int as usize] as uint16_t as libc::c_int)
            as uint16_t;
        (*dev).calib.par_p2 = ((coeff_array[7 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[6 as libc::c_int as usize] as uint16_t as libc::c_int)
            as int16_t;
        (*dev).calib.par_p3 = coeff_array[8 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_p4 = ((coeff_array[11 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[10 as libc::c_int as usize] as uint16_t as libc::c_int)
            as int16_t;
        (*dev).calib.par_p5 = ((coeff_array[13 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[12 as libc::c_int as usize] as uint16_t as libc::c_int)
            as int16_t;
        (*dev).calib.par_p6 = coeff_array[15 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_p7 = coeff_array[14 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_p8 = ((coeff_array[19 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[18 as libc::c_int as usize] as uint16_t as libc::c_int)
            as int16_t;
        (*dev).calib.par_p9 = ((coeff_array[21 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[20 as libc::c_int as usize] as uint16_t as libc::c_int)
            as int16_t;
        (*dev).calib.par_p10 = coeff_array[22 as libc::c_int as usize];
        (*dev).calib.par_h1 = ((coeff_array[25 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 4 as libc::c_int
            | coeff_array[24 as libc::c_int as usize] as libc::c_int & 0xf as libc::c_int)
            as uint16_t;
        (*dev).calib.par_h2 = ((coeff_array[23 as libc::c_int as usize] as uint16_t as libc::c_int)
            << 4 as libc::c_int
            | coeff_array[24 as libc::c_int as usize] as libc::c_int >> 4 as libc::c_int)
            as uint16_t;
        (*dev).calib.par_h3 = coeff_array[26 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_h4 = coeff_array[27 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_h5 = coeff_array[28 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_h6 = coeff_array[29 as libc::c_int as usize];
        (*dev).calib.par_h7 = coeff_array[30 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_gh1 = coeff_array[35 as libc::c_int as usize] as int8_t;
        (*dev).calib.par_gh2 = ((coeff_array[34 as libc::c_int as usize] as uint16_t
            as libc::c_int)
            << 8 as libc::c_int
            | coeff_array[33 as libc::c_int as usize] as uint16_t as libc::c_int)
            as int16_t;
        (*dev).calib.par_gh3 = coeff_array[36 as libc::c_int as usize] as int8_t;
        (*dev).calib.res_heat_range = ((coeff_array[39 as libc::c_int as usize] as libc::c_int
            & 0x30 as libc::c_int)
            / 16 as libc::c_int) as uint8_t;
        (*dev).calib.res_heat_val = coeff_array[37 as libc::c_int as usize] as int8_t;
        (*dev).calib.range_sw_err = ((coeff_array[41 as libc::c_int as usize] as libc::c_int
            & 0xf0 as libc::c_int) as int8_t as libc::c_int
            / 16 as libc::c_int) as int8_t;
    }
    return rslt;
}
unsafe extern "C" fn read_variant_id(mut dev: *mut bme68x_dev) -> int8_t {
    let mut rslt: int8_t = 0;
    let mut reg_data: uint8_t = 0 as libc::c_int as uint8_t;
    rslt = bme68x_get_regs(
        0xf0 as libc::c_int as uint8_t,
        &mut reg_data,
        1 as libc::c_int as uint32_t,
        dev,
    );
    if rslt as libc::c_int == 0 as libc::c_int {
        (*dev).variant_id = reg_data as uint32_t;
    }
    return rslt;
}
