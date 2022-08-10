use libc;

use crate::interface::{check_rslt, Error, Interface};

#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CommInterface {
    SPI = 0,
    I2C = 1,
}

#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct SensorData {
    pub status: u8,
    pub gas_index: u8,
    pub meas_index: u8,
    pub res_heat: u8,
    pub idac: u8,
    pub gas_wait: u8,
    pub temperature: libc::c_float,
    pub pressure: libc::c_float,
    pub humidity: libc::c_float,
    pub gas_resistance: libc::c_float,
}
#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct CalibrationData {
    pub par_h1: u16,
    pub par_h2: u16,
    pub par_h3: i8,
    pub par_h4: i8,
    pub par_h5: i8,
    pub par_h6: u8,
    pub par_h7: i8,
    pub par_gh1: i8,
    pub par_gh2: i16,
    pub par_gh3: i8,
    pub par_t1: u16,
    pub par_t2: i16,
    pub par_t3: i8,
    pub par_p1: u16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i16,
    pub par_p5: i16,
    pub par_p6: i8,
    pub par_p7: i8,
    pub par_p8: i16,
    pub par_p9: i16,
    pub par_p10: u8,
    pub t_fine: libc::c_float,
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: i8,
}
#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct DeviceConf {
    pub os_hum: u8,
    pub os_temp: u8,
    pub os_pres: u8,
    pub filter: u8,
    pub odr: u8,
}
#[derive(Copy, Clone)]
#[repr(C)]
pub struct HeaterConf {
    pub enable: u8,
    pub heatr_temp: u16,
    pub heatr_dur: u16,
    pub heatr_temp_prof: *mut u16,
    pub heatr_dur_prof: *mut u16,
    pub profile_len: u8,
    pub shared_heatr_dur: u16,
}
impl Default for HeaterConf {
    fn default() -> Self {
        Self {
            enable: 0,
            heatr_temp: 0,
            heatr_dur: 0,
            heatr_temp_prof: 0 as *mut u16,
            heatr_dur_prof: 0 as *mut u16,
            profile_len: 0,
            shared_heatr_dur: 0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
#[repr(C)]
pub struct Device<I: Interface> {
    pub intf: I,
    pub chip_id: u8,
    pub variant_id: u32,
    pub mem_page: u8,
    pub amb_temp: i8,
    pub calib: CalibrationData,
    pub intf_rslt: i8,
    pub info_msg: u8,
}

impl<I: Interface> Device<I> {
    pub fn new(intf: I) -> Self {
        // NOTE moved amb_temp from bme68x_interface_init since everything else that function did
        // is now contained within the interface trait.
        let amb_temp = 25;
        Self {
            chip_id: 0,
            variant_id: 0,
            intf,
            mem_page: 0,
            amb_temp,
            calib: CalibrationData {
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
            intf_rslt: 0,
            info_msg: 0,
        }
    }

    pub fn init(&mut self) -> Result<(), Error> {
        unsafe {
            let mut rslt: i8 = 0;
            rslt = bme68x_soft_reset(self);
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = bme68x_get_regs(
                    0xd0 as libc::c_int as u8,
                    &mut (*self).chip_id,
                    1 as libc::c_int as u32,
                    self,
                );
                if rslt as libc::c_int == 0 as libc::c_int {
                    if (*self).chip_id as libc::c_int == 0x61 as libc::c_int {
                        rslt = read_variant_id(self);
                        if rslt as libc::c_int == 0 as libc::c_int {
                            rslt = get_calib_data(self);
                        }
                    } else {
                        rslt = -(3 as libc::c_int) as i8;
                    }
                }
            }
            check_rslt(rslt)
        }
    }
}
pub unsafe fn bme68x_set_regs<I: Interface>(
    reg_addr: *const u8,
    reg_data: *const u8,
    len: u32,
    mut dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0;
    let mut tmp_buff: [u8; 20] = [
        0 as libc::c_int as u8,
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
    let mut index: u16 = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int && !reg_addr.is_null() && !reg_data.is_null() {
        if len > 0 as libc::c_int as libc::c_uint
            && len <= (20 as libc::c_int / 2 as libc::c_int) as libc::c_uint
        {
            index = 0 as libc::c_int as u16;
            while (index as libc::c_uint) < len {
                if (*dev).intf.interface_type() == CommInterface::SPI {
                    rslt = set_mem_page(*reg_addr.offset(index as isize), dev);
                    tmp_buff[(2 as libc::c_int * index as libc::c_int) as usize] =
                        (*reg_addr.offset(index as isize) as libc::c_int & 0x7f as libc::c_int)
                            as u8;
                } else {
                    tmp_buff[(2 as libc::c_int * index as libc::c_int) as usize] =
                        *reg_addr.offset(index as isize);
                }
                tmp_buff[(2 as libc::c_int * index as libc::c_int + 1 as libc::c_int) as usize] =
                    *reg_data.offset(index as isize);
                index = index.wrapping_add(1);
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                (*dev).intf_rslt = (*dev).intf.write_raw(
                    tmp_buff[0],
                    &mut *tmp_buff.as_mut_ptr().offset(1 as libc::c_int as isize),
                    (2 as libc::c_int as libc::c_uint)
                        .wrapping_mul(len)
                        .wrapping_sub(1 as libc::c_int as libc::c_uint),
                );
                if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
                    rslt = -(2 as libc::c_int) as i8;
                }
            }
        } else {
            rslt = -(4 as libc::c_int) as i8;
        }
    } else {
        rslt = -(1 as libc::c_int) as i8;
    }
    return rslt;
}
pub unsafe fn bme68x_get_regs<I: Interface>(
    mut reg_addr: u8,
    reg_data: *mut u8,
    len: u32,
    mut dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int && !reg_data.is_null() {
        if (*dev).intf.interface_type() == CommInterface::SPI {
            rslt = set_mem_page(reg_addr, dev);
            if rslt as libc::c_int == 0 as libc::c_int {
                reg_addr = (reg_addr as libc::c_int | 0x80 as libc::c_int) as u8;
            }
        }
        (*dev).intf_rslt = (*dev).intf.read_raw(reg_addr, reg_data, len);
        if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
            rslt = -(2 as libc::c_int) as i8;
        }
    } else {
        rslt = -(1 as libc::c_int) as i8;
    }
    return rslt;
}
pub unsafe fn bme68x_soft_reset<I: Interface>(dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut reg_addr: u8 = 0xe0 as libc::c_int as u8;
    let mut soft_rst_cmd: u8 = 0xb6 as libc::c_int as u8;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        if (*dev).intf.interface_type() == CommInterface::SPI {
            rslt = get_mem_page(dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = bme68x_set_regs(
                &mut reg_addr,
                &mut soft_rst_cmd,
                1 as libc::c_int as u32,
                dev,
            );
            (*dev).intf.delay(10000 as libc::c_uint);
            if rslt as libc::c_int == 0 as libc::c_int {
                if (*dev).intf.interface_type() == CommInterface::SPI {
                    rslt = get_mem_page(dev);
                }
            }
        }
    }
    return rslt;
}

impl<I: Interface> Device<I> {
    pub fn set_conf(&mut self, conf: *mut DeviceConf) -> Result<(), Error> {
        unsafe {
            let mut rslt: i8 = 0;
            let mut odr20: u8 = 0 as libc::c_int as u8;
            let mut odr3: u8 = 1 as libc::c_int as u8;
            let mut current_op_mode: u8 = 0;
            let mut reg_array: [u8; 5] = [
                0x71 as libc::c_int as u8,
                0x72 as libc::c_int as u8,
                0x73 as libc::c_int as u8,
                0x74 as libc::c_int as u8,
                0x75 as libc::c_int as u8,
            ];
            let mut data_array: [u8; 5] = [0 as libc::c_int as u8, 0, 0, 0, 0];
            rslt = bme68x_get_op_mode(&mut current_op_mode, self);
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = bme68x_set_op_mode(0 as libc::c_int as u8, self);
            }
            if conf.is_null() {
                rslt = -(1 as libc::c_int) as i8;
            } else if rslt as libc::c_int == 0 as libc::c_int {
                rslt = bme68x_get_regs(
                    reg_array[0],
                    data_array.as_mut_ptr(),
                    5 as libc::c_int as u32,
                    self,
                );
                (*self).info_msg = 0 as libc::c_int as u8;
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = boundary_check(&mut (*conf).filter, 7 as libc::c_int as u8, self);
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = boundary_check(&mut (*conf).os_temp, 5 as libc::c_int as u8, self);
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = boundary_check(&mut (*conf).os_pres, 5 as libc::c_int as u8, self);
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = boundary_check(&mut (*conf).os_hum, 5 as libc::c_int as u8, self);
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = boundary_check(&mut (*conf).odr, 8 as libc::c_int as u8, self);
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    data_array[4] = (data_array[4] as libc::c_int & !(0x1c as libc::c_int)
                        | ((*conf).filter as libc::c_int) << 2 as libc::c_int & 0x1c as libc::c_int)
                        as u8;
                    data_array[3] = (data_array[3] as libc::c_int & !(0xe0 as libc::c_int)
                        | ((*conf).os_temp as libc::c_int) << 5 as libc::c_int
                            & 0xe0 as libc::c_int) as u8;
                    data_array[3] = (data_array[3] as libc::c_int & !(0x1c as libc::c_int)
                        | ((*conf).os_pres as libc::c_int) << 2 as libc::c_int
                            & 0x1c as libc::c_int) as u8;
                    data_array[1] = (data_array[1] as libc::c_int & !(0x7 as libc::c_int)
                        | (*conf).os_hum as libc::c_int & 0x7 as libc::c_int)
                        as u8;
                    if (*conf).odr as libc::c_int != 8 as libc::c_int {
                        odr20 = (*conf).odr;
                        odr3 = 0 as libc::c_int as u8;
                    }
                    data_array[4] = (data_array[4] as libc::c_int & !(0xe0 as libc::c_int)
                        | (odr20 as libc::c_int) << 5 as libc::c_int & 0xe0 as libc::c_int)
                        as u8;
                    data_array[0] = (data_array[0] as libc::c_int & !(0x80 as libc::c_int)
                        | (odr3 as libc::c_int) << 7 as libc::c_int & 0x80 as libc::c_int)
                        as u8;
                }
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = bme68x_set_regs(
                    reg_array.as_mut_ptr(),
                    data_array.as_mut_ptr(),
                    5 as libc::c_int as u32,
                    self,
                );
            }
            if current_op_mode as libc::c_int != 0 as libc::c_int
                && rslt as libc::c_int == 0 as libc::c_int
            {
                rslt = bme68x_set_op_mode(current_op_mode, self);
            }
            return check_rslt(rslt);
        }
    }
}
pub unsafe fn bme68x_get_conf<I: Interface>(mut conf: *mut DeviceConf, dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let reg_addr: u8 = 0x71 as libc::c_int as u8;
    let mut data_array: [u8; 5] = [0; 5];
    rslt = bme68x_get_regs(
        reg_addr,
        data_array.as_mut_ptr(),
        5 as libc::c_int as u32,
        dev,
    );
    if conf.is_null() {
        rslt = -(1 as libc::c_int) as i8;
    } else if rslt as libc::c_int == 0 as libc::c_int {
        (*conf).os_hum = (data_array[1] as libc::c_int & 0x7 as libc::c_int) as u8;
        (*conf).filter =
            ((data_array[4] as libc::c_int & 0x1c as libc::c_int) >> 2 as libc::c_int) as u8;
        (*conf).os_temp =
            ((data_array[3] as libc::c_int & 0xe0 as libc::c_int) >> 5 as libc::c_int) as u8;
        (*conf).os_pres =
            ((data_array[3] as libc::c_int & 0x1c as libc::c_int) >> 2 as libc::c_int) as u8;
        if (data_array[0] as libc::c_int & 0x80 as libc::c_int) >> 7 as libc::c_int != 0 {
            (*conf).odr = 8 as libc::c_int as u8;
        } else {
            (*conf).odr =
                ((data_array[4] as libc::c_int & 0xe0 as libc::c_int) >> 5 as libc::c_int) as u8;
        }
    }
    return rslt;
}
pub unsafe fn bme68x_set_op_mode<I: Interface>(op_mode: u8, dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut tmp_pow_mode: u8 = 0;
    let mut pow_mode: u8 = 0 as libc::c_int as u8;
    let mut reg_addr: u8 = 0x74 as libc::c_int as u8;
    loop {
        rslt = bme68x_get_regs(
            0x74 as libc::c_int as u8,
            &mut tmp_pow_mode,
            1 as libc::c_int as u32,
            dev,
        );
        if rslt as libc::c_int == 0 as libc::c_int {
            pow_mode = (tmp_pow_mode as libc::c_int & 0x3 as libc::c_int) as u8;
            if pow_mode as libc::c_int != 0 as libc::c_int {
                tmp_pow_mode = (tmp_pow_mode as libc::c_int & !(0x3 as libc::c_int)) as u8;
                rslt = bme68x_set_regs(
                    &mut reg_addr,
                    &mut tmp_pow_mode,
                    1 as libc::c_int as u32,
                    dev,
                );
                (*dev).intf.delay(10000 as libc::c_uint);
            }
        }
        if !(pow_mode as libc::c_int != 0 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int)
        {
            break;
        }
    }
    if op_mode as libc::c_int != 0 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
        tmp_pow_mode = (tmp_pow_mode as libc::c_int & !(0x3 as libc::c_int)
            | op_mode as libc::c_int & 0x3 as libc::c_int) as u8;
        rslt = bme68x_set_regs(
            &mut reg_addr,
            &mut tmp_pow_mode,
            1 as libc::c_int as u32,
            dev,
        );
    }
    return rslt;
}
pub unsafe fn bme68x_get_op_mode<I: Interface>(op_mode: *mut u8, dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut mode: u8 = 0;
    if !op_mode.is_null() {
        rslt = bme68x_get_regs(
            0x74 as libc::c_int as u8,
            &mut mode,
            1 as libc::c_int as u32,
            dev,
        );
        *op_mode = (mode as libc::c_int & 0x3 as libc::c_int) as u8;
    } else {
        rslt = -(1 as libc::c_int) as i8;
    }
    return rslt;
}
pub unsafe fn bme68x_get_meas_dur<I: Interface>(
    op_mode: u8,
    conf: *mut DeviceConf,
    dev: *mut Device<I>,
) -> u32 {
    let mut rslt: i8 = 0;
    let mut meas_dur: u32 = 0 as libc::c_int as u32;
    let mut meas_cycles: u32 = 0;
    let os_to_meas_cycles: [u8; 6] = [
        0 as libc::c_int as u8,
        1 as libc::c_int as u8,
        2 as libc::c_int as u8,
        4 as libc::c_int as u8,
        8 as libc::c_int as u8,
        16 as libc::c_int as u8,
    ];
    if !conf.is_null() {
        rslt = boundary_check(&mut (*conf).os_temp, 5 as libc::c_int as u8, dev);
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).os_pres, 5 as libc::c_int as u8, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = boundary_check(&mut (*conf).os_hum, 5 as libc::c_int as u8, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            meas_cycles = os_to_meas_cycles[(*conf).os_temp as usize] as u32;
            meas_cycles = (meas_cycles as libc::c_uint)
                .wrapping_add(os_to_meas_cycles[(*conf).os_pres as usize] as libc::c_uint)
                as u32 as u32;
            meas_cycles = (meas_cycles as libc::c_uint)
                .wrapping_add(os_to_meas_cycles[(*conf).os_hum as usize] as libc::c_uint)
                as u32 as u32;
            meas_dur = meas_cycles.wrapping_mul(1963 as libc::c_uint);
            meas_dur = (meas_dur as libc::c_uint)
                .wrapping_add((477 as libc::c_int as libc::c_uint).wrapping_mul(4 as libc::c_uint))
                as u32 as u32;
            meas_dur = (meas_dur as libc::c_uint)
                .wrapping_add((477 as libc::c_int as libc::c_uint).wrapping_mul(5 as libc::c_uint))
                as u32 as u32;
            if op_mode as libc::c_int != 2 as libc::c_int {
                meas_dur =
                    (meas_dur as libc::c_uint).wrapping_add(1000 as libc::c_uint) as u32 as u32;
            }
        }
    }
    return meas_dur;
}
pub unsafe fn bme68x_get_data<I: Interface>(
    op_mode: u8,
    data: *mut SensorData,
    n_data: *mut u8,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0;
    let mut i: u8 = 0 as libc::c_int as u8;
    let mut j: u8 = 0 as libc::c_int as u8;
    let mut new_fields: u8 = 0 as libc::c_int as u8;
    let mut field_ptr: [*mut SensorData; 3] = [
        0 as *mut SensorData,
        0 as *mut SensorData,
        0 as *mut SensorData,
    ];
    let mut field_data: [SensorData; 3] = [
        {
            let init = SensorData {
                status: 0 as libc::c_int as u8,
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
        SensorData {
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
        SensorData {
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
    field_ptr[0] =
        &mut *field_data.as_mut_ptr().offset(0 as libc::c_int as isize) as *mut SensorData;
    field_ptr[1] =
        &mut *field_data.as_mut_ptr().offset(1 as libc::c_int as isize) as *mut SensorData;
    field_ptr[2] =
        &mut *field_data.as_mut_ptr().offset(2 as libc::c_int as isize) as *mut SensorData;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int && !data.is_null() {
        if op_mode as libc::c_int == 1 as libc::c_int {
            rslt = read_field_data(0 as libc::c_int as u8, data, dev);
            if rslt as libc::c_int == 0 as libc::c_int {
                if (*data).status as libc::c_int & 0x80 as libc::c_int != 0 {
                    new_fields = 1 as libc::c_int as u8;
                } else {
                    new_fields = 0 as libc::c_int as u8;
                    rslt = 2 as libc::c_int as i8;
                }
            }
        } else if op_mode as libc::c_int == 2 as libc::c_int
            || op_mode as libc::c_int == 3 as libc::c_int
        {
            rslt = read_all_field_data(field_ptr.as_mut_ptr() as *const *mut SensorData, dev);
            new_fields = 0 as libc::c_int as u8;
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < 3 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
                if (*field_ptr[i as usize]).status as libc::c_int & 0x80 as libc::c_int != 0 {
                    new_fields = new_fields.wrapping_add(1);
                }
                i = i.wrapping_add(1);
            }
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < 2 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
                j = (i as libc::c_int + 1 as libc::c_int) as u8;
                while (j as libc::c_int) < 3 as libc::c_int {
                    sort_sensor_data(i, j, field_ptr.as_mut_ptr());
                    j = j.wrapping_add(1);
                }
                i = i.wrapping_add(1);
            }
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < 3 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
                *data.offset(i as isize) = *field_ptr[i as usize];
                i = i.wrapping_add(1);
            }
            if new_fields as libc::c_int == 0 as libc::c_int {
                rslt = 2 as libc::c_int as i8;
            }
        } else {
            rslt = 1 as libc::c_int as i8;
        }
        if n_data.is_null() {
            rslt = -(1 as libc::c_int) as i8;
        } else {
            *n_data = new_fields;
        }
    } else {
        rslt = -(1 as libc::c_int) as i8;
    }
    return rslt;
}
pub unsafe fn bme68x_set_heatr_conf<I: Interface>(
    op_mode: u8,
    conf: *const HeaterConf,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0;
    let mut nb_conv: u8 = 0 as libc::c_int as u8;
    let mut hctrl: u8 = 0;
    let mut run_gas: u8 = 0 as libc::c_int as u8;
    let mut ctrl_gas_data: [u8; 2] = [0; 2];
    let mut ctrl_gas_addr: [u8; 2] = [0x70 as libc::c_int as u8, 0x71 as libc::c_int as u8];
    if !conf.is_null() {
        rslt = bme68x_set_op_mode(0 as libc::c_int as u8, dev);
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = set_conf(conf, op_mode, &mut nb_conv, dev);
        }
        if rslt as libc::c_int == 0 as libc::c_int {
            rslt = bme68x_get_regs(
                0x70 as libc::c_int as u8,
                ctrl_gas_data.as_mut_ptr(),
                2 as libc::c_int as u32,
                dev,
            );
            if rslt as libc::c_int == 0 as libc::c_int {
                if (*conf).enable as libc::c_int == 0x1 as libc::c_int {
                    hctrl = 0 as libc::c_int as u8;
                    if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
                        run_gas = 0x2 as libc::c_int as u8;
                    } else {
                        run_gas = 0x1 as libc::c_int as u8;
                    }
                } else {
                    hctrl = 0x1 as libc::c_int as u8;
                    run_gas = 0 as libc::c_int as u8;
                }
                ctrl_gas_data[0] = (ctrl_gas_data[0] as libc::c_int & !(0x8 as libc::c_int)
                    | (hctrl as libc::c_int) << 3 as libc::c_int & 0x8 as libc::c_int)
                    as u8;
                ctrl_gas_data[1] = (ctrl_gas_data[1] as libc::c_int & !(0xf as libc::c_int)
                    | nb_conv as libc::c_int & 0xf as libc::c_int)
                    as u8;
                ctrl_gas_data[1] = (ctrl_gas_data[1] as libc::c_int & !(0x30 as libc::c_int)
                    | (run_gas as libc::c_int) << 4 as libc::c_int & 0x30 as libc::c_int)
                    as u8;
                rslt = bme68x_set_regs(
                    ctrl_gas_addr.as_mut_ptr(),
                    ctrl_gas_data.as_mut_ptr(),
                    2 as libc::c_int as u32,
                    dev,
                );
            }
        }
    } else {
        rslt = -(1 as libc::c_int) as i8;
    }
    return rslt;
}
pub unsafe fn bme68x_get_heatr_conf<I: Interface>(
    conf: *const HeaterConf,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0;
    let mut data_array: [u8; 10] = [0 as libc::c_int as u8, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    let mut i: u8 = 0;
    rslt = bme68x_get_regs(
        0x5a as libc::c_int as u8,
        data_array.as_mut_ptr(),
        10 as libc::c_int as u32,
        dev,
    );
    if rslt as libc::c_int == 0 as libc::c_int {
        if !conf.is_null()
            && !((*conf).heatr_dur_prof).is_null()
            && !((*conf).heatr_temp_prof).is_null()
        {
            i = 0 as libc::c_int as u8;
            while (i as libc::c_int) < 10 as libc::c_int {
                *((*conf).heatr_temp_prof).offset(i as isize) = data_array[i as usize] as u16;
                i = i.wrapping_add(1);
            }
            rslt = bme68x_get_regs(
                0x64 as libc::c_int as u8,
                data_array.as_mut_ptr(),
                10 as libc::c_int as u32,
                dev,
            );
            if rslt as libc::c_int == 0 as libc::c_int {
                i = 0 as libc::c_int as u8;
                while (i as libc::c_int) < 10 as libc::c_int {
                    *((*conf).heatr_dur_prof).offset(i as isize) = data_array[i as usize] as u16;
                    i = i.wrapping_add(1);
                }
            }
        } else {
            rslt = -(1 as libc::c_int) as i8;
        }
    }
    return rslt;
}

unsafe fn calc_temperature<I: Interface>(temp_adc: u32, mut dev: *mut Device<I>) -> libc::c_float {
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
unsafe fn calc_pressure<I: Interface>(pres_adc: u32, dev: *const Device<I>) -> libc::c_float {
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
unsafe fn calc_humidity<I: Interface>(hum_adc: u16, dev: *const Device<I>) -> libc::c_float {
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
unsafe fn calc_gas_resistance_low<I: Interface>(
    gas_res_adc: u16,
    gas_range: u8,
    dev: *const Device<I>,
) -> libc::c_float {
    let mut calc_gas_res: libc::c_float = 0.;
    let mut var1: libc::c_float = 0.;
    let mut var2: libc::c_float = 0.;
    let mut var3: libc::c_float = 0.;
    let gas_res_f: libc::c_float = gas_res_adc as libc::c_float;
    let gas_range_f: libc::c_float =
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
unsafe fn calc_gas_resistance_high(gas_res_adc: u16, gas_range: u8) -> libc::c_float {
    let mut calc_gas_res: libc::c_float = 0.;
    let var1: u32 = 262144 as libc::c_uint >> gas_range as libc::c_int;
    let mut var2: i32 = gas_res_adc as i32 - 512 as libc::c_int;
    var2 *= 3 as libc::c_int;
    var2 = 4096 as libc::c_int + var2;
    calc_gas_res = 1000000.0f32 * var1 as libc::c_float / var2 as libc::c_float;
    return calc_gas_res;
}
unsafe fn calc_res_heat<I: Interface>(mut temp: u16, dev: *const Device<I>) -> u8 {
    let mut var1: libc::c_float = 0.;
    let mut var2: libc::c_float = 0.;
    let mut var3: libc::c_float = 0.;
    let mut var4: libc::c_float = 0.;
    let mut var5: libc::c_float = 0.;
    let mut res_heat: u8 = 0;
    if temp as libc::c_int > 400 as libc::c_int {
        temp = 400 as libc::c_int as u16;
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
            - 25 as libc::c_int as libc::c_float)) as u8;
    return res_heat;
}
unsafe fn calc_gas_wait(mut dur: u16) -> u8 {
    let mut factor: u8 = 0 as libc::c_int as u8;
    let mut durval: u8 = 0;
    if dur as libc::c_int >= 0xfc0 as libc::c_int {
        durval = 0xff as libc::c_int as u8;
    } else {
        while dur as libc::c_int > 0x3f as libc::c_int {
            dur = (dur as libc::c_int / 4 as libc::c_int) as u16;
            factor = (factor as libc::c_int + 1 as libc::c_int) as u8;
        }
        durval = (dur as libc::c_int + factor as libc::c_int * 64 as libc::c_int) as u8;
    }
    return durval;
}
unsafe fn read_field_data<I: Interface>(
    index: u8,
    mut data: *mut SensorData,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0 as libc::c_int as i8;
    let mut buff: [u8; 17] = [
        0 as libc::c_int as u8,
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
    let mut gas_range_l: u8 = 0;
    let mut gas_range_h: u8 = 0;
    let mut adc_temp: u32 = 0;
    let mut adc_pres: u32 = 0;
    let mut adc_hum: u16 = 0;
    let mut adc_gas_res_low: u16 = 0;
    let mut adc_gas_res_high: u16 = 0;
    let mut tries: u8 = 5 as libc::c_int as u8;
    while tries as libc::c_int != 0 && rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            (0x1d as libc::c_int + index as libc::c_int * 17 as libc::c_int) as u8,
            buff.as_mut_ptr(),
            17 as libc::c_int as u16 as u32,
            dev,
        );
        if data.is_null() {
            rslt = -(1 as libc::c_int) as i8;
            break;
        } else {
            (*data).status = (buff[0] as libc::c_int & 0x80 as libc::c_int) as u8;
            (*data).gas_index = (buff[0] as libc::c_int & 0xf as libc::c_int) as u8;
            (*data).meas_index = buff[1];
            adc_pres = (buff[2] as u32).wrapping_mul(4096 as libc::c_int as libc::c_uint)
                | (buff[3] as u32).wrapping_mul(16 as libc::c_int as libc::c_uint)
                | (buff[4] as u32).wrapping_div(16 as libc::c_int as libc::c_uint);
            adc_temp = (buff[5] as u32).wrapping_mul(4096 as libc::c_int as libc::c_uint)
                | (buff[6] as u32).wrapping_mul(16 as libc::c_int as libc::c_uint)
                | (buff[7] as u32).wrapping_div(16 as libc::c_int as libc::c_uint);
            adc_hum = ((buff[8] as u32).wrapping_mul(256 as libc::c_int as libc::c_uint)
                | buff[9] as u32) as u16;
            adc_gas_res_low = ((buff[13] as u32).wrapping_mul(4 as libc::c_int as libc::c_uint)
                | (buff[14] as u32).wrapping_div(64 as libc::c_int as libc::c_uint))
                as u16;
            adc_gas_res_high = ((buff[15] as u32).wrapping_mul(4 as libc::c_int as libc::c_uint)
                | (buff[16] as u32).wrapping_div(64 as libc::c_int as libc::c_uint))
                as u16;
            gas_range_l = (buff[14] as libc::c_int & 0xf as libc::c_int) as u8;
            gas_range_h = (buff[16] as libc::c_int & 0xf as libc::c_int) as u8;
            if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
                let ref mut fresh0 = (*data).status;
                *fresh0 =
                    (*fresh0 as libc::c_int | buff[16] as libc::c_int & 0x20 as libc::c_int) as u8;
                let ref mut fresh1 = (*data).status;
                *fresh1 =
                    (*fresh1 as libc::c_int | buff[16] as libc::c_int & 0x10 as libc::c_int) as u8;
            } else {
                let ref mut fresh2 = (*data).status;
                *fresh2 =
                    (*fresh2 as libc::c_int | buff[14] as libc::c_int & 0x20 as libc::c_int) as u8;
                let ref mut fresh3 = (*data).status;
                *fresh3 =
                    (*fresh3 as libc::c_int | buff[14] as libc::c_int & 0x10 as libc::c_int) as u8;
            }
            if (*data).status as libc::c_int & 0x80 as libc::c_int != 0
                && rslt as libc::c_int == 0 as libc::c_int
            {
                rslt = bme68x_get_regs(
                    (0x5a as libc::c_int + (*data).gas_index as libc::c_int) as u8,
                    &mut (*data).res_heat,
                    1 as libc::c_int as u32,
                    dev,
                );
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = bme68x_get_regs(
                        (0x50 as libc::c_int + (*data).gas_index as libc::c_int) as u8,
                        &mut (*data).idac,
                        1 as libc::c_int as u32,
                        dev,
                    );
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = bme68x_get_regs(
                        (0x64 as libc::c_int + (*data).gas_index as libc::c_int) as u8,
                        &mut (*data).gas_wait,
                        1 as libc::c_int as u32,
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
                (*dev).intf.delay(10000 as libc::c_uint);
            }
            tries = tries.wrapping_sub(1);
        }
    }
    return rslt;
}
unsafe fn read_all_field_data<I: Interface>(
    data: *const *mut SensorData,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0 as libc::c_int as i8;
    let mut buff: [u8; 51] = [
        0 as libc::c_int as u8,
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
    let mut gas_range_l: u8 = 0;
    let mut gas_range_h: u8 = 0;
    let mut adc_temp: u32 = 0;
    let mut adc_pres: u32 = 0;
    let mut adc_hum: u16 = 0;
    let mut adc_gas_res_low: u16 = 0;
    let mut adc_gas_res_high: u16 = 0;
    let mut off: u8 = 0;
    let mut set_val: [u8; 30] = [
        0 as libc::c_int as u8,
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
    let mut i: u8 = 0;
    if (*data.offset(0 as libc::c_int as isize)).is_null()
        && (*data.offset(1 as libc::c_int as isize)).is_null()
        && (*data.offset(2 as libc::c_int as isize)).is_null()
    {
        rslt = -(1 as libc::c_int) as i8;
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0x1d as libc::c_int as u8,
            buff.as_mut_ptr(),
            (17 as libc::c_int as u32).wrapping_mul(3 as libc::c_int as libc::c_uint),
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0x50 as libc::c_int as u8,
            set_val.as_mut_ptr(),
            30 as libc::c_int as u32,
            dev,
        );
    }
    i = 0 as libc::c_int as u8;
    while (i as libc::c_int) < 3 as libc::c_int && rslt as libc::c_int == 0 as libc::c_int {
        off = (i as libc::c_int * 17 as libc::c_int) as u8;
        (**data.offset(i as isize)).status =
            (buff[off as usize] as libc::c_int & 0x80 as libc::c_int) as u8;
        (**data.offset(i as isize)).gas_index =
            (buff[off as usize] as libc::c_int & 0xf as libc::c_int) as u8;
        (**data.offset(i as isize)).meas_index =
            buff[(off as libc::c_int + 1 as libc::c_int) as usize];
        adc_pres = (buff[(off as libc::c_int + 2 as libc::c_int) as usize] as u32)
            .wrapping_mul(4096 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 3 as libc::c_int) as usize] as u32)
                .wrapping_mul(16 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 4 as libc::c_int) as usize] as u32)
                .wrapping_div(16 as libc::c_int as libc::c_uint);
        adc_temp = (buff[(off as libc::c_int + 5 as libc::c_int) as usize] as u32)
            .wrapping_mul(4096 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 6 as libc::c_int) as usize] as u32)
                .wrapping_mul(16 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 7 as libc::c_int) as usize] as u32)
                .wrapping_div(16 as libc::c_int as libc::c_uint);
        adc_hum = ((buff[(off as libc::c_int + 8 as libc::c_int) as usize] as u32)
            .wrapping_mul(256 as libc::c_int as libc::c_uint)
            | buff[(off as libc::c_int + 9 as libc::c_int) as usize] as u32)
            as u16;
        adc_gas_res_low = ((buff[(off as libc::c_int + 13 as libc::c_int) as usize] as u32)
            .wrapping_mul(4 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 14 as libc::c_int) as usize] as u32)
                .wrapping_div(64 as libc::c_int as libc::c_uint)) as u16;
        adc_gas_res_high = ((buff[(off as libc::c_int + 15 as libc::c_int) as usize] as u32)
            .wrapping_mul(4 as libc::c_int as libc::c_uint)
            | (buff[(off as libc::c_int + 16 as libc::c_int) as usize] as u32)
                .wrapping_div(64 as libc::c_int as libc::c_uint)) as u16;
        gas_range_l = (buff[(off as libc::c_int + 14 as libc::c_int) as usize] as libc::c_int
            & 0xf as libc::c_int) as u8;
        gas_range_h = (buff[(off as libc::c_int + 16 as libc::c_int) as usize] as libc::c_int
            & 0xf as libc::c_int) as u8;
        if (*dev).variant_id == 0x1 as libc::c_int as libc::c_uint {
            let ref mut fresh4 = (**data.offset(i as isize)).status;
            *fresh4 = (*fresh4 as libc::c_int
                | buff[(off as libc::c_int + 16 as libc::c_int) as usize] as libc::c_int
                    & 0x20 as libc::c_int) as u8;
            let ref mut fresh5 = (**data.offset(i as isize)).status;
            *fresh5 = (*fresh5 as libc::c_int
                | buff[(off as libc::c_int + 16 as libc::c_int) as usize] as libc::c_int
                    & 0x10 as libc::c_int) as u8;
        } else {
            let ref mut fresh6 = (**data.offset(i as isize)).status;
            *fresh6 = (*fresh6 as libc::c_int
                | buff[(off as libc::c_int + 14 as libc::c_int) as usize] as libc::c_int
                    & 0x20 as libc::c_int) as u8;
            let ref mut fresh7 = (**data.offset(i as isize)).status;
            *fresh7 = (*fresh7 as libc::c_int
                | buff[(off as libc::c_int + 14 as libc::c_int) as usize] as libc::c_int
                    & 0x10 as libc::c_int) as u8;
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
unsafe fn set_mem_page<I: Interface>(reg_addr: u8, mut dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut reg: u8 = 0;
    let mut mem_page: u8 = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        if reg_addr as libc::c_int > 0x7f as libc::c_int {
            mem_page = 0 as libc::c_int as u8;
        } else {
            mem_page = 0x10 as libc::c_int as u8;
        }
        if mem_page as libc::c_int != (*dev).mem_page as libc::c_int {
            (*dev).mem_page = mem_page;
            (*dev).intf_rslt = (*dev).intf.read_raw(
                (0xf3 as libc::c_int | 0x80 as libc::c_int) as u8,
                &mut reg,
                1 as libc::c_int as u32,
            );
            if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
                rslt = -(2 as libc::c_int) as i8;
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                reg = (reg as libc::c_int & !(0x10 as libc::c_int)) as u8;
                reg = (reg as libc::c_int | (*dev).mem_page as libc::c_int & 0x10 as libc::c_int)
                    as u8;
                (*dev).intf_rslt = (*dev).intf.write_raw(
                    (0xf3 as libc::c_int & 0x7f as libc::c_int) as u8,
                    &mut reg,
                    1 as libc::c_int as u32,
                );
                if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
                    rslt = -(2 as libc::c_int) as i8;
                }
            }
        }
    }
    return rslt;
}
unsafe fn get_mem_page<I: Interface>(mut dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut reg: u8 = 0;
    rslt = null_ptr_check(dev);
    if rslt as libc::c_int == 0 as libc::c_int {
        (*dev).intf_rslt = (*dev).intf.read_raw(
            (0xf3 as libc::c_int | 0x80 as libc::c_int) as u8,
            &mut reg,
            1 as libc::c_int as u32,
        );
        if (*dev).intf_rslt as libc::c_int != 0 as libc::c_int {
            rslt = -(2 as libc::c_int) as i8;
        } else {
            (*dev).mem_page = (reg as libc::c_int & 0x10 as libc::c_int) as u8;
        }
    }
    return rslt;
}
unsafe fn boundary_check<I: Interface>(value: *mut u8, max: u8, dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    rslt = null_ptr_check(dev);
    if !value.is_null() && rslt as libc::c_int == 0 as libc::c_int {
        if *value as libc::c_int > max as libc::c_int {
            *value = max;
            let ref mut fresh8 = (*dev).info_msg;
            *fresh8 = (*fresh8 as libc::c_int | 1 as libc::c_int) as u8;
        }
    } else {
        rslt = -(1 as libc::c_int) as i8;
    }
    return rslt;
}
unsafe fn null_ptr_check<I: Interface>(dev: *const Device<I>) -> i8 {
    let mut rslt: i8 = 0 as libc::c_int as i8;
    if dev.is_null() {
        rslt = -(1 as libc::c_int) as i8;
    }
    return rslt;
}
unsafe fn set_conf<I: Interface>(
    conf: *const HeaterConf,
    op_mode: u8,
    nb_conv: *mut u8,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0 as libc::c_int as i8;
    let mut i: u8 = 0;
    let mut shared_dur: u8 = 0;
    let mut write_len: u8 = 0 as libc::c_int as u8;
    let mut heater_dur_shared_addr: u8 = 0x6e as libc::c_int as u8;
    let mut rh_reg_addr: [u8; 10] = [
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
    ];
    let mut rh_reg_data: [u8; 10] = [
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
    ];
    let mut gw_reg_addr: [u8; 10] = [
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
    ];
    let mut gw_reg_data: [u8; 10] = [
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
        0 as libc::c_int as u8,
    ];
    match op_mode as libc::c_int {
        1 => {
            rh_reg_addr[0] = 0x5a as libc::c_int as u8;
            rh_reg_data[0] = calc_res_heat((*conf).heatr_temp, dev);
            gw_reg_addr[0] = 0x64 as libc::c_int as u8;
            gw_reg_data[0] = calc_gas_wait((*conf).heatr_dur);
            *nb_conv = 0 as libc::c_int as u8;
            write_len = 1 as libc::c_int as u8;
        }
        3 => {
            if ((*conf).heatr_dur_prof).is_null() || ((*conf).heatr_temp_prof).is_null() {
                rslt = -(1 as libc::c_int) as i8;
            } else {
                i = 0 as libc::c_int as u8;
                while (i as libc::c_int) < (*conf).profile_len as libc::c_int {
                    rh_reg_addr[i as usize] = (0x5a as libc::c_int + i as libc::c_int) as u8;
                    rh_reg_data[i as usize] =
                        calc_res_heat(*((*conf).heatr_temp_prof).offset(i as isize), dev);
                    gw_reg_addr[i as usize] = (0x64 as libc::c_int + i as libc::c_int) as u8;
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
                rslt = -(1 as libc::c_int) as i8;
            } else {
                if (*conf).shared_heatr_dur as libc::c_int == 0 as libc::c_int {
                    rslt = 3 as libc::c_int as i8;
                }
                i = 0 as libc::c_int as u8;
                while (i as libc::c_int) < (*conf).profile_len as libc::c_int {
                    rh_reg_addr[i as usize] = (0x5a as libc::c_int + i as libc::c_int) as u8;
                    rh_reg_data[i as usize] =
                        calc_res_heat(*((*conf).heatr_temp_prof).offset(i as isize), dev);
                    gw_reg_addr[i as usize] = (0x64 as libc::c_int + i as libc::c_int) as u8;
                    gw_reg_data[i as usize] = *((*conf).heatr_dur_prof).offset(i as isize) as u8;
                    i = i.wrapping_add(1);
                }
                *nb_conv = (*conf).profile_len;
                write_len = (*conf).profile_len;
                shared_dur = calc_heatr_dur_shared((*conf).shared_heatr_dur);
                if rslt as libc::c_int == 0 as libc::c_int {
                    rslt = bme68x_set_regs(
                        &mut heater_dur_shared_addr,
                        &mut shared_dur,
                        1 as libc::c_int as u32,
                        dev,
                    );
                }
            }
        }
        _ => {
            rslt = 1 as libc::c_int as i8;
        }
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_set_regs(
            rh_reg_addr.as_mut_ptr(),
            rh_reg_data.as_mut_ptr(),
            write_len as u32,
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_set_regs(
            gw_reg_addr.as_mut_ptr(),
            gw_reg_data.as_mut_ptr(),
            write_len as u32,
            dev,
        );
    }
    return rslt;
}
unsafe fn calc_heatr_dur_shared(mut dur: u16) -> u8 {
    let mut factor: u8 = 0 as libc::c_int as u8;
    let mut heatdurval: u8 = 0;
    if dur as libc::c_int >= 0x783 as libc::c_int {
        heatdurval = 0xff as libc::c_int as u8;
    } else {
        dur = (dur as u32)
            .wrapping_mul(1000 as libc::c_int as libc::c_uint)
            .wrapping_div(477 as libc::c_int as libc::c_uint) as u16;
        while dur as libc::c_int > 0x3f as libc::c_int {
            dur = (dur as libc::c_int >> 2 as libc::c_int) as u16;
            factor = (factor as libc::c_int + 1 as libc::c_int) as u8;
        }
        heatdurval = (dur as libc::c_int + factor as libc::c_int * 64 as libc::c_int) as u8;
    }
    return heatdurval;
}
unsafe fn sort_sensor_data(low_index: u8, high_index: u8, field: *mut *mut SensorData) {
    let mut meas_index1: i16 = 0;
    let mut meas_index2: i16 = 0;
    meas_index1 = (**field.offset(low_index as isize)).meas_index as i16;
    meas_index2 = (**field.offset(high_index as isize)).meas_index as i16;
    if (**field.offset(low_index as isize)).status as libc::c_int & 0x80 as libc::c_int != 0
        && (**field.offset(high_index as isize)).status as libc::c_int & 0x80 as libc::c_int != 0
    {
        let diff: i16 = (meas_index2 as libc::c_int - meas_index1 as libc::c_int) as i16;
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
unsafe fn swap_fields(index1: u8, index2: u8, field: *mut *mut SensorData) {
    let mut temp: *mut SensorData = 0 as *mut SensorData;
    temp = *field.offset(index1 as isize);
    let ref mut fresh9 = *field.offset(index1 as isize);
    *fresh9 = *field.offset(index2 as isize);
    let ref mut fresh10 = *field.offset(index2 as isize);
    *fresh10 = temp;
}
unsafe fn analyze_sensor_data(data: *const SensorData, n_meas: u8) -> i8 {
    let mut rslt: i8 = 0 as libc::c_int as i8;
    let mut self_test_failed: u8 = 0 as libc::c_int as u8;
    let mut i: u8 = 0;
    let mut cent_res: u32 = 0 as libc::c_int as u32;
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
    i = 0 as libc::c_int as u8;
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
            as u32;
    }
    if cent_res < 6 as libc::c_int as libc::c_uint {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    if self_test_failed != 0 {
        rslt = -(5 as libc::c_int) as i8;
    }
    return rslt;
}
unsafe fn get_calib_data<I: Interface>(mut dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut coeff_array: [u8; 42] = [0; 42];
    rslt = bme68x_get_regs(
        0x8a as libc::c_int as u8,
        coeff_array.as_mut_ptr(),
        23 as libc::c_int as u32,
        dev,
    );
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0xe1 as libc::c_int as u8,
            &mut *coeff_array.as_mut_ptr().offset(23 as libc::c_int as isize),
            14 as libc::c_int as u32,
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        rslt = bme68x_get_regs(
            0 as libc::c_int as u8,
            &mut *coeff_array
                .as_mut_ptr()
                .offset((23 as libc::c_int + 14 as libc::c_int) as isize),
            5 as libc::c_int as u32,
            dev,
        );
    }
    if rslt as libc::c_int == 0 as libc::c_int {
        (*dev).calib.par_t1 = ((coeff_array[32] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[31] as u16 as libc::c_int) as u16;
        (*dev).calib.par_t2 = ((coeff_array[1] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[0] as u16 as libc::c_int) as i16;
        (*dev).calib.par_t3 = coeff_array[2] as i8;
        (*dev).calib.par_p1 = ((coeff_array[5] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[4] as u16 as libc::c_int) as u16;
        (*dev).calib.par_p2 = ((coeff_array[7] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[6] as u16 as libc::c_int) as i16;
        (*dev).calib.par_p3 = coeff_array[8] as i8;
        (*dev).calib.par_p4 = ((coeff_array[11] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[10] as u16 as libc::c_int) as i16;
        (*dev).calib.par_p5 = ((coeff_array[13] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[12] as u16 as libc::c_int) as i16;
        (*dev).calib.par_p6 = coeff_array[15] as i8;
        (*dev).calib.par_p7 = coeff_array[14] as i8;
        (*dev).calib.par_p8 = ((coeff_array[19] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[18] as u16 as libc::c_int) as i16;
        (*dev).calib.par_p9 = ((coeff_array[21] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[20] as u16 as libc::c_int) as i16;
        (*dev).calib.par_p10 = coeff_array[22];
        (*dev).calib.par_h1 = ((coeff_array[25] as u16 as libc::c_int) << 4 as libc::c_int
            | coeff_array[24] as libc::c_int & 0xf as libc::c_int)
            as u16;
        (*dev).calib.par_h2 = ((coeff_array[23] as u16 as libc::c_int) << 4 as libc::c_int
            | coeff_array[24] as libc::c_int >> 4 as libc::c_int)
            as u16;
        (*dev).calib.par_h3 = coeff_array[26] as i8;
        (*dev).calib.par_h4 = coeff_array[27] as i8;
        (*dev).calib.par_h5 = coeff_array[28] as i8;
        (*dev).calib.par_h6 = coeff_array[29];
        (*dev).calib.par_h7 = coeff_array[30] as i8;
        (*dev).calib.par_gh1 = coeff_array[35] as i8;
        (*dev).calib.par_gh2 = ((coeff_array[34] as u16 as libc::c_int) << 8 as libc::c_int
            | coeff_array[33] as u16 as libc::c_int) as i16;
        (*dev).calib.par_gh3 = coeff_array[36] as i8;
        (*dev).calib.res_heat_range =
            ((coeff_array[39] as libc::c_int & 0x30 as libc::c_int) / 16 as libc::c_int) as u8;
        (*dev).calib.res_heat_val = coeff_array[37] as i8;
        (*dev).calib.range_sw_err = ((coeff_array[41] as libc::c_int & 0xf0 as libc::c_int) as i8
            as libc::c_int
            / 16 as libc::c_int) as i8;
    }
    return rslt;
}

unsafe fn read_variant_id<I: Interface>(mut dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut reg_data: u8 = 0 as libc::c_int as u8;
    rslt = bme68x_get_regs(
        0xf0 as libc::c_int as u8,
        &mut reg_data,
        1 as libc::c_int as u32,
        dev,
    );
    if rslt as libc::c_int == 0 as libc::c_int {
        (*dev).variant_id = reg_data as u32;
    }
    return rslt;
}
