use crate::{Device, GasHeaterConfig, Interface, SensorData};
use core::ffi;

pub(crate) unsafe fn calc_temperature<I: Interface>(
    temp_adc: u32,
    mut dev: *mut Device<I>,
) -> ffi::c_float {
    let mut var1: ffi::c_float = 0.;
    let mut var2: ffi::c_float = 0.;
    let mut calc_temp: ffi::c_float = 0.;
    var1 = (temp_adc as ffi::c_float / 16384.0f32
        - (*dev).calib.par_t1 as ffi::c_float / 1024.0f32)
        * (*dev).calib.par_t2 as ffi::c_float;
    var2 = (temp_adc as ffi::c_float / 131072.0f32
        - (*dev).calib.par_t1 as ffi::c_float / 8192.0f32)
        * (temp_adc as ffi::c_float / 131072.0f32
            - (*dev).calib.par_t1 as ffi::c_float / 8192.0f32)
        * ((*dev).calib.par_t3 as ffi::c_float * 16.0f32);
    (*dev).calib.t_fine = var1 + var2;
    calc_temp = (*dev).calib.t_fine / 5120.0f32;
    return calc_temp;
}
pub(crate) unsafe fn calc_pressure<I: Interface>(
    pres_adc: u32,
    dev: *const Device<I>,
) -> ffi::c_float {
    let mut var1: ffi::c_float = 0.;
    let mut var2: ffi::c_float = 0.;
    let mut var3: ffi::c_float = 0.;
    let mut calc_pres: ffi::c_float = 0.;
    var1 = (*dev).calib.t_fine / 2.0f32 - 64000.0f32;
    var2 = var1 * var1 * ((*dev).calib.par_p6 as ffi::c_float / 131072.0f32);
    var2 = var2 + var1 * (*dev).calib.par_p5 as ffi::c_float * 2.0f32;
    var2 = var2 / 4.0f32 + (*dev).calib.par_p4 as ffi::c_float * 65536.0f32;
    var1 = ((*dev).calib.par_p3 as ffi::c_float * var1 * var1 / 16384.0f32
        + (*dev).calib.par_p2 as ffi::c_float * var1)
        / 524288.0f32;
    var1 = (1.0f32 + var1 / 32768.0f32) * (*dev).calib.par_p1 as ffi::c_float;
    calc_pres = 1048576.0f32 - pres_adc as ffi::c_float;
    if var1 as ffi::c_int != 0 as ffi::c_int {
        calc_pres = (calc_pres - var2 / 4096.0f32) * 6250.0f32 / var1;
        var1 = (*dev).calib.par_p9 as ffi::c_float * calc_pres * calc_pres / 2147483648.0f32;
        var2 = calc_pres * ((*dev).calib.par_p8 as ffi::c_float / 32768.0f32);
        var3 = calc_pres / 256.0f32
            * (calc_pres / 256.0f32)
            * (calc_pres / 256.0f32)
            * ((*dev).calib.par_p10 as ffi::c_int as ffi::c_float / 131072.0f32);
        calc_pres = calc_pres
            + (var1 + var2 + var3 + (*dev).calib.par_p7 as ffi::c_float * 128.0f32) / 16.0f32;
    } else {
        calc_pres = 0 as ffi::c_int as ffi::c_float;
    }
    return calc_pres;
}
pub(crate) unsafe fn calc_humidity<I: Interface>(
    hum_adc: u16,
    dev: *const Device<I>,
) -> ffi::c_float {
    let mut calc_hum: ffi::c_float = 0.;
    let mut var1: ffi::c_float = 0.;
    let mut var2: ffi::c_float = 0.;
    let mut var3: ffi::c_float = 0.;
    let mut var4: ffi::c_float = 0.;
    let mut temp_comp: ffi::c_float = 0.;
    temp_comp = (*dev).calib.t_fine / 5120.0f32;
    var1 = hum_adc as ffi::c_float
        - ((*dev).calib.par_h1 as ffi::c_float * 16.0f32
            + (*dev).calib.par_h3 as ffi::c_float / 2.0f32 * temp_comp);
    var2 = var1
        * ((*dev).calib.par_h2 as ffi::c_float / 262144.0f32
            * (1.0f32
                + (*dev).calib.par_h4 as ffi::c_float / 16384.0f32 * temp_comp
                + (*dev).calib.par_h5 as ffi::c_float / 1048576.0f32 * temp_comp * temp_comp));
    var3 = (*dev).calib.par_h6 as ffi::c_float / 16384.0f32;
    var4 = (*dev).calib.par_h7 as ffi::c_float / 2097152.0f32;
    calc_hum = var2 + (var3 + var4 * temp_comp) * var2 * var2;
    if calc_hum > 100.0f32 {
        calc_hum = 100.0f32;
    } else if calc_hum < 0.0f32 {
        calc_hum = 0.0f32;
    }
    return calc_hum;
}
pub(crate) unsafe fn calc_gas_resistance_low<I: Interface>(
    gas_res_adc: u16,
    gas_range: u8,
    dev: *const Device<I>,
) -> ffi::c_float {
    let mut calc_gas_res: ffi::c_float = 0.;
    let mut var1: ffi::c_float = 0.;
    let mut var2: ffi::c_float = 0.;
    let mut var3: ffi::c_float = 0.;
    let gas_res_f: ffi::c_float = gas_res_adc as ffi::c_float;
    let gas_range_f: ffi::c_float = ((1 as ffi::c_uint) << gas_range as ffi::c_int) as ffi::c_float;
    let lookup_k1_range: [ffi::c_float; 16] = [
        0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32, -1.0f32, 0.0f32, -0.8f32, 0.0f32, 0.0f32, -0.2f32,
        -0.5f32, 0.0f32, -1.0f32, 0.0f32, 0.0f32,
    ];
    let lookup_k2_range: [ffi::c_float; 16] = [
        0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.1f32, 0.7f32, 0.0f32, -0.8f32, -0.1f32, 0.0f32, 0.0f32,
        0.0f32, 0.0f32, 0.0f32, 0.0f32, 0.0f32,
    ];
    var1 = 1340.0f32 + 5.0f32 * (*dev).calib.range_sw_err as ffi::c_int as ffi::c_float;
    var2 = var1 * (1.0f32 + lookup_k1_range[gas_range as usize] / 100.0f32);
    var3 = 1.0f32 + lookup_k2_range[gas_range as usize] / 100.0f32;
    calc_gas_res =
        1.0f32 / (var3 * 0.000000125f32 * gas_range_f * ((gas_res_f - 512.0f32) / var2 + 1.0f32));
    return calc_gas_res;
}
pub(crate) unsafe fn calc_gas_resistance_high(gas_res_adc: u16, gas_range: u8) -> ffi::c_float {
    let mut calc_gas_res: ffi::c_float = 0.;
    let var1: u32 = 262144 as ffi::c_uint >> gas_range as ffi::c_int;
    let mut var2: i32 = gas_res_adc as i32 - 512 as ffi::c_int;
    var2 *= 3 as ffi::c_int;
    var2 = 4096 as ffi::c_int + var2;
    calc_gas_res = 1000000.0f32 * var1 as ffi::c_float / var2 as ffi::c_float;
    return calc_gas_res;
}
pub(crate) unsafe fn calc_res_heat<I: Interface>(mut temp: u16, dev: *const Device<I>) -> u8 {
    let mut var1: ffi::c_float = 0.;
    let mut var2: ffi::c_float = 0.;
    let mut var3: ffi::c_float = 0.;
    let mut var4: ffi::c_float = 0.;
    let mut var5: ffi::c_float = 0.;
    let mut res_heat: u8 = 0;
    if temp as ffi::c_int > 400 as ffi::c_int {
        temp = 400 as ffi::c_int as u16;
    }
    var1 = (*dev).calib.par_gh1 as ffi::c_float / 16.0f32 + 49.0f32;
    var2 = (*dev).calib.par_gh2 as ffi::c_float / 32768.0f32 * 0.0005f32 + 0.00235f32;
    var3 = (*dev).calib.par_gh3 as ffi::c_float / 1024.0f32;
    var4 = var1 * (1.0f32 + var2 * temp as ffi::c_float);
    var5 = var4 + var3 * (*dev).amb_temp as ffi::c_float;
    res_heat = (3.4f32
        * (var5
            * (4 as ffi::c_int as ffi::c_float
                / (4 as ffi::c_int as ffi::c_float + (*dev).calib.res_heat_range as ffi::c_float))
            * (1 as ffi::c_int as ffi::c_float
                / (1 as ffi::c_int as ffi::c_float
                    + (*dev).calib.res_heat_val as ffi::c_float * 0.002f32))
            - 25 as ffi::c_int as ffi::c_float)) as u8;
    return res_heat;
}
pub(crate) unsafe fn calc_gas_wait(mut dur: u16) -> u8 {
    let mut factor: u8 = 0 as ffi::c_int as u8;
    let mut durval: u8 = 0;
    if dur as ffi::c_int >= 0xfc0 as ffi::c_int {
        durval = 0xff as ffi::c_int as u8;
    } else {
        while dur as ffi::c_int > 0x3f as ffi::c_int {
            dur = (dur as ffi::c_int / 4 as ffi::c_int) as u16;
            factor = (factor as ffi::c_int + 1 as ffi::c_int) as u8;
        }
        durval = (dur as ffi::c_int + factor as ffi::c_int * 64 as ffi::c_int) as u8;
    }
    return durval;
}
pub(crate) unsafe fn read_field_data<I: Interface>(
    index: u8,
    mut data: *mut SensorData,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0 as ffi::c_int as i8;
    let mut buff: [u8; 17] = [
        0 as ffi::c_int as u8,
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
    let mut tries: u8 = 5 as ffi::c_int as u8;
    while tries as ffi::c_int != 0 && rslt as ffi::c_int == 0 as ffi::c_int {
        (*dev)
            .get_regs(
                (0x1d as ffi::c_int + index as ffi::c_int * 17 as ffi::c_int) as u8,
                buff.as_mut_ptr(),
                17 as ffi::c_int as u16 as u32,
            )
            .unwrap();
        if data.is_null() {
            rslt = -(1 as ffi::c_int) as i8;
            break;
        } else {
            (*data).status = (buff[0] as ffi::c_int & 0x80 as ffi::c_int) as u8;
            (*data).gas_index = (buff[0] as ffi::c_int & 0xf as ffi::c_int) as u8;
            (*data).meas_index = buff[1];
            adc_pres = (buff[2] as u32).wrapping_mul(4096 as ffi::c_int as ffi::c_uint)
                | (buff[3] as u32).wrapping_mul(16 as ffi::c_int as ffi::c_uint)
                | (buff[4] as u32).wrapping_div(16 as ffi::c_int as ffi::c_uint);
            adc_temp = (buff[5] as u32).wrapping_mul(4096 as ffi::c_int as ffi::c_uint)
                | (buff[6] as u32).wrapping_mul(16 as ffi::c_int as ffi::c_uint)
                | (buff[7] as u32).wrapping_div(16 as ffi::c_int as ffi::c_uint);
            adc_hum = ((buff[8] as u32).wrapping_mul(256 as ffi::c_int as ffi::c_uint)
                | buff[9] as u32) as u16;
            adc_gas_res_low = ((buff[13] as u32).wrapping_mul(4 as ffi::c_int as ffi::c_uint)
                | (buff[14] as u32).wrapping_div(64 as ffi::c_int as ffi::c_uint))
                as u16;
            adc_gas_res_high = ((buff[15] as u32).wrapping_mul(4 as ffi::c_int as ffi::c_uint)
                | (buff[16] as u32).wrapping_div(64 as ffi::c_int as ffi::c_uint))
                as u16;
            gas_range_l = (buff[14] as ffi::c_int & 0xf as ffi::c_int) as u8;
            gas_range_h = (buff[16] as ffi::c_int & 0xf as ffi::c_int) as u8;
            if (*dev).variant_id == 0x1 as ffi::c_int as ffi::c_uint {
                let ref mut fresh0 = (*data).status;
                *fresh0 =
                    (*fresh0 as ffi::c_int | buff[16] as ffi::c_int & 0x20 as ffi::c_int) as u8;
                let ref mut fresh1 = (*data).status;
                *fresh1 =
                    (*fresh1 as ffi::c_int | buff[16] as ffi::c_int & 0x10 as ffi::c_int) as u8;
            } else {
                let ref mut fresh2 = (*data).status;
                *fresh2 =
                    (*fresh2 as ffi::c_int | buff[14] as ffi::c_int & 0x20 as ffi::c_int) as u8;
                let ref mut fresh3 = (*data).status;
                *fresh3 =
                    (*fresh3 as ffi::c_int | buff[14] as ffi::c_int & 0x10 as ffi::c_int) as u8;
            }
            if (*data).status as ffi::c_int & 0x80 as ffi::c_int != 0
                && rslt as ffi::c_int == 0 as ffi::c_int
            {
                (*dev)
                    .get_regs(
                        (0x5a as ffi::c_int + (*data).gas_index as ffi::c_int) as u8,
                        &mut (*data).res_heat,
                        1 as ffi::c_int as u32,
                    )
                    .unwrap();
                if rslt as ffi::c_int == 0 as ffi::c_int {
                    (*dev)
                        .get_regs(
                            (0x50 as ffi::c_int + (*data).gas_index as ffi::c_int) as u8,
                            &mut (*data).idac,
                            1 as ffi::c_int as u32,
                        )
                        .unwrap();
                }
                if rslt as ffi::c_int == 0 as ffi::c_int {
                    (*dev)
                        .get_regs(
                            (0x64 as ffi::c_int + (*data).gas_index as ffi::c_int) as u8,
                            &mut (*data).gas_wait,
                            1 as ffi::c_int as u32,
                        )
                        .unwrap();
                }
                if rslt as ffi::c_int == 0 as ffi::c_int {
                    (*data).temperature = calc_temperature(adc_temp, dev);
                    (*data).pressure = calc_pressure(adc_pres, dev);
                    (*data).humidity = calc_humidity(adc_hum, dev);
                    if (*dev).variant_id == 0x1 as ffi::c_int as ffi::c_uint {
                        (*data).gas_resistance =
                            calc_gas_resistance_high(adc_gas_res_high, gas_range_h);
                    } else {
                        (*data).gas_resistance =
                            calc_gas_resistance_low(adc_gas_res_low, gas_range_l, dev);
                    }
                    break;
                }
            }
            if rslt as ffi::c_int == 0 as ffi::c_int {
                (*dev).interface.delay(10000 as ffi::c_uint);
            }
            tries = tries.wrapping_sub(1);
        }
    }
    return rslt;
}
pub(crate) unsafe fn read_all_field_data<I: Interface>(
    data: *const *mut SensorData,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0 as ffi::c_int as i8;
    let mut buff: [u8; 51] = [
        0 as ffi::c_int as u8,
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
        0 as ffi::c_int as u8,
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
    if (*data.offset(0 as ffi::c_int as isize)).is_null()
        && (*data.offset(1 as ffi::c_int as isize)).is_null()
        && (*data.offset(2 as ffi::c_int as isize)).is_null()
    {
        rslt = -(1 as ffi::c_int) as i8;
    }
    if rslt as ffi::c_int == 0 as ffi::c_int {
        (*dev)
            .get_regs(
                0x1d as ffi::c_int as u8,
                buff.as_mut_ptr(),
                (17 as ffi::c_int as u32).wrapping_mul(3 as ffi::c_int as ffi::c_uint),
            )
            .unwrap();
    }
    if rslt as ffi::c_int == 0 as ffi::c_int {
        (*dev)
            .get_regs(
                0x50 as ffi::c_int as u8,
                set_val.as_mut_ptr(),
                30 as ffi::c_int as u32,
            )
            .unwrap();
    }
    i = 0 as ffi::c_int as u8;
    while (i as ffi::c_int) < 3 as ffi::c_int && rslt as ffi::c_int == 0 as ffi::c_int {
        off = (i as ffi::c_int * 17 as ffi::c_int) as u8;
        (**data.offset(i as isize)).status =
            (buff[off as usize] as ffi::c_int & 0x80 as ffi::c_int) as u8;
        (**data.offset(i as isize)).gas_index =
            (buff[off as usize] as ffi::c_int & 0xf as ffi::c_int) as u8;
        (**data.offset(i as isize)).meas_index =
            buff[(off as ffi::c_int + 1 as ffi::c_int) as usize];
        adc_pres = (buff[(off as ffi::c_int + 2 as ffi::c_int) as usize] as u32)
            .wrapping_mul(4096 as ffi::c_int as ffi::c_uint)
            | (buff[(off as ffi::c_int + 3 as ffi::c_int) as usize] as u32)
                .wrapping_mul(16 as ffi::c_int as ffi::c_uint)
            | (buff[(off as ffi::c_int + 4 as ffi::c_int) as usize] as u32)
                .wrapping_div(16 as ffi::c_int as ffi::c_uint);
        adc_temp = (buff[(off as ffi::c_int + 5 as ffi::c_int) as usize] as u32)
            .wrapping_mul(4096 as ffi::c_int as ffi::c_uint)
            | (buff[(off as ffi::c_int + 6 as ffi::c_int) as usize] as u32)
                .wrapping_mul(16 as ffi::c_int as ffi::c_uint)
            | (buff[(off as ffi::c_int + 7 as ffi::c_int) as usize] as u32)
                .wrapping_div(16 as ffi::c_int as ffi::c_uint);
        adc_hum = ((buff[(off as ffi::c_int + 8 as ffi::c_int) as usize] as u32)
            .wrapping_mul(256 as ffi::c_int as ffi::c_uint)
            | buff[(off as ffi::c_int + 9 as ffi::c_int) as usize] as u32) as u16;
        adc_gas_res_low = ((buff[(off as ffi::c_int + 13 as ffi::c_int) as usize] as u32)
            .wrapping_mul(4 as ffi::c_int as ffi::c_uint)
            | (buff[(off as ffi::c_int + 14 as ffi::c_int) as usize] as u32)
                .wrapping_div(64 as ffi::c_int as ffi::c_uint)) as u16;
        adc_gas_res_high = ((buff[(off as ffi::c_int + 15 as ffi::c_int) as usize] as u32)
            .wrapping_mul(4 as ffi::c_int as ffi::c_uint)
            | (buff[(off as ffi::c_int + 16 as ffi::c_int) as usize] as u32)
                .wrapping_div(64 as ffi::c_int as ffi::c_uint)) as u16;
        gas_range_l = (buff[(off as ffi::c_int + 14 as ffi::c_int) as usize] as ffi::c_int
            & 0xf as ffi::c_int) as u8;
        gas_range_h = (buff[(off as ffi::c_int + 16 as ffi::c_int) as usize] as ffi::c_int
            & 0xf as ffi::c_int) as u8;
        if (*dev).variant_id == 0x1 as ffi::c_int as ffi::c_uint {
            let ref mut fresh4 = (**data.offset(i as isize)).status;
            *fresh4 = (*fresh4 as ffi::c_int
                | buff[(off as ffi::c_int + 16 as ffi::c_int) as usize] as ffi::c_int
                    & 0x20 as ffi::c_int) as u8;
            let ref mut fresh5 = (**data.offset(i as isize)).status;
            *fresh5 = (*fresh5 as ffi::c_int
                | buff[(off as ffi::c_int + 16 as ffi::c_int) as usize] as ffi::c_int
                    & 0x10 as ffi::c_int) as u8;
        } else {
            let ref mut fresh6 = (**data.offset(i as isize)).status;
            *fresh6 = (*fresh6 as ffi::c_int
                | buff[(off as ffi::c_int + 14 as ffi::c_int) as usize] as ffi::c_int
                    & 0x20 as ffi::c_int) as u8;
            let ref mut fresh7 = (**data.offset(i as isize)).status;
            *fresh7 = (*fresh7 as ffi::c_int
                | buff[(off as ffi::c_int + 14 as ffi::c_int) as usize] as ffi::c_int
                    & 0x10 as ffi::c_int) as u8;
        }
        (**data.offset(i as isize)).idac = set_val[(**data.offset(i as isize)).gas_index as usize];
        (**data.offset(i as isize)).res_heat = set_val
            [(10 as ffi::c_int + (**data.offset(i as isize)).gas_index as ffi::c_int) as usize];
        (**data.offset(i as isize)).gas_wait = set_val
            [(20 as ffi::c_int + (**data.offset(i as isize)).gas_index as ffi::c_int) as usize];
        (**data.offset(i as isize)).temperature = calc_temperature(adc_temp, dev);
        (**data.offset(i as isize)).pressure = calc_pressure(adc_pres, dev);
        (**data.offset(i as isize)).humidity = calc_humidity(adc_hum, dev);
        if (*dev).variant_id == 0x1 as ffi::c_int as ffi::c_uint {
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
pub(crate) unsafe fn set_mem_page<I: Interface>(reg_addr: u8, mut dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut reg: u8 = 0;
    let mut mem_page: u8 = 0;
    rslt = null_ptr_check(dev);
    if rslt as ffi::c_int == 0 as ffi::c_int {
        if reg_addr as ffi::c_int > 0x7f as ffi::c_int {
            mem_page = 0 as ffi::c_int as u8;
        } else {
            mem_page = 0x10 as ffi::c_int as u8;
        }
        if mem_page as ffi::c_int != (*dev).mem_page as ffi::c_int {
            (*dev).mem_page = mem_page;
            (*dev).interface_result = (*dev).interface.read_raw(
                (0xf3 as ffi::c_int | 0x80 as ffi::c_int) as u8,
                &mut reg,
                1 as ffi::c_int as u32,
            );
            if (*dev).interface_result as ffi::c_int != 0 as ffi::c_int {
                rslt = -(2 as ffi::c_int) as i8;
            }
            if rslt as ffi::c_int == 0 as ffi::c_int {
                reg = (reg as ffi::c_int & !(0x10 as ffi::c_int)) as u8;
                reg =
                    (reg as ffi::c_int | (*dev).mem_page as ffi::c_int & 0x10 as ffi::c_int) as u8;
                (*dev).interface_result = (*dev).interface.write_raw(
                    (0xf3 as ffi::c_int & 0x7f as ffi::c_int) as u8,
                    &mut reg,
                    1 as ffi::c_int as u32,
                );
                if (*dev).interface_result as ffi::c_int != 0 as ffi::c_int {
                    rslt = -(2 as ffi::c_int) as i8;
                }
            }
        }
    }
    return rslt;
}
pub(crate) unsafe fn get_mem_page<I: Interface>(mut dev: *mut Device<I>) -> i8 {
    let mut rslt: i8 = 0;
    let mut reg: u8 = 0;
    rslt = null_ptr_check(dev);
    if rslt as ffi::c_int == 0 as ffi::c_int {
        (*dev).interface_result = (*dev).interface.read_raw(
            (0xf3 as ffi::c_int | 0x80 as ffi::c_int) as u8,
            &mut reg,
            1 as ffi::c_int as u32,
        );
        if (*dev).interface_result as ffi::c_int != 0 as ffi::c_int {
            rslt = -(2 as ffi::c_int) as i8;
        } else {
            (*dev).mem_page = (reg as ffi::c_int & 0x10 as ffi::c_int) as u8;
        }
    }
    return rslt;
}
pub(crate) unsafe fn boundary_check<I: Interface>(
    value: *mut u8,
    max: u8,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0;
    rslt = null_ptr_check(dev);
    if !value.is_null() && rslt as ffi::c_int == 0 as ffi::c_int {
        if *value as ffi::c_int > max as ffi::c_int {
            *value = max;
            let ref mut fresh8 = (*dev).info_msg;
            *fresh8 = (*fresh8 as ffi::c_int | 1 as ffi::c_int) as u8;
        }
    } else {
        rslt = -(1 as ffi::c_int) as i8;
    }
    return rslt;
}
pub(crate) unsafe fn null_ptr_check<I: Interface>(dev: *const Device<I>) -> i8 {
    let mut rslt: i8 = 0 as ffi::c_int as i8;
    if dev.is_null() {
        rslt = -(1 as ffi::c_int) as i8;
    }
    return rslt;
}
pub(crate) unsafe fn set_conf<I: Interface>(
    conf: *const GasHeaterConfig,
    op_mode: u8,
    nb_conv: *mut u8,
    dev: *mut Device<I>,
) -> i8 {
    let mut rslt: i8 = 0 as ffi::c_int as i8;
    let mut i: u8 = 0;
    let mut shared_dur: u8 = 0;
    let mut write_len: u8 = 0 as ffi::c_int as u8;
    let mut heater_dur_shared_addr: u8 = 0x6e as ffi::c_int as u8;
    let mut rh_reg_addr: [u8; 10] = [
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
    ];
    let mut rh_reg_data: [u8; 10] = [
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
    ];
    let mut gw_reg_addr: [u8; 10] = [
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
    ];
    let mut gw_reg_data: [u8; 10] = [
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
        0 as ffi::c_int as u8,
    ];
    match op_mode as ffi::c_int {
        1 => {
            rh_reg_addr[0] = 0x5a as ffi::c_int as u8;
            rh_reg_data[0] = calc_res_heat((*conf).heatr_temp, dev);
            gw_reg_addr[0] = 0x64 as ffi::c_int as u8;
            gw_reg_data[0] = calc_gas_wait((*conf).heatr_dur);
            *nb_conv = 0 as ffi::c_int as u8;
            write_len = 1 as ffi::c_int as u8;
        }
        3 => {
            if ((*conf).heatr_dur_prof).is_null() || ((*conf).heatr_temp_prof).is_null() {
                rslt = -(1 as ffi::c_int) as i8;
            } else {
                i = 0 as ffi::c_int as u8;
                while (i as ffi::c_int) < (*conf).profile_len as ffi::c_int {
                    rh_reg_addr[i as usize] = (0x5a as ffi::c_int + i as ffi::c_int) as u8;
                    rh_reg_data[i as usize] =
                        calc_res_heat(*((*conf).heatr_temp_prof).offset(i as isize), dev);
                    gw_reg_addr[i as usize] = (0x64 as ffi::c_int + i as ffi::c_int) as u8;
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
                rslt = -(1 as ffi::c_int) as i8;
            } else {
                if (*conf).shared_heatr_dur as ffi::c_int == 0 as ffi::c_int {
                    rslt = 3 as ffi::c_int as i8;
                }
                i = 0 as ffi::c_int as u8;
                while (i as ffi::c_int) < (*conf).profile_len as ffi::c_int {
                    rh_reg_addr[i as usize] = (0x5a as ffi::c_int + i as ffi::c_int) as u8;
                    rh_reg_data[i as usize] =
                        calc_res_heat(*((*conf).heatr_temp_prof).offset(i as isize), dev);
                    gw_reg_addr[i as usize] = (0x64 as ffi::c_int + i as ffi::c_int) as u8;
                    gw_reg_data[i as usize] = *((*conf).heatr_dur_prof).offset(i as isize) as u8;
                    i = i.wrapping_add(1);
                }
                *nb_conv = (*conf).profile_len;
                write_len = (*conf).profile_len;
                shared_dur = calc_heatr_dur_shared((*conf).shared_heatr_dur);
                if rslt as ffi::c_int == 0 as ffi::c_int {
                    (*dev)
                        .set_regs(
                            &mut heater_dur_shared_addr,
                            &mut shared_dur,
                            1 as ffi::c_int as u32,
                        )
                        .unwrap();
                }
            }
        }
        _ => {
            rslt = 1 as ffi::c_int as i8;
        }
    }
    if rslt as ffi::c_int == 0 as ffi::c_int {
        (*dev)
            .set_regs(
                rh_reg_addr.as_mut_ptr(),
                rh_reg_data.as_mut_ptr(),
                write_len as u32,
            )
            .unwrap();
    }
    if rslt as ffi::c_int == 0 as ffi::c_int {
        (*dev)
            .set_regs(
                gw_reg_addr.as_mut_ptr(),
                gw_reg_data.as_mut_ptr(),
                write_len as u32,
            )
            .unwrap();
    }
    return rslt;
}
pub(crate) unsafe fn calc_heatr_dur_shared(mut dur: u16) -> u8 {
    let mut factor: u8 = 0 as ffi::c_int as u8;
    let mut heatdurval: u8 = 0;
    if dur as ffi::c_int >= 0x783 as ffi::c_int {
        heatdurval = 0xff as ffi::c_int as u8;
    } else {
        dur = (dur as u32)
            .wrapping_mul(1000 as ffi::c_int as ffi::c_uint)
            .wrapping_div(477 as ffi::c_int as ffi::c_uint) as u16;
        while dur as ffi::c_int > 0x3f as ffi::c_int {
            dur = (dur as ffi::c_int >> 2 as ffi::c_int) as u16;
            factor = (factor as ffi::c_int + 1 as ffi::c_int) as u8;
        }
        heatdurval = (dur as ffi::c_int + factor as ffi::c_int * 64 as ffi::c_int) as u8;
    }
    return heatdurval;
}
pub(crate) unsafe fn sort_sensor_data(low_index: u8, high_index: u8, field: *mut *mut SensorData) {
    let mut meas_index1: i16 = 0;
    let mut meas_index2: i16 = 0;
    meas_index1 = (**field.offset(low_index as isize)).meas_index as i16;
    meas_index2 = (**field.offset(high_index as isize)).meas_index as i16;
    if (**field.offset(low_index as isize)).status as ffi::c_int & 0x80 as ffi::c_int != 0
        && (**field.offset(high_index as isize)).status as ffi::c_int & 0x80 as ffi::c_int != 0
    {
        let diff: i16 = (meas_index2 as ffi::c_int - meas_index1 as ffi::c_int) as i16;
        if diff as ffi::c_int > -(3 as ffi::c_int) && (diff as ffi::c_int) < 0 as ffi::c_int
            || diff as ffi::c_int > 2 as ffi::c_int
        {
            swap_fields(low_index, high_index, field);
        }
    } else if (**field.offset(high_index as isize)).status as ffi::c_int & 0x80 as ffi::c_int != 0 {
        swap_fields(low_index, high_index, field);
    }
}
pub(crate) unsafe fn swap_fields(index1: u8, index2: u8, field: *mut *mut SensorData) {
    let mut temp: *mut SensorData = 0 as *mut SensorData;
    temp = *field.offset(index1 as isize);
    let ref mut fresh9 = *field.offset(index1 as isize);
    *fresh9 = *field.offset(index2 as isize);
    let ref mut fresh10 = *field.offset(index2 as isize);
    *fresh10 = temp;
}
pub(crate) unsafe fn analyze_sensor_data(data: *const SensorData, n_meas: u8) -> i8 {
    let mut rslt: i8 = 0 as ffi::c_int as i8;
    let mut self_test_failed: u8 = 0 as ffi::c_int as u8;
    let mut i: u8 = 0;
    let mut cent_res: u32 = 0 as ffi::c_int as u32;
    if (*data.offset(0 as ffi::c_int as isize)).temperature < 0 as ffi::c_int as ffi::c_float
        || (*data.offset(0 as ffi::c_int as isize)).temperature > 60 as ffi::c_int as ffi::c_float
    {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    if (*data.offset(0 as ffi::c_int as isize)).pressure < 90000 as ffi::c_uint as ffi::c_float
        || (*data.offset(0 as ffi::c_int as isize)).pressure > 110000 as ffi::c_uint as ffi::c_float
    {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    if (*data.offset(0 as ffi::c_int as isize)).humidity < 20 as ffi::c_uint as ffi::c_float
        || (*data.offset(0 as ffi::c_int as isize)).humidity > 80 as ffi::c_uint as ffi::c_float
    {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    i = 0 as ffi::c_int as u8;
    while (i as ffi::c_int) < n_meas as ffi::c_int {
        if (*data.offset(i as isize)).status as ffi::c_int & 0x20 as ffi::c_int == 0 {
            self_test_failed = self_test_failed.wrapping_add(1);
        }
        i = i.wrapping_add(1);
    }
    if n_meas as ffi::c_int >= 6 as ffi::c_int {
        cent_res = (5 as ffi::c_int as ffi::c_float
            * ((*data.offset(3 as ffi::c_int as isize)).gas_resistance
                + (*data.offset(5 as ffi::c_int as isize)).gas_resistance)
            / (2 as ffi::c_int as ffi::c_float
                * (*data.offset(4 as ffi::c_int as isize)).gas_resistance))
            as u32;
    }
    if cent_res < 6 as ffi::c_int as ffi::c_uint {
        self_test_failed = self_test_failed.wrapping_add(1);
    }
    if self_test_failed != 0 {
        rslt = -(5 as ffi::c_int) as i8;
    }
    return rslt;
}
pub(crate) fn get_calib_data<I: Interface>(mut dev: &mut Device<I>) -> i8 {
    let mut coeff_array: [u8; 42] = [0; 42];
    unsafe {
        dev.get_regs(
            0x8a as ffi::c_int as u8,
            coeff_array.as_mut_ptr(),
            23 as ffi::c_int as u32,
        )
        .unwrap();
        dev.get_regs(
            0xe1 as ffi::c_int as u8,
            &mut *coeff_array.as_mut_ptr().offset(23 as ffi::c_int as isize),
            14 as ffi::c_int as u32,
        )
        .unwrap();
        dev.get_regs(
            0 as ffi::c_int as u8,
            &mut *coeff_array
                .as_mut_ptr()
                .offset((23 as ffi::c_int + 14 as ffi::c_int) as isize),
            5 as ffi::c_int as u32,
        )
        .unwrap();
    }
    dev.calib.par_t1 = ((coeff_array[32] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[31] as u16 as ffi::c_int) as u16;
    dev.calib.par_t2 = ((coeff_array[1] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[0] as u16 as ffi::c_int) as i16;
    dev.calib.par_t3 = coeff_array[2] as i8;
    dev.calib.par_p1 = ((coeff_array[5] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[4] as u16 as ffi::c_int) as u16;
    dev.calib.par_p2 = ((coeff_array[7] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[6] as u16 as ffi::c_int) as i16;
    dev.calib.par_p3 = coeff_array[8] as i8;
    dev.calib.par_p4 = ((coeff_array[11] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[10] as u16 as ffi::c_int) as i16;
    dev.calib.par_p5 = ((coeff_array[13] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[12] as u16 as ffi::c_int) as i16;
    dev.calib.par_p6 = coeff_array[15] as i8;
    dev.calib.par_p7 = coeff_array[14] as i8;
    dev.calib.par_p8 = ((coeff_array[19] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[18] as u16 as ffi::c_int) as i16;
    dev.calib.par_p9 = ((coeff_array[21] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[20] as u16 as ffi::c_int) as i16;
    dev.calib.par_p10 = coeff_array[22];
    dev.calib.par_h1 = ((coeff_array[25] as u16 as ffi::c_int) << 4 as ffi::c_int
        | coeff_array[24] as ffi::c_int & 0xf as ffi::c_int) as u16;
    dev.calib.par_h2 = ((coeff_array[23] as u16 as ffi::c_int) << 4 as ffi::c_int
        | coeff_array[24] as ffi::c_int >> 4 as ffi::c_int) as u16;
    dev.calib.par_h3 = coeff_array[26] as i8;
    dev.calib.par_h4 = coeff_array[27] as i8;
    dev.calib.par_h5 = coeff_array[28] as i8;
    dev.calib.par_h6 = coeff_array[29];
    dev.calib.par_h7 = coeff_array[30] as i8;
    dev.calib.par_gh1 = coeff_array[35] as i8;
    dev.calib.par_gh2 = ((coeff_array[34] as u16 as ffi::c_int) << 8 as ffi::c_int
        | coeff_array[33] as u16 as ffi::c_int) as i16;
    dev.calib.par_gh3 = coeff_array[36] as i8;
    dev.calib.res_heat_range =
        ((coeff_array[39] as ffi::c_int & 0x30 as ffi::c_int) / 16 as ffi::c_int) as u8;
    dev.calib.res_heat_val = coeff_array[37] as i8;
    dev.calib.range_sw_err = ((coeff_array[41] as ffi::c_int & 0xf0 as ffi::c_int) as i8
        as ffi::c_int
        / 16 as ffi::c_int) as i8;
    return 0;
}

pub(crate) fn read_variant_id<I: Interface>(mut dev: &mut Device<I>) -> i8 {
    let mut reg_data: u8 = 0 as ffi::c_int as u8;
    dev.get_regs(
        0xf0 as ffi::c_int as u8,
        &mut reg_data,
        1 as ffi::c_int as u32,
    )
    .unwrap();
    dev.variant_id = reg_data as u32;
    return 0;
}
