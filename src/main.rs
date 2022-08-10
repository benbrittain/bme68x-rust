mod bme68x;
mod common;

use bme68x::*;
use common::bme68x_check_rslt;
use common::bme68x_interface_init;

use libc::*;

unsafe fn main_0() -> libc::c_int {
    let mut bme: bme68x_dev = bme68x_dev {
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
    let mut rslt: int8_t = 0;
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
    let mut data: bme68x_data = bme68x_data {
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
    };
    let mut del_period: uint32_t = 0;
    let mut time_ms: uint32_t = 0 as libc::c_int as uint32_t;
    let mut n_fields: uint8_t = 0;
    let mut sample_count: uint16_t = 1 as libc::c_int as uint16_t;
    rslt = bme68x_interface_init(
        &mut bme as *mut _,
        BME68X_SPI_INTF as libc::c_int as uint8_t,
    );
    bme68x_check_rslt(
        b"bme68x_interface_init\0" as *const u8 as *const libc::c_char,
        rslt,
    );
    rslt = bme68x_init(&mut bme);
    bme68x_check_rslt(b"bme68x_init\0" as *const u8 as *const libc::c_char, rslt);
    conf.filter = 0 as libc::c_int as uint8_t;
    conf.odr = 8 as libc::c_int as uint8_t;
    conf.os_hum = 5 as libc::c_int as uint8_t;
    conf.os_pres = 1 as libc::c_int as uint8_t;
    conf.os_temp = 2 as libc::c_int as uint8_t;
    rslt = bme68x_set_conf(&mut conf, &mut bme);
    bme68x_check_rslt(
        b"bme68x_set_conf\0" as *const u8 as *const libc::c_char,
        rslt,
    );
    heatr_conf.enable = 0x1 as libc::c_int as uint8_t;
    heatr_conf.heatr_temp = 300 as libc::c_int as uint16_t;
    heatr_conf.heatr_dur = 100 as libc::c_int as uint16_t;
    rslt = bme68x_set_heatr_conf(1 as libc::c_int as uint8_t, &mut heatr_conf, &mut bme);
    bme68x_check_rslt(
        b"bme68x_set_heatr_conf\0" as *const u8 as *const libc::c_char,
        rslt,
    );
    println!("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
    while sample_count as libc::c_int <= 300 as libc::c_int {
        rslt = bme68x_set_op_mode(1 as libc::c_int as uint8_t, &mut bme);
        bme68x_check_rslt(
            b"bme68x_set_op_mode\0" as *const u8 as *const libc::c_char,
            rslt,
        );
        del_period = (bme68x_get_meas_dur(1 as libc::c_int as uint8_t, &mut conf, &mut bme))
            .wrapping_add(
                (heatr_conf.heatr_dur as libc::c_int * 1000 as libc::c_int) as libc::c_uint,
            );
        (bme.delay_us).expect("non-null function pointer")(del_period, bme.intf_ptr);
        rslt = bme68x_get_data(
            1 as libc::c_int as uint8_t,
            &mut data,
            &mut n_fields,
            &mut bme,
        );
        bme68x_check_rslt(
            b"bme68x_get_data\0" as *const u8 as *const libc::c_char,
            rslt,
        );
        if n_fields != 0 {
            println!(
                //b"%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x\n\0" as *const u8
                "{}, {} ,{}, {}, {}, {}, {}",
                sample_count as libc::c_int,
                time_ms as libc::c_ulong,
                data.temperature as libc::c_double,
                data.pressure as libc::c_double,
                data.humidity as libc::c_double,
                data.gas_resistance as libc::c_double,
                data.status as libc::c_int,
            );
            sample_count = sample_count.wrapping_add(1);
        }
    }
    return rslt as libc::c_int;
}
pub fn main() {
    unsafe { ::std::process::exit(main_0() as i32) }
}
