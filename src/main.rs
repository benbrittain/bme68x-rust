mod bme68x;
mod interface;

use bme68x::*;
use interface::{bme68x_interface_init, check_rslt, Error};

use libc::*;

fn main() -> Result<(), Error> {
    let mut bme: bme68x_dev = bme68x_dev::default();

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
        heatr_temp_prof: 0 as *mut u16,
        heatr_dur_prof: 0 as *mut u16,
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
    let mut del_period: u32 = 0;
    let time_ms = std::time::Instant::now();
    let mut n_fields: u8 = 0;
    let mut sample_count: u16 = 1 as libc::c_int as u16;
    let rslt =
        unsafe { bme68x_interface_init(&mut bme as *mut _, BME68X_SPI_INTF as libc::c_int as u8) };
    check_rslt(String::from("bme68x_interface_init"), rslt)?;
    let rslt = unsafe { bme68x_init(&mut bme) };
    check_rslt(String::from("bme68x_init"), rslt)?;
    conf.filter = 0 as libc::c_int as u8;
    conf.odr = 8 as libc::c_int as u8;
    conf.os_hum = 5 as libc::c_int as u8;
    conf.os_pres = 1 as libc::c_int as u8;
    conf.os_temp = 2 as libc::c_int as u8;
    let rslt = unsafe { bme68x_set_conf(&mut conf, &mut bme) };
    check_rslt(String::from("bme68x_set_conf"), rslt)?;
    heatr_conf.enable = 0x1 as libc::c_int as u8;
    heatr_conf.heatr_temp = 300 as libc::c_int as u16;
    heatr_conf.heatr_dur = 100 as libc::c_int as u16;
    let rslt = unsafe { bme68x_set_heatr_conf(1 as libc::c_int as u8, &mut heatr_conf, &mut bme) };
    check_rslt(String::from("bme68x_set_heatr_conf"), rslt)?;
    println!("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
    while sample_count as libc::c_int <= 300 as libc::c_int {
        let rslt = unsafe { bme68x_set_op_mode(1 as libc::c_int as u8, &mut bme) };
        check_rslt(String::from("bme68x_set_op_mode"), rslt)?;
        del_period = unsafe {
            (bme68x_get_meas_dur(1 as libc::c_int as u8, &mut conf, &mut bme)).wrapping_add(
                (heatr_conf.heatr_dur as libc::c_int * 1000 as libc::c_int) as libc::c_uint,
            )
        };
        unsafe {
            (bme.delay_us).expect("non-null function pointer")(del_period, bme.intf_ptr);
        }
        let rslt =
            unsafe { bme68x_get_data(1 as libc::c_int as u8, &mut data, &mut n_fields, &mut bme) };
        check_rslt(String::from("bme68x_get_data"), rslt)?;
        if n_fields != 0 {
            println!(
                "{}, {:?}, {:.2}, {:.2}, {:.2} {:.2} {:x}",
                sample_count as libc::c_int,
                time_ms.elapsed().as_millis(),
                data.temperature as libc::c_double,
                data.pressure as libc::c_double,
                data.humidity as libc::c_double,
                data.gas_resistance as libc::c_double,
                data.status as libc::c_int,
            );

            sample_count = sample_count.wrapping_add(1);
        }
    }

    check_rslt(String::from(""), rslt)
}
