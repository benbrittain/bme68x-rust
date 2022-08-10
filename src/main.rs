mod bme68x;
mod interface;

use bme68x::{Device, *};
use interface::{bme68x_interface_init, check_rslt, Error};

fn main() -> Result<(), Error> {
    // Init interface
    let mut bme: Device = Device::default();
    let rslt =
        unsafe { bme68x_interface_init(&mut bme as *mut _, BME68X_SPI_INTF as libc::c_int as u8) };
    check_rslt(String::from("bme68x_interface_init"), rslt)?;

    // Init bme68x
    let rslt = unsafe { bme68x_init(&mut bme) };
    check_rslt(String::from("bme68x_init"), rslt)?;

    // configure device
    let mut conf: DeviceConf = DeviceConf::default();
    conf.filter = 0;
    conf.odr = 8;
    conf.os_hum = 5;
    conf.os_pres = 1;
    conf.os_temp = 2;
    let rslt = unsafe { bme68x_set_conf(&mut conf, &mut bme) };
    check_rslt(String::from("bme68x_set_conf"), rslt)?;

    // configure heater
    let mut heatr_conf: HeaterConf = HeaterConf::default();
    heatr_conf.enable = 0x1;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    let rslt = unsafe { bme68x_set_heatr_conf(1 as libc::c_int as u8, &mut heatr_conf, &mut bme) };
    check_rslt(String::from("bme68x_set_heatr_conf"), rslt)?;

    let time_ms = std::time::Instant::now();
    println!("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
    for sample_count in 0..300 {
        // Set operating mode
        let rslt = unsafe { bme68x_set_op_mode(1 as libc::c_int as u8, &mut bme) };
        check_rslt(String::from("bme68x_set_op_mode"), rslt)?;

        // Delay the remaining duration that can be used for heating
        let del_period = unsafe {
            (bme68x_get_meas_dur(1 as libc::c_int as u8, &mut conf, &mut bme)).wrapping_add(
                (heatr_conf.heatr_dur as libc::c_int * 1000 as libc::c_int) as libc::c_uint,
            )
        };
        unsafe {
            (bme.delay_us).expect("non-null function pointer")(del_period, bme.intf_ptr);
        }

        // Get the sensor data
        let mut n_fields = 0;
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
        let rslt = unsafe { bme68x_get_data(1, &mut data, &mut n_fields, &mut bme) };
        check_rslt(String::from("bme68x_get_data"), rslt)?;

        if n_fields != 0 {
            println!(
                "{}, {:?}, {:.2}, {:.2}, {:.2} {:.2} {:x}",
                sample_count,
                time_ms.elapsed().as_millis(),
                data.temperature,
                data.pressure,
                data.humidity,
                data.gas_resistance,
                data.status,
            );
        }
    }

    Ok(())
}
