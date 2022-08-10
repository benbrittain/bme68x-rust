use std::process::Command;

mod bme68x;
mod interface;

use bme68x::{Device, *};
use interface::{check_rslt, Error, Interface};

struct SpiDriver {}

impl Interface for SpiDriver {
    fn interface_type(&self) -> CommInterface {
        CommInterface::SPI
    }

    fn delay(&self, period: u32) {
        let delay = std::time::Duration::from_micros(period as u64);
        std::thread::sleep(delay);
    }

    fn write(&self, reg_addr: u8, reg_data: &[u8]) -> Result<(), Error> {
        let data: String = reg_data.iter().map(|b| format!("0x{:x},", b)).collect();
        let cmd = format!("s w 0x{:x} w {} u", reg_addr, data);
        let output = Command::new("/home/ben/workspace/spidriver/c/build/spicl")
            .arg("/dev/ttyUSB1")
            .args(cmd.split(" "))
            .output()
            .map_err(|_| Error::CommunicationFailure)?;
        Ok(())
    }

    fn read(&self, reg_addr: u8, reg_data: &mut [u8]) -> Result<(), Error> {
        let len = reg_data.len();
        let cmd = format!("s w 0x{:x} r {len} u", reg_addr);
        let output = Command::new("/home/ben/workspace/spidriver/c/build/spicl")
            .arg("/dev/ttyUSB1")
            .args(cmd.split(" "))
            .output()
            .map_err(|_| Error::CommunicationFailure)?;

        let return_bytes: Vec<u8> = std::str::from_utf8(&output.stdout)
            .unwrap()
            .trim()
            .split(",")
            .map(|s| u8::from_str_radix(s.trim_start_matches("0x"), 16).unwrap())
            .collect();
        reg_data.copy_from_slice(&return_bytes[..len as usize]);
        Ok(())
    }
}

fn main() -> Result<(), Error> {
    // Init interface
    let mut bme = Device::new(SpiDriver {});

    // Init bme68x
    bme.init()?;

    // configure device
    let mut conf: DeviceConf = DeviceConf::default();
    conf.filter = 0;
    conf.odr = 8;
    conf.os_hum = 5;
    conf.os_pres = 1;
    conf.os_temp = 2;
    let rslt = unsafe { bme68x_set_conf(&mut conf, &mut bme) };
    check_rslt(rslt)?;

    // configure heater
    let mut heatr_conf: HeaterConf = HeaterConf::default();
    heatr_conf.enable = 0x1;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    let rslt = unsafe { bme68x_set_heatr_conf(1 as libc::c_int as u8, &mut heatr_conf, &mut bme) };
    check_rslt(rslt)?;

    let time_ms = std::time::Instant::now();
    println!("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
    for sample_count in 0..300 {
        // Set operating mode
        let rslt = unsafe { bme68x_set_op_mode(1 as libc::c_int as u8, &mut bme) };
        check_rslt(rslt)?;

        // Delay the remaining duration that can be used for heating
        let del_period = unsafe {
            (bme68x_get_meas_dur(1 as libc::c_int as u8, &mut conf, &mut bme)).wrapping_add(
                (heatr_conf.heatr_dur as libc::c_int * 1000 as libc::c_int) as libc::c_uint,
            )
        };
        unsafe {
            bme.intf.delay(del_period);
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
        check_rslt(rslt)?;

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
