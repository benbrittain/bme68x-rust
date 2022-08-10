use bme68x_rust::{CommInterface, Device, DeviceConf, Error, HeaterConf, Interface, SensorData};
use std::process::Command;

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
        Command::new("/home/ben/workspace/spidriver/c/build/spicl")
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
    bme.set_conf(&mut conf)?;

    // configure heater
    let mut heatr_conf: HeaterConf = HeaterConf::default();
    heatr_conf.enable = 0x1;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    bme.set_heatr_conf(1, &mut heatr_conf)?;

    let time_ms = std::time::Instant::now();
    println!("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
    for sample_count in 0..300 {
        // Set operating mode
        bme.set_op_mode(1)?;

        // Delay the remaining duration that can be used for heating
        let del_period = bme
            .get_meas_dur(1, &mut conf)
            .wrapping_add(heatr_conf.heatr_dur as u32 * 1000);
        bme.intf.delay(del_period);

        // Get the sensor data
        let mut n_fields = 0;
        let mut data: SensorData = SensorData::default();
        bme.get_data(1, &mut data, &mut n_fields)?;

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
