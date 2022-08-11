use bme68x_rust::{
    CommInterface, Device, DeviceConfig, Error, Filter, GasHeaterConfig, Interface, Odr,
    OperationMode, Sample, SensorData,
};
use clap::Parser;
use std::{path::PathBuf, process::Command};

struct SpiDriver {
    spicl: PathBuf,
    tty: PathBuf,
}

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
        Command::new(&self.spicl)
            .arg(&self.tty)
            .args(cmd.split(" "))
            .output()
            .map_err(|_| Error::CommunicationFailure)?;
        Ok(())
    }

    fn read(&self, reg_addr: u8, reg_data: &mut [u8]) -> Result<(), Error> {
        let len = reg_data.len();
        let cmd = format!("s w 0x{:x} r {len} u", reg_addr);
        let output = Command::new(&self.spicl)
            .arg(&self.tty)
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

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Location of the spidriver "spicl" binary.
    #[clap(short, long)]
    spicl: PathBuf,

    /// Location of the tty USB device of the spidriver.
    #[clap(short, long)]
    tty: PathBuf,
}

fn main() -> Result<(), Error> {
    let args = Args::parse();

    // initialize the bme68x device
    let mut bme = Device::initialize(SpiDriver {
        spicl: args.spicl,
        tty: args.tty,
    })?;

    // configure device
    bme.set_config(
        DeviceConfig::default()
            .filter(Filter::Off)
            .odr(Odr::StandbyNone)
            .oversample_humidity(Sample::X16)
            .oversample_pressure(Sample::Once)
            .oversample_temperature(Sample::X2),
    )?;

    // configure heater
    bme.set_gas_heater_conf(
        OperationMode::Forced,
        GasHeaterConfig::default()
            .enable()
            .heater_temp(300)
            .heater_duration(100),
    )?;

    let time_ms = std::time::Instant::now();
    println!("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status");
    for sample_count in 0..300 {
        // Set operating mode
        bme.set_op_mode(OperationMode::Forced)?;

        // Delay the remaining duration that can be used for heating
        let del_period = bme
            .get_measure_duration(OperationMode::Forced)
            .wrapping_add(300 as u32 * 1000);
        bme.interface.delay(del_period);

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
