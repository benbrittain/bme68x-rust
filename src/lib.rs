#![no_std]
// TODO work through these and remove them
#![allow(dead_code)]
#![allow(unused_assignments)]

mod bme68x;
mod internal;
pub use bme68x::{
    CommInterface, Device, DeviceConfig, Filter, GasHeaterConfig, Odr, OperationMode, Sample,
    SensorData,
};

mod interface;
pub use interface::{Error, Interface};

/// High I2C Address of the sensor
pub const I2C_ADDR_LOW: u8 = 0x76;

/// Low I2C Address of the sensor
pub const I2C_ADDR_HIGH: u8 = 0x77;
