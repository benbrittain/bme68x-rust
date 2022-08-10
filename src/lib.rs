#![no_std]
// TODO work through these and remove them
#![allow(dead_code)]
#![allow(unused_assignments)]

mod bme68x;
mod internal;
pub use bme68x::{CommInterface, Device, DeviceConf, HeaterConf, SensorData};

mod interface;
pub use interface::{Error, Interface};
