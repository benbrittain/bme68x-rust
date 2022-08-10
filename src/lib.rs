#![no_std]
// TODO work through these and remove them
#![allow(dead_code)]
#![allow(unused_assignments)]

pub mod bme68x;

mod interface;
pub use interface::{Error, Interface};

// TODO remove this
pub use interface::check_rslt;
