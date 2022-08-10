# bme68x-rust

A Rust implementation of the BME68X Sensor API.


### Implementation
This crate took a somewhat novel approach of using [c2rust](https://c2rust.com) as it's primary way of being ported. This was done so that the internal logic matches the upstream C library as closely as possible. It has been edited to provide a safe and ergonomic API. (Work in Progress)


### Examples
Examples are assumed to run on a host computer and have [spidriver](https://spidriver.com/) peripheral attached.
```
cargo run --example forced_mode -- --spicl /path/to/spicl --tty /dev/ttyUSB1
```
