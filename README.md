# bme68x-rust

A Rust implementation of the BME68X Sensor API.


### Examples 
Examples are assumed to run on a host computer and have [spidriver](https://spidriver.com/) peripheral attached.
```
cargo run --example forced_mode -- --spicl /path/to/spicl --tty /dev/ttyUSB1
```
