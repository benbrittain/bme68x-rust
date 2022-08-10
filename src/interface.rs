use crate::bme68x::*;

/// BME68X Errors
#[derive(Debug)]
pub enum Error {
    /// Null pointer
    NullPointer,
    /// Communication failure
    CommunicationFailure,
    /// Incorrect length parameter
    IncorrectLengthParameter,
    /// Device not found
    DeviceNotFound,
    /// Self test error
    SelfTestError,
    /// No new data found
    NoNewDataFound,
    /// Unknown error code
    Unknown,
}

pub fn check_rslt(rslt: i8) -> Result<(), Error> {
    match rslt {
        0 => Ok(()),
        -1 => Err(Error::NullPointer),
        -2 => Err(Error::CommunicationFailure),
        -3 => Err(Error::DeviceNotFound),
        -4 => Err(Error::IncorrectLengthParameter),
        -5 => Err(Error::SelfTestError),
        2 => Err(Error::NoNewDataFound),
        _ => Err(Error::Unknown),
    }
}

pub trait Interface {
    /// Communication to the bme68x device occurs over SPI or I2C.
    fn interface_type(&self) -> CommInterface;

    /// Function for reading the sensor's registers through SPI bus.
    fn read(&self, _reg_addr: u8, _reg_data: &mut [u8]) -> Result<(), Error>;

    /// Function for writing the sensor's registers through SPI bus.
    fn write(&self, _reg_addr: u8, _reg_data: &[u8]) -> Result<(), Error>;

    /// Function for delaying in Microseconds.
    fn delay(&self, _us: u32);

    #[doc(hidden)]
    unsafe fn read_raw(&self, reg_addr: u8, reg_data: *mut u8, len: u32) -> i8 {
        let reg_slice: &mut [u8] =
            &mut *core::ptr::slice_from_raw_parts_mut(reg_data, len as usize);
        if self.read(reg_addr, reg_slice).is_ok() {
            0
        } else {
            -1
        }
    }

    #[doc(hidden)]
    unsafe fn write_raw(&self, reg_addr: u8, reg_data: *const u8, len: u32) -> i8 {
        let reg_slice: &[u8] = &*core::ptr::slice_from_raw_parts(reg_data, len as usize);
        if self.write(reg_addr, reg_slice).is_ok() {
            0
        } else {
            -1
        }
    }
}
