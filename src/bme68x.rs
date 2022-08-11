use libc;

use crate::interface::{check_rslt, Error, Interface};
use crate::internal::*;

/// Operation mode of the sensor.
#[derive(PartialEq)]
pub enum OperationMode {
    /// No measurements are performed. Minimal power consumption.
    Sleep = 0,
    /// Single TPHG cycle is performed. Gas sensor heater only operates during gas measurement.
    /// Returns to Sleep afterwards.
    Forced = 1,
    /// Multiple TPHG cycles are performed. Gas sensor heater operates in parallel to TPH
    /// measurement. Does not return to Sleep Mode.
    Parallel = 2,
    Sequential = 3,
}

/// ODR/Standby time macros
#[repr(u8)]
pub enum Odr {
    /// Standby time of 0.59ms
    Standby0_59Ms = 0,
    /// Standby time of 62.5ms
    Standby62_5Ms = 1,
    /// Standby time of 125ms
    Standby125Ms = 2,
    /// Standby time of 250ms
    Standby250Ms = 3,
    /// Standby time of 500ms
    Standby500Ms = 4,
    /// Standby time of 1s
    Standby1000Ms = 5,
    /// Standby time of 10ms
    Standby10Ms = 6,
    /// Standby time of 20ms
    Standby20Ms = 7,
    /// No standby time
    StandbyNone = 8,
}

/// Possible IIR Filter settings
#[repr(u8)]
pub enum Filter {
    /// Switch off the filter
    Off = 0u8,
    /// Filter coefficient of 2
    Size1 = 1,
    /// Filter coefficient of 4
    Size3 = 2,
    /// Filter coefficient of 8
    Size7 = 3,
    /// Filter coefficient of 16
    Size15 = 4,
    /// Filter coefficient of 32
    Size31 = 5,
    /// Filter coefficient of 64
    Size63 = 6,
    /// Filter coefficient of 128
    Size127 = 7,
}

/// Hardware communication interface (SPI & I2C)
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum CommInterface {
    SPI = 0,
    I2C = 1,
}

#[derive(Copy, Clone, Default)]
#[repr(C)]
pub struct SensorData {
    pub status: u8,
    pub gas_index: u8,
    pub meas_index: u8,
    pub res_heat: u8,
    pub idac: u8,
    pub gas_wait: u8,
    pub temperature: libc::c_float,
    pub pressure: libc::c_float,
    pub humidity: libc::c_float,
    pub gas_resistance: libc::c_float,
}

#[derive(Debug, Copy, Clone, Default)]
#[repr(C)]
pub struct CalibrationData {
    pub par_h1: u16,
    pub par_h2: u16,
    pub par_h3: i8,
    pub par_h4: i8,
    pub par_h5: i8,
    pub par_h6: u8,
    pub par_h7: i8,
    pub par_gh1: i8,
    pub par_gh2: i16,
    pub par_gh3: i8,
    pub par_t1: u16,
    pub par_t2: i16,
    pub par_t3: i8,
    pub par_p1: u16,
    pub par_p2: i16,
    pub par_p3: i8,
    pub par_p4: i16,
    pub par_p5: i16,
    pub par_p6: i8,
    pub par_p7: i8,
    pub par_p8: i16,
    pub par_p9: i16,
    pub par_p10: u8,
    pub t_fine: libc::c_float,
    pub res_heat_range: u8,
    pub res_heat_val: i8,
    pub range_sw_err: i8,
}

#[derive(Copy, Clone, Default)]
/// Sensor settings structure
pub struct DeviceConfig {
    os_hum: u8,
    /// Temperature oversampling.
    os_temp: u8,
    /// Pressure oversampling.
    os_pres: u8,
    /// Filter coefficient.
    filter: u8,
    /// Standby time between sequential mode measurement profiles.
    odr: u8,
}

/// Oversampling setting
pub enum Sample {
    /// Switch off measurements
    Off = 0,
    /// Perform 1 measurement
    Once = 1,
    /// Perform 2 measurements
    X2 = 2,
    /// Perform 4 measurements
    X4 = 3,
    /// Perform 8 measurements
    X8 = 4,
    /// Perform 16 measurements
    X16 = 5,
}

impl DeviceConfig {
    pub fn filter(&self, filter: Filter) -> Self {
        let mut conf = *self;
        conf.filter = filter as u8;
        conf
    }
    pub fn odr(&self, odr: Odr) -> Self {
        let mut conf = *self;
        conf.odr = odr as u8;
        conf
    }
    pub fn oversample_humidity(&self, h: Sample) -> Self {
        let mut conf = *self;
        conf.os_hum = h as u8;
        conf
    }
    pub fn oversample_pressure(&self, p: Sample) -> Self {
        let mut conf = *self;
        conf.os_pres = p as u8;
        conf
    }
    pub fn oversample_temperature(&self, t: Sample) -> Self {
        let mut conf = *self;
        conf.os_temp = t as u8;
        conf
    }
}

#[derive(Copy, Clone)]
#[repr(C)]
pub struct GasHeaterConfig {
    pub(crate) enable: u8,
    pub(crate) heatr_temp: u16,
    pub(crate) heatr_dur: u16,
    pub(crate) heatr_temp_prof: *mut u16,
    pub(crate) heatr_dur_prof: *mut u16,
    pub(crate) profile_len: u8,
    pub(crate) shared_heatr_dur: u16,
}

impl GasHeaterConfig {
    pub fn enable(&self) -> Self {
        let mut conf = *self;
        conf.enable = true as u8;
        conf
    }
    pub fn heater_temp(&self, temp: u16) -> Self {
        let mut conf = *self;
        conf.heatr_temp = temp;
        conf
    }
    pub fn heater_duration(&self, duration: u16) -> Self {
        let mut conf = *self;
        conf.heatr_dur = duration;
        conf
    }
    pub fn disable(&self) -> Self {
        let mut conf = *self;
        conf.enable = false as u8;
        conf
    }
}

impl Default for GasHeaterConfig {
    fn default() -> Self {
        Self {
            enable: 0,
            heatr_temp: 0,
            heatr_dur: 0,
            heatr_temp_prof: 0 as *mut u16,
            heatr_dur_prof: 0 as *mut u16,
            profile_len: 0,
            shared_heatr_dur: 0,
        }
    }
}

/// BME68x Device Controller
///
/// ```
///  let mut bme = Device::initialize(SpiDriver {
///      spicl: args.spicl,
///      tty: args.tty,
///  })?;
///
///  bme.set_config(DeviceConfigig::default()
///                  .filter(0)
///                  .odr(8)
///                  .oversample_humidity(5)
///                  .oversample_pressure(1)
///                  .oversample_temperature(2))?;
/// ```
///
pub struct Device<I: Interface> {
    pub interface: I,
    pub config: DeviceConfig,
    pub gas_heater_config: GasHeaterConfig,
    pub(crate) variant_id: u32,
    pub(crate) mem_page: u8,
    pub(crate) amb_temp: i8,
    pub(crate) calib: CalibrationData,
    pub(crate) interface_result: i8,
    pub(crate) info_msg: u8,
}

impl<I: Interface> Device<I> {
    /// Initialization.
    ///
    /// Reads the chip-id of the sensor which is the first step to verify the sensor
    /// and also calibrates the sensor.
    pub fn initialize(interface: I) -> Result<Self, Error> {
        // NOTE moved amb_temp from bme68x_interface_init since everything else that function did
        // is now contained within the interface trait.
        let amb_temp = 25;
        let mut device = Self {
            variant_id: 0,
            interface,
            mem_page: 0,
            amb_temp,
            gas_heater_config: GasHeaterConfig::default(),
            config: DeviceConfig::default(),
            calib: CalibrationData::default(),
            interface_result: 0,
            info_msg: 0,
        };

        device.soft_reset()?;
        let mut chip_id = 0;
        device.get_regs(
            0xd0 as libc::c_int as u8,
            &mut chip_id,
            1 as libc::c_int as u32,
        )?;
        let mut rslt: i8 = 0;
        if chip_id == 0x61 {
            rslt = read_variant_id(&mut device);
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = get_calib_data(&mut device);
            }
        } else {
            rslt = -(3 as libc::c_int) as i8;
        }
        check_rslt(rslt)?;
        Ok(device)
    }

    /// Writes the given data to the register address of the sensor.
    pub(crate) fn set_regs(
        &mut self,
        reg_addr: *const u8,
        reg_data: *const u8,
        len: u32,
    ) -> Result<(), Error> {
        unsafe {
            let mut rslt: i8 = 0;
            let mut tmp_buff: [u8; 20] = [0; 20];
            let mut index: u16 = 0;
            rslt = null_ptr_check(self);
            if rslt as libc::c_int == 0 as libc::c_int && !reg_addr.is_null() && !reg_data.is_null()
            {
                if len > 0 as libc::c_int as libc::c_uint
                    && len <= (20 as libc::c_int / 2 as libc::c_int) as libc::c_uint
                {
                    index = 0 as libc::c_int as u16;
                    while (index as libc::c_uint) < len {
                        if (*self).interface.interface_type() == CommInterface::SPI {
                            rslt = set_mem_page(*reg_addr.offset(index as isize), self);
                            tmp_buff[(2 as libc::c_int * index as libc::c_int) as usize] =
                                (*reg_addr.offset(index as isize) as libc::c_int
                                    & 0x7f as libc::c_int) as u8;
                        } else {
                            tmp_buff[(2 as libc::c_int * index as libc::c_int) as usize] =
                                *reg_addr.offset(index as isize);
                        }
                        tmp_buff[(2 as libc::c_int * index as libc::c_int + 1 as libc::c_int)
                            as usize] = *reg_data.offset(index as isize);
                        index = index.wrapping_add(1);
                    }
                    if rslt as libc::c_int == 0 as libc::c_int {
                        (*self).interface_result = (*self).interface.write_raw(
                            tmp_buff[0],
                            &mut *tmp_buff.as_mut_ptr().offset(1 as libc::c_int as isize),
                            (2 as libc::c_int as libc::c_uint)
                                .wrapping_mul(len)
                                .wrapping_sub(1 as libc::c_int as libc::c_uint),
                        );
                        if (*self).interface_result as libc::c_int != 0 as libc::c_int {
                            rslt = -(2 as libc::c_int) as i8;
                        }
                    }
                } else {
                    rslt = -(4 as libc::c_int) as i8;
                }
            } else {
                rslt = -(1 as libc::c_int) as i8;
            }
            check_rslt(rslt)
        }
    }

    /// Reads the data from the given register address of sensor.
    pub(crate) fn get_regs(
        &mut self,
        mut reg_addr: u8,
        reg_data: *mut u8,
        len: u32,
    ) -> Result<(), Error> {
        unsafe {
            let mut rslt: i8 = 0;
            rslt = null_ptr_check(self);
            if rslt as libc::c_int == 0 as libc::c_int && !reg_data.is_null() {
                if (*self).interface.interface_type() == CommInterface::SPI {
                    rslt = set_mem_page(reg_addr, self);
                    if rslt as libc::c_int == 0 as libc::c_int {
                        reg_addr = (reg_addr as libc::c_int | 0x80 as libc::c_int) as u8;
                    }
                }
                (*self).interface_result = (*self).interface.read_raw(reg_addr, reg_data, len);
                if (*self).interface_result as libc::c_int != 0 as libc::c_int {
                    rslt = -(2 as libc::c_int) as i8;
                }
            } else {
                rslt = -(1 as libc::c_int) as i8;
            }
            check_rslt(rslt)
        }
    }

    /// Triggers a soft-resets of the sensor.
    pub fn soft_reset(&mut self) -> Result<(), Error> {
        unsafe {
            let mut rslt: i8 = 0;
            let mut reg_addr: u8 = 0xe0 as libc::c_int as u8;
            let mut soft_rst_cmd: u8 = 0xb6 as libc::c_int as u8;
            rslt = null_ptr_check(self);
            if rslt as libc::c_int == 0 as libc::c_int {
                if (*self).interface.interface_type() == CommInterface::SPI {
                    rslt = get_mem_page(self);
                }
                if rslt as libc::c_int == 0 as libc::c_int {
                    self.set_regs(&mut reg_addr, &mut soft_rst_cmd, 1 as libc::c_int as u32)?;
                    (*self).interface.delay(10000 as libc::c_uint);
                    if (*self).interface.interface_type() == CommInterface::SPI {
                        rslt = get_mem_page(self);
                    }
                }
            }
            check_rslt(rslt)
        }
    }

    /// Used to set the oversampling, filter and odr configuration.
    pub fn set_config(&mut self, conf: DeviceConfig) -> Result<(), Error> {
        self.config = conf;
        unsafe {
            let mut rslt: i8 = 0;
            let mut odr20: u8 = 0;
            let mut odr3: u8 = 1;
            let mut reg_array: [u8; 5] = [0x71, 0x72, 0x73, 0x74, 0x75];
            let mut data_array: [u8; 5] = [0, 0, 0, 0, 0];
            let current_op_mode = self.get_op_mode()?;
            self.set_op_mode(OperationMode::Sleep)?;
            self.get_regs(reg_array[0], data_array.as_mut_ptr(), 5)?;
            (*self).info_msg = 0 as u8;
            if rslt == 0 {
                rslt = boundary_check(&mut self.config.filter, 7 as u8, self);
            }
            if rslt == 0 {
                rslt = boundary_check(&mut self.config.os_temp, 5 as u8, self);
            }
            if rslt == 0 {
                rslt = boundary_check(&mut self.config.os_pres, 5 as u8, self);
            }
            if rslt == 0 {
                rslt = boundary_check(&mut self.config.os_hum, 5 as u8, self);
            }
            if rslt == 0 {
                rslt = boundary_check(&mut self.config.odr, 8 as u8, self);
            }
            if rslt == 0 {
                data_array[4] = (data_array[4] as libc::c_int & !(0x1c as libc::c_int)
                    | (self.config.filter as libc::c_int) << 2 as libc::c_int & 0x1c as libc::c_int)
                    as u8;
                data_array[3] = (data_array[3] as libc::c_int & !(0xe0 as libc::c_int)
                    | (self.config.os_temp as libc::c_int) << 5 as libc::c_int
                        & 0xe0 as libc::c_int) as u8;
                data_array[3] = (data_array[3] as libc::c_int & !(0x1c as libc::c_int)
                    | (self.config.os_pres as libc::c_int) << 2 as libc::c_int
                        & 0x1c as libc::c_int) as u8;
                data_array[1] = (data_array[1] as libc::c_int & !(0x7 as libc::c_int)
                    | self.config.os_hum as libc::c_int & 0x7 as libc::c_int)
                    as u8;
                if self.config.odr as libc::c_int != 8 as libc::c_int {
                    odr20 = self.config.odr;
                    odr3 = 0 as libc::c_int as u8;
                }
                data_array[4] = (data_array[4] as libc::c_int & !(0xe0 as libc::c_int)
                    | (odr20 as libc::c_int) << 5 as libc::c_int & 0xe0 as libc::c_int)
                    as u8;
                data_array[0] = (data_array[0] as libc::c_int & !(0x80 as libc::c_int)
                    | (odr3 as libc::c_int) << 7 as libc::c_int & 0x80 as libc::c_int)
                    as u8;
            }
            if rslt == 0 {
                self.set_regs(reg_array.as_mut_ptr(), data_array.as_mut_ptr(), 5)?;
            }
            if current_op_mode != OperationMode::Sleep && rslt == 0 {
                self.set_op_mode(current_op_mode)?;
            }
            return check_rslt(rslt);
        }
    }

    /// Used to get the oversampling, filter and odr configurations.
    pub fn get_conf(&mut self, mut conf: &mut DeviceConfig) -> Result<(), Error> {
        let reg_addr: u8 = 0x71 as libc::c_int as u8;
        let mut data_array: [u8; 5] = [0; 5];
        self.get_regs(reg_addr, data_array.as_mut_ptr(), 5 as libc::c_int as u32)?;
        (*conf).os_hum = (data_array[1] as libc::c_int & 0x7 as libc::c_int) as u8;
        (*conf).filter =
            ((data_array[4] as libc::c_int & 0x1c as libc::c_int) >> 2 as libc::c_int) as u8;
        (*conf).os_temp =
            ((data_array[3] as libc::c_int & 0xe0 as libc::c_int) >> 5 as libc::c_int) as u8;
        (*conf).os_pres =
            ((data_array[3] as libc::c_int & 0x1c as libc::c_int) >> 2 as libc::c_int) as u8;
        if (data_array[0] as libc::c_int & 0x80 as libc::c_int) >> 7 as libc::c_int != 0 {
            (*conf).odr = 8 as libc::c_int as u8;
        } else {
            (*conf).odr =
                ((data_array[4] as libc::c_int & 0xe0 as libc::c_int) >> 5 as libc::c_int) as u8;
        }
        Ok(())
    }

    /// Used to set the operation mode of the sensor.
    pub fn set_op_mode(&mut self, op_mode: OperationMode) -> Result<(), Error> {
        let op_mode = op_mode as u8;
        let mut tmp_pow_mode: u8 = 0;
        let mut reg_addr: u8 = 0x74 as libc::c_int as u8;
        loop {
            self.get_regs(
                0x74 as libc::c_int as u8,
                &mut tmp_pow_mode,
                1 as libc::c_int as u32,
            )?;
            let pow_mode = (tmp_pow_mode as libc::c_int & 0x3 as libc::c_int) as u8;
            if pow_mode as libc::c_int != 0 as libc::c_int {
                tmp_pow_mode = (tmp_pow_mode as libc::c_int & !(0x3 as libc::c_int)) as u8;
                self.set_regs(&mut reg_addr, &mut tmp_pow_mode, 1 as libc::c_int as u32)?;
                (*self).interface.delay(10000 as libc::c_uint);
            }
            if !(pow_mode as libc::c_int != 0 as libc::c_int) {
                break;
            }
        }
        if op_mode as libc::c_int != 0 as libc::c_int {
            tmp_pow_mode = (tmp_pow_mode as libc::c_int & !(0x3 as libc::c_int)
                | op_mode as libc::c_int & 0x3 as libc::c_int) as u8;
            self.set_regs(&mut reg_addr, &mut tmp_pow_mode, 1 as libc::c_int as u32)?;
        }
        Ok(())
    }

    /// Used to get the operation mode of the sensor.
    pub fn get_op_mode(&mut self) -> Result<OperationMode, Error> {
        let mut mode: u8 = 0;
        self.get_regs(
            0x74 as libc::c_int as u8,
            &mut mode,
            1 as libc::c_int as u32,
        )?;
        let op_mode = (mode as libc::c_int & 0x3 as libc::c_int) as u8;
        Ok(match op_mode {
            0 => OperationMode::Sleep,
            1 => OperationMode::Forced,
            2 => OperationMode::Parallel,
            _ => unreachable!(),
        })
    }

    /// Used to get the remaining duration that can be used for heating.
    pub fn get_measure_duration(&mut self, op_mode: OperationMode) -> u32 {
        unsafe {
            let mut rslt: i8 = 0;
            let mut meas_dur: u32 = 0 as libc::c_int as u32;
            let mut meas_cycles: u32 = 0;
            let os_to_meas_cycles: [u8; 6] = [
                0 as libc::c_int as u8,
                1 as libc::c_int as u8,
                2 as libc::c_int as u8,
                4 as libc::c_int as u8,
                8 as libc::c_int as u8,
                16 as libc::c_int as u8,
            ];
            rslt = boundary_check(&mut self.config.os_temp, 5 as libc::c_int as u8, self);
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = boundary_check(&mut self.config.os_pres, 5 as libc::c_int as u8, self);
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                rslt = boundary_check(&mut self.config.os_hum, 5 as libc::c_int as u8, self);
            }
            if rslt as libc::c_int == 0 as libc::c_int {
                meas_cycles = os_to_meas_cycles[self.config.os_temp as usize] as u32;
                meas_cycles = (meas_cycles as libc::c_uint)
                    .wrapping_add(os_to_meas_cycles[self.config.os_pres as usize] as libc::c_uint)
                    as u32 as u32;
                meas_cycles = (meas_cycles as libc::c_uint)
                    .wrapping_add(os_to_meas_cycles[self.config.os_hum as usize] as libc::c_uint)
                    as u32 as u32;
                meas_dur = meas_cycles.wrapping_mul(1963 as libc::c_uint);
                meas_dur = (meas_dur as libc::c_uint).wrapping_add(
                    (477 as libc::c_int as libc::c_uint).wrapping_mul(4 as libc::c_uint),
                ) as u32 as u32;
                meas_dur = (meas_dur as libc::c_uint).wrapping_add(
                    (477 as libc::c_int as libc::c_uint).wrapping_mul(5 as libc::c_uint),
                ) as u32 as u32;
                if op_mode != OperationMode::Parallel {
                    meas_dur = (meas_dur as libc::c_uint).wrapping_add(1000 as libc::c_uint) as u32;
                }
            }
            return meas_dur;
        }
    }

    /// Reads the pressure, temperature humidity and gas data from the sensor,
    /// compensates the data and store it in the SensorData structure instance passed by the user.
    ///
    /// TODO refactor and make safer.
    pub fn get_data(
        &mut self,
        op_mode: u8,
        data: *mut SensorData,
        n_data: &mut u8,
    ) -> Result<(), Error> {
        unsafe {
            let mut rslt: i8 = 0;
            let mut i: u8 = 0 as libc::c_int as u8;
            let mut j: u8 = 0 as libc::c_int as u8;
            let mut new_fields: u8 = 0 as libc::c_int as u8;
            let mut field_ptr: [*mut SensorData; 3] = [
                0 as *mut SensorData,
                0 as *mut SensorData,
                0 as *mut SensorData,
            ];
            let mut field_data: [SensorData; 3] = [
                {
                    let init = SensorData {
                        status: 0 as libc::c_int as u8,
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
                    init
                },
                SensorData {
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
                },
                SensorData {
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
                },
            ];
            field_ptr[0] =
                &mut *field_data.as_mut_ptr().offset(0 as libc::c_int as isize) as *mut SensorData;
            field_ptr[1] =
                &mut *field_data.as_mut_ptr().offset(1 as libc::c_int as isize) as *mut SensorData;
            field_ptr[2] =
                &mut *field_data.as_mut_ptr().offset(2 as libc::c_int as isize) as *mut SensorData;
            rslt = null_ptr_check(self);
            if rslt as libc::c_int == 0 as libc::c_int && !data.is_null() {
                if op_mode as libc::c_int == 1 as libc::c_int {
                    rslt = read_field_data(0 as libc::c_int as u8, data, self);
                    if rslt as libc::c_int == 0 as libc::c_int {
                        if (*data).status as libc::c_int & 0x80 as libc::c_int != 0 {
                            new_fields = 1 as libc::c_int as u8;
                        } else {
                            new_fields = 0 as libc::c_int as u8;
                            rslt = 2 as libc::c_int as i8;
                        }
                    }
                } else if op_mode as libc::c_int == 2 as libc::c_int
                    || op_mode as libc::c_int == 3 as libc::c_int
                {
                    rslt =
                        read_all_field_data(field_ptr.as_mut_ptr() as *const *mut SensorData, self);
                    new_fields = 0 as libc::c_int as u8;
                    i = 0 as libc::c_int as u8;
                    while (i as libc::c_int) < 3 as libc::c_int
                        && rslt as libc::c_int == 0 as libc::c_int
                    {
                        if (*field_ptr[i as usize]).status as libc::c_int & 0x80 as libc::c_int != 0
                        {
                            new_fields = new_fields.wrapping_add(1);
                        }
                        i = i.wrapping_add(1);
                    }
                    i = 0 as libc::c_int as u8;
                    while (i as libc::c_int) < 2 as libc::c_int
                        && rslt as libc::c_int == 0 as libc::c_int
                    {
                        j = (i as libc::c_int + 1 as libc::c_int) as u8;
                        while (j as libc::c_int) < 3 as libc::c_int {
                            sort_sensor_data(i, j, field_ptr.as_mut_ptr());
                            j = j.wrapping_add(1);
                        }
                        i = i.wrapping_add(1);
                    }
                    i = 0 as libc::c_int as u8;
                    while (i as libc::c_int) < 3 as libc::c_int
                        && rslt as libc::c_int == 0 as libc::c_int
                    {
                        *data.offset(i as isize) = *field_ptr[i as usize];
                        i = i.wrapping_add(1);
                    }
                    if new_fields as libc::c_int == 0 as libc::c_int {
                        rslt = 2 as libc::c_int as i8;
                    }
                } else {
                    rslt = 1 as libc::c_int as i8;
                }
                *n_data = new_fields;
            } else {
                rslt = -(1 as libc::c_int) as i8;
            }
            check_rslt(rslt)
        }
    }

    /// Used to set the gas configuration of the sensor.
    pub fn set_gas_heater_conf(
        &mut self,
        op_mode: OperationMode,
        conf: GasHeaterConfig,
    ) -> Result<(), Error> {
        unsafe {
            let mut rslt: i8 = 0;
            let mut nb_conv: u8 = 0 as libc::c_int as u8;
            let mut hctrl: u8 = 0;
            let mut run_gas: u8 = 0 as libc::c_int as u8;
            let mut ctrl_gas_data: [u8; 2] = [0; 2];
            let mut ctrl_gas_addr: [u8; 2] = [0x70 as libc::c_int as u8, 0x71 as libc::c_int as u8];
            self.set_op_mode(OperationMode::Sleep)?;
            // NOTE BWB, not the same as self.set_config
            rslt = set_conf(&conf, op_mode as u8, &mut nb_conv, self);
            if rslt as libc::c_int == 0 as libc::c_int {
                self.get_regs(
                    0x70 as libc::c_int as u8,
                    ctrl_gas_data.as_mut_ptr(),
                    2 as libc::c_int as u32,
                )?;
                if rslt as libc::c_int == 0 as libc::c_int {
                    if conf.enable as libc::c_int == 0x1 as libc::c_int {
                        hctrl = 0 as libc::c_int as u8;
                        if (*self).variant_id == 0x1 as libc::c_int as libc::c_uint {
                            run_gas = 0x2 as libc::c_int as u8;
                        } else {
                            run_gas = 0x1 as libc::c_int as u8;
                        }
                    } else {
                        hctrl = 0x1 as libc::c_int as u8;
                        run_gas = 0 as libc::c_int as u8;
                    }
                    ctrl_gas_data[0] = (ctrl_gas_data[0] as libc::c_int & !(0x8 as libc::c_int)
                        | (hctrl as libc::c_int) << 3 as libc::c_int & 0x8 as libc::c_int)
                        as u8;
                    ctrl_gas_data[1] = (ctrl_gas_data[1] as libc::c_int & !(0xf as libc::c_int)
                        | nb_conv as libc::c_int & 0xf as libc::c_int)
                        as u8;
                    ctrl_gas_data[1] = (ctrl_gas_data[1] as libc::c_int & !(0x30 as libc::c_int)
                        | (run_gas as libc::c_int) << 4 as libc::c_int & 0x30 as libc::c_int)
                        as u8;
                    self.set_regs(
                        ctrl_gas_addr.as_mut_ptr(),
                        ctrl_gas_data.as_mut_ptr(),
                        2 as libc::c_int as u32,
                    )?;
                }
            }
            check_rslt(rslt)
        }
    }

    /// Used to get the gas configuration of the sensor.
    pub fn get_gas_heater_conf(&mut self, config: GasHeaterConfig) -> Result<(), Error> {
        self.gas_heater_config = config;
        unsafe {
            let mut rslt: i8 = 0;
            let mut data_array: [u8; 10] = [0 as libc::c_int as u8, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            let mut i: u8 = 0;
            self.get_regs(
                0x5a as libc::c_int as u8,
                data_array.as_mut_ptr(),
                10 as libc::c_int as u32,
            )?;
            if rslt as libc::c_int == 0 as libc::c_int {
                if !(self.gas_heater_config.heatr_dur_prof).is_null()
                    && !(self.gas_heater_config.heatr_temp_prof).is_null()
                {
                    i = 0 as libc::c_int as u8;
                    while (i as libc::c_int) < 10 as libc::c_int {
                        *(self.gas_heater_config.heatr_temp_prof).offset(i as isize) =
                            data_array[i as usize] as u16;
                        i = i.wrapping_add(1);
                    }
                    self.get_regs(
                        0x64 as libc::c_int as u8,
                        data_array.as_mut_ptr(),
                        10 as libc::c_int as u32,
                    )?;
                    if rslt as libc::c_int == 0 as libc::c_int {
                        i = 0 as libc::c_int as u8;
                        while (i as libc::c_int) < 10 as libc::c_int {
                            *(self.gas_heater_config.heatr_dur_prof).offset(i as isize) =
                                data_array[i as usize] as u16;
                            i = i.wrapping_add(1);
                        }
                    }
                } else {
                    rslt = -(1 as libc::c_int) as i8;
                }
            }
            check_rslt(rslt)
        }
    }
}
