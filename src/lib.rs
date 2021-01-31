//! Unofficial device driver for [Dyn Pick, Wacoh-tech force-torque sensor](https://wacoh-tech.com/en/products/dynpick/).
//! # Examples
//! ```no_run
//! use dynpick_force_torque_sensor::{DynpickSensorBuilder, Sensitivity, Triplet};
//!
//! let sensitivity = {
//!     let force = Triplet::new(24.9, 24.6, 24.5);
//!     let torque = Triplet::new(1664.7, 1639.7, 1638.0);
//!     Sensitivity::new(force, torque)
//! };
//!
//! let mut sensor = DynpickSensorBuilder::open("/dev/ttyUSB0")
//!     // Automatic sensitivity set is also available. See the document in detail.
//!     .map(|b| b.set_sensitivity_manually(sensitivity))
//!     .and_then(|b| b.build())
//!     .unwrap();
//!
//! sensor.zeroed_next().unwrap(); // Calibration
//!
//! let wrench = sensor.update().unwrap();
//! println!("Force: {}, Torque: {}", wrench.force, wrench.torque);
//! ```
//!
//! # Dependency under Linux environment
//! `libudev-dev` is required under Linux environment. Please install it by  
//! `sudo apt install libudev-dev`
//!
//! # Setup
//! It may be required to customize udev rules if you use usb-connected sensors.
//!
//! [This shell script](https://github.com/Amelia10007/dynpick-force-torque-sensor-rs/blob/master/examples/setup_udev_rule.sh) can be useful for customize (see the file in detail).
//!
//! # Note
//! I tested this crate only by WDF-6M200-3 sensor because I have no other dynpick sensor.
#![warn(missing_docs)]

use itertools::Itertools;
pub use pair_macro::Triplet;
pub use serialport;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::borrow::Cow;
use std::fmt::{self, Display, Formatter};
use std::marker::PhantomData;
use std::str::FromStr;
use std::time::Duration;

/// Marker type for builder.
pub struct Ready;

/// Marker type for builder.
pub struct SensitivityNotSetYet;

/// Builder of a connection to a dynpick sensor.
/// # Examples
/// ```no_run
/// use dynpick_force_torque_sensor::DynpickSensorBuilder;
///
/// let sensor = DynpickSensorBuilder::open("/dev/ttyUSB0")
///     .and_then(|b| b.set_sensitivity_by_builtin_data())
///     .and_then(|b| b.build())
///     .unwrap();
/// ```
pub struct DynpickSensorBuilder<C> {
    /// Serial port device.
    port: Box<dyn SerialPort>,
    /// The sensitivity of the connected sensor.
    sensitivity: Sensitivity,
    /// 👻
    _calibrated: PhantomData<fn() -> C>,
}

impl DynpickSensorBuilder<SensitivityNotSetYet> {
    /// Connects to the dynpick force torque sensor.
    /// # Params
    /// 1. `path` The sensor's path.
    ///
    /// # Returns
    /// `Ok(builder)` if successfully connected, `Err(reason)` if failed.  
    /// Before you use the sensor, you need to calibrate the sensor by calling a calibration method.
    /// See also [`Self::set_sensitivity_by_builtin_data`] or [`Self::set_sensitivity_manually`].
    ///
    /// # Examples
    /// See the example [here](`DynpickSensorBuilder`).
    pub fn open<'a>(
        path: impl Into<Cow<'a, str>>,
    ) -> Result<DynpickSensorBuilder<SensitivityNotSetYet>, Error> {
        // These settings were determined according to the hardware configuration.
        let port = serialport::new(path, 921600)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_millis(1))
            .open()
            .map_err(Error::SerialPort)?;

        let builder = Self {
            port,
            sensitivity: Sensitivity {
                digital_per_newton: Triplet::default(),
                digital_per_newtonmeter: Triplet::default(),
            },
            _calibrated: PhantomData,
        };

        Ok(builder)
    }

    ///  Set the [`Sensitivity`] of the connected sensor by using the specified sensitivity.
    /// # Examples
    /// ```no_run
    /// use dynpick_force_torque_sensor::{DynpickSensorBuilder, Sensitivity, Triplet};
    ///
    /// let sensitivity = {
    ///     let force = Triplet::new(24.9, 24.6, 24.5);
    ///     let torque = Triplet::new(1664.7, 1639.7, 1638.0);
    ///     Sensitivity::new(force, torque)
    /// };
    ///
    /// let sensor = DynpickSensorBuilder::open("/dev/ttyUSB0")
    ///     .map(|b| b.set_sensitivity_manually(sensitivity))
    ///     .and_then(|b| b.build())
    ///     .unwrap();
    /// ```
    pub fn set_sensitivity_manually(self, sensitivity: Sensitivity) -> DynpickSensorBuilder<Ready> {
        DynpickSensorBuilder {
            port: self.port,
            sensitivity,
            _calibrated: PhantomData,
        }
    }

    /// Set the [`Sensitivity`] of the connected sensor, reading its sensitivity from it.  
    /// Some sensors may not support this functionality (`Err(_)` will be returned under this situation).
    /// # Examples
    /// See the example [here](`DynpickSensorBuilder`)
    ///
    /// # Note
    /// This method has not been tested yet because my sensor (WDF-6M200-3) does not support this functionality.
    pub fn set_sensitivity_by_builtin_data(
        mut self,
    ) -> Result<DynpickSensorBuilder<Ready>, Error> {
        const SENSITIVITY_RESPONSE_LENGTH: usize = 46;

        // Send and wait.
        self.port.write_all(&['p' as u8]).map_err(Error::IO)?;
        std::thread::sleep(self.port.timeout());

        let mut res = [0; SENSITIVITY_RESPONSE_LENGTH];
        self.port.read_exact(&mut res).map_err(Error::IO)?;

        let res = std::str::from_utf8(&res).or(Err(Error::Utf8(res.to_vec())))?;
        let (fx, fy, fz, mx, my, mz) = res
            .split(',')
            .map(f64::from_str)
            .filter_map(Result::ok)
            .next_tuple()
            .ok_or(Error::ParseResponse(res.to_owned()))?;

        let force = Triplet::new(fx, fy, fz);
        let torque = Triplet::new(mx, my, mz);

        let sensitivity = Sensitivity::new(force, torque);

        Ok(self.set_sensitivity_manually(sensitivity))
    }
}

impl DynpickSensorBuilder<Ready> {
    /// Consuming this builder, attempts to construct a sensor instance.
    pub fn build(self) -> Result<DynpickSensor, Error> {
        let mut sensor = DynpickSensor {
            port: self.port,
            last_wrench: Wrench::zeroed(),
            sensitivity: self.sensitivity,
        };

        // First single data request.
        sensor.request_next_wrench()?;

        Ok(sensor)
    }
}

/// Dynpick 6-axis force-torque sensor.
pub struct DynpickSensor {
    /// Serial port device.
    port: Box<dyn SerialPort>,
    /// The latest wrench acquired by `update`.
    last_wrench: Wrench,
    /// The sensitivity of the connected sensor.
    sensitivity: Sensitivity,
}

impl DynpickSensor {
    /// Returns the latest wrench that is stored in this instance without communicaing the sensor.
    ///
    /// Use [`Self::update`] instead to obtain a new wrench from the sensor.
    /// # Returns
    /// `Ok(sensor)` if successfully connected, `Err(reason)` if failed.
    pub fn last_wrench(&self) -> Wrench {
        self.last_wrench
    }

    /// Returns the sensitivity of this sensor.
    pub fn sensitivity(&self) -> Sensitivity {
        self.sensitivity
    }

    /// Communicating to the sensor, updates the latest wrench.
    /// # Returns
    /// `Ok(wrench)` if succeeds, `Err(reason)` if failed.
    pub fn update(&mut self) -> Result<Wrench, Error> {
        const WRENCH_RESPONSE_LENGTH: usize = 27;

        let mut res = [0; WRENCH_RESPONSE_LENGTH];
        self.port.read_exact(&mut res).map_err(Error::IO)?;

        let res = std::str::from_utf8(&res).or(Err(Error::Utf8(res.to_vec())))?;

        let (fx, fy, fz, mx, my, mz) = (0..6)
            .map(|i| 1 + i * 4)
            .map(|start| &res[start..start + 4])
            .map(|src| i32::from_str_radix(src, 16))
            .filter_map(Result::ok)
            .next_tuple()
            .ok_or(Error::ParseResponse(res.to_owned()))?;

        let digital_force = Triplet::new(fx, fy, fz);
        let digital_torque = Triplet::new(mx, my, mz);

        let force = digital_force
            .map(|d| d - 8192)
            .map(|d| d as f64)
            .map_entrywise(self.sensitivity.digital_per_newton, |d, s| d / s);

        let torque = digital_torque
            .map(|d| d - 8192)
            .map(|d| d as f64)
            .map_entrywise(self.sensitivity.digital_per_newtonmeter, |d, s| d / s);

        self.last_wrench = Wrench::new(force, torque);

        // Send a request to obtain a new wrench.
        self.request_next_wrench()?;

        Ok(self.last_wrench)
    }

    /// If this method succeeds, the next wrench acquired by [`Self::update`] will be zeroed.  
    /// This methos is useful for zero-point calibration.
    /// # Examples
    /// ```no_run
    /// use dynpick_force_torque_sensor::{DynpickSensorBuilder, Triplet};
    ///
    /// let mut sensor = DynpickSensorBuilder::open("/dev/ttyUSB0")
    ///     .and_then(|b| b.set_sensitivity_by_builtin_data())
    ///     .and_then(|b| b.build())
    ///     .unwrap();
    ///
    /// sensor.zeroed_next().unwrap();
    ///
    /// let wrench = sensor.update().unwrap();
    ///
    /// assert_eq!(wrench.force, Triplet::new(0.0, 0.0, 0.0));
    /// assert_eq!(wrench.torque, Triplet::new(0.0, 0.0, 0.0));
    /// ```
    pub fn zeroed_next(&mut self) -> Result<(), Error> {
        self.port.write_all(&['O' as u8]).map_err(Error::IO)
    }

    /// Reads the product info from the sensor.
    /// # Returns
    /// `Ok(product_info)` if succeeds, `Err(reason)` if failed.
    pub fn receive_product_info(&mut self) -> Result<String, Error> {
        // Buffer may not be empty due to request_next_wrench() or its response.
        self.port
            .clear(serialport::ClearBuffer::All)
            .map_err(Error::SerialPort)?;

        // Send and wait.
        self.port.write_all(&['V' as u8]).map_err(Error::IO)?;
        std::thread::sleep(self.port.timeout());

        // The response may be non-fixed size.
        let bytes = self.port.bytes_to_read().map_err(Error::SerialPort)?;
        let mut res = vec![0; bytes as usize];

        let info = match self.port.read(&mut res) {
            Ok(_) => match std::str::from_utf8(&res) {
                Ok(str) => Ok(str.to_owned()),
                Err(_) => Err(Error::Utf8(res)),
            },
            Err(e) => Err(Error::IO(e)),
        };

        // Restart sensing wrenches.
        self.request_next_wrench()?;

        info
    }

    /// Returns the reference to the serial port for the sensor.
    pub fn inner_port(&self) -> &Box<dyn SerialPort> {
        &self.port
    }

    /// Request single wrench.
    fn request_next_wrench(&mut self) -> Result<(), Error> {
        self.port.write_all(&['R' as u8]).map_err(Error::IO)
    }
}

/// Represents an error occurred while communicating sensors.
#[derive(Debug)]
pub enum Error {
    /// Failed to open the port for the sensor.
    SerialPort(serialport::Error),
    /// Failed to read or write data during communication.
    IO(std::io::Error),
    /// The reception from the sensor cannot be interpleted with UTF-8 format.
    Utf8(Vec<u8>),
    /// The reception format from the sensor is different from the expected one.
    ParseResponse(String),
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Error::SerialPort(e) => write!(f, "SerialPort: {}", e),
            Error::IO(e) => write!(f, "IO: {}", e),
            Error::Utf8(v) => write!(
                f,
                "The response from the sensor is invalid for utf8. Raw response: {:X?}",
                v
            ),
            Error::ParseResponse(res) => write!(
                f,
                "Failed to parse the response from the sensor. The response: {}",
                res
            ),
        }
    }
}

impl std::error::Error for Error {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Error::SerialPort(e) => Some(e),
            Error::IO(e) => Some(e),
            _ => None,
        }
    }
}

/// A pair of force and torque.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Wrench {
    /// 3-dimensional force in Newton.
    pub force: Triplet<f64>,
    /// 3-dimensional torque in NewtonMeter.
    pub torque: Triplet<f64>,
}

impl Wrench {
    /// Returns a new wrench.
    pub fn new(force: Triplet<f64>, torque: Triplet<f64>) -> Wrench {
        Self { force, torque }
    }

    /// Returns a new wrench, initializing it to 0 Newton and 0 NewtonMeter.
    pub fn zeroed() -> Wrench {
        Wrench::new(Triplet::default(), Triplet::default())
    }
}

/// How much the digital value from the sensor increses per 1 Newton (for force) and per 1 NewtonMeter (for torque).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensitivity {
    /// How much the digital value from the sensor increses per 1 Newton.
    digital_per_newton: Triplet<f64>,
    /// How much the digital value from the sensor increses per 1 NewtonMeter.
    digital_per_newtonmeter: Triplet<f64>,
}

impl Sensitivity {
    /// Initialize a new sensitivity of a sensor.
    /// # Params
    /// 1. `digital_per_newton` How much the digital value from the sensor increses per 1 Newton.
    /// 1. `digital_per_newtonmeter` How much the digital value from the sensor increses per 1 NewtonMeter.
    pub fn new(
        digital_per_newton: Triplet<f64>,
        digital_per_newtonmeter: Triplet<f64>,
    ) -> Sensitivity {
        Self {
            digital_per_newton,
            digital_per_newtonmeter,
        }
    }
}
