//! Device driver for Wacoh-tech force-torque sensor.

use itertools::Itertools;
pub use pair_macro::Triplet;
pub use serialport;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::borrow::Cow;
use std::fmt::{self, Display, Formatter};
use std::str::FromStr;
use std::time::Duration;

/// Dynpick 6-axis force-torque sensor.
pub struct DynpickSensor {
    /// Serial port device.
    port: Box<dyn SerialPort>,
    /// The latest wrench acquired by `update_wrench`.
    last_wrench: Wrench,
    /// How much the digital value from the sensor increses per 1 Newton (for force) and per 1 NewtonMeter (for torque).
    sensitivity: Wrench,
}

impl DynpickSensor {
    /// Connects to the dynpick force torque sensor.
    /// # Params
    /// 1. `path` The sensor's path.
    ///
    /// # Returns
    /// `Ok(sensor)` if successfully connected, `Err(reason)` if failed.
    ///
    /// # Examples
    /// ```no_run
    /// use dynpick_force_torque_sensor::DynpickSensor;
    ///
    /// let mut sensor = DynpickSensor::open("/dev/ttyUSB0").unwrap();
    ///
    /// let wrench = sensor.update_wrench().unwrap();
    /// ```
    pub fn open<'a>(path: impl Into<Cow<'a, str>>) -> Result<DynpickSensor, Error> {
        let mut port = serialport::new(path, 921600)
            .data_bits(DataBits::Eight)
            .flow_control(FlowControl::None)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_millis(1))
            .open()
            .map_err(Error::SerialPort)?;

        let sensitivity = read_sensitivity(&mut port)?;

        let sensor = Self {
            port,
            last_wrench: Wrench::zeroed(),
            sensitivity,
        };

        Ok(sensor)
    }

    /// Returns the latest wrench without connecting the sensor.
    ///
    /// Use `update_wrench()` to obtain a new wrench from the sensor.
    pub fn last_wrench(&self) -> Wrench {
        self.last_wrench
    }

    /// Updating the latest wrench, communicating to the sensor.
    /// # Returns
    /// `Ok(wrench)` if succeeds, `Err(reason)` if failed.
    pub fn update_wrench(&mut self) -> Result<Wrench, Error> {
        const RES_LEN: usize = 27;

        self.port.write_all(&['R' as u8]).map_err(Error::IO)?;

        let mut res = [0; RES_LEN];
        self.port.read_exact(&mut res).map_err(Error::IO)?;

        let res = std::str::from_utf8(&res).or(Err(Error::Utf8))?;
        if &res[res.len() - 2..] != "\n\r" {
            return Err(Error::ParseResponse(res.to_owned()));
        }

        let (fx, fy, fz, mx, my, mz) = (0..6)
            .map(|i| 1 + i * 4)
            .map(|start| &res[start..start + 4])
            .map(|src| u16::from_str_radix(src, 16))
            .filter_map(Result::ok)
            .next_tuple()
            .ok_or(Error::ParseResponse(res.to_owned()))?;

        let raw_force = Triplet::new(fx, fy, fz);
        let raw_torque = Triplet::new(mx, my, mz);

        let force =
            raw_force.map_entrywise(self.sensitivity.force, |raw, s| ((raw - 8192) as f64) / s);
        let torque =
            raw_torque.map_entrywise(self.sensitivity.torque, |raw, s| ((raw - 8192) as f64) / s);

        self.last_wrench = Wrench::new(force, torque);

        Ok(self.last_wrench)
    }

    /// Attempts to set offset.
    /// The current corrected wrench will be zeroed.
    /// # Returns
    /// `Ok(())` if succeeds, `Err(reason)` if failed.
    pub fn offset(&mut self) -> Result<(), Error> {
        // It may only work if this command sent twice or more.
        self.port.write_all(&['O' as u8]).map_err(Error::IO)
    }

    /// Attempts to set the embedded frequency driver filter.
    /// # Params
    /// 1. `filter`
    ///
    /// # Returns
    /// `Ok(())` if succeeds, `Err(reason)` if failed.
    pub fn set_frequency_divider_filter(&mut self, filter: u8) -> Result<(), Error> {
        assert!(matches!(filter, 1 | 2 | 4 | 8));

        let command = format!("{}F", filter);
        self.port.write_all(command.as_bytes()).map_err(Error::IO)?;

        let mut res = [0; 3];
        self.port.read_exact(&mut res).map_err(Error::IO)?;

        let res = std::str::from_utf8(&res).or(Err(Error::Utf8))?;
        let set_filter = u8::from_str(&res[0..1]).or(Err(Error::Utf8))?;

        if filter == set_filter {
            Ok(())
        } else {
            Err(Error::ParseResponse(String::from("Failed to set filter")))
        }
    }

    /// Returns the reference to the serial port for the sensor.
    pub fn inner_port(&self) -> &Box<dyn SerialPort> {
        &self.port
    }
}

/// Read sensitivity of the force torque sensor.
fn read_sensitivity(port: &mut Box<dyn SerialPort>) -> Result<Wrench, Error> {
    const RES_LEN: usize = 46;

    port.write_all(&['p' as u8]).map_err(Error::IO)?;

    let mut res = [0; RES_LEN];
    port.read_exact(&mut res).map_err(Error::IO)?;

    let res = std::str::from_utf8(&res).or(Err(Error::Utf8))?;
    let (fx, fy, fz, mx, my, mz) = res
        .split(',')
        .map(f64::from_str)
        .filter_map(Result::ok)
        .next_tuple()
        .ok_or(Error::ParseResponse(res.to_owned()))?;

    let force = Triplet::new(fx, fy, fz);
    let torque = Triplet::new(mx, my, mz);

    Ok(Wrench::new(force, torque))
}

/// Represents an error occurred while communicating sensors.
#[derive(Debug)]
pub enum Error {
    /// Failed to open the port for the sensor.
    SerialPort(serialport::Error),
    /// Failed to read or write data during communication.
    IO(std::io::Error),
    /// The size of sent data to the sensor was different from the expected one.
    WrittenLength(usize, usize),
    /// The size of received data from the sensor was different from the expected one.
    ReadLength(usize, usize),
    /// The reception from the sensor cannot be interpleted with UTF-8 format.
    Utf8,
    /// The reception format from the sensor is different from the expected one.
    ParseResponse(String),
}

impl Display for Error {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Error::SerialPort(e) => write!(f, "SerialPort: {}", e),
            Error::IO(e) => write!(f, "IO: {}", e),
            Error::WrittenLength(expected, actual) => write!(
                f,
                "{} bytes data should have been sent to the sensor, but actually {} bytes were.",
                expected, actual
            ),
            Error::ReadLength(expected, actual) => write!(
                f,
                "{} bytes data should have been sent from the sensor, but actually {} bytes were.",
                expected, actual
            ),
            Error::Utf8 => write!(f, "The response s invalid for utf8."),
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
