//! Device driver for Wacoh-tech force-torque sensor.

use itertools::Itertools;
pub use pair_macro::Triplet;
pub use serialport;
use serialport::{DataBits, FlowControl, Parity, SerialPort, StopBits};
use std::borrow::Cow;
use std::fmt::{self, Display, Formatter};
use std::marker::PhantomData;
use std::str::FromStr;
use std::time::Duration;

pub struct Calibrated;

pub struct NotCalibratedYet;

const WRENCH_RESPONSE_LENGTH: usize = 27;

pub struct DynpickSensorBuilder<C> {
    /// Serial port device.
    port: Box<dyn SerialPort>,
    /// How much the digital value from the sensor increses per 1 Newton (for force) and per 1 NewtonMeter (for torque).
    sensitivity: Sensitivity,
    /// 👻
    _calibrated: PhantomData<fn() -> C>,
}

impl DynpickSensorBuilder<NotCalibratedYet> {
    /// Connects to the dynpick force torque sensor.
    /// # Params
    /// 1. `path` The sensor's path.
    ///
    /// # Returns
    /// `Ok(builder)` if successfully connected, `Err(reason)` if failed.
    pub fn open<'a>(
        path: impl Into<Cow<'a, str>>,
    ) -> Result<DynpickSensorBuilder<NotCalibratedYet>, Error> {
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

    /// Calibrate the sensor by using the specified sensitivity.
    pub fn calibrate_manually(self, sensitivity: Sensitivity) -> DynpickSensorBuilder<Calibrated> {
        DynpickSensorBuilder {
            port: self.port,
            sensitivity,
            _calibrated: PhantomData,
        }
    }

    /// Calibrate the sensor, reading its sensitivity.
    pub fn calibrate_by_embedded_data(mut self) -> Result<DynpickSensorBuilder<Calibrated>, Error> {
        const RES_LEN: usize = 46;

        self.port.write_all(&['p' as u8]).map_err(Error::IO)?;

        std::thread::sleep(self.port.timeout());

        let mut res = [0; RES_LEN];
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

        Ok(self.calibrate_manually(sensitivity))
    }
}

impl DynpickSensorBuilder<Calibrated> {
    pub fn build(self) -> Result<DynpickSensor, Error> {
        let mut sensor = DynpickSensor {
            port: self.port,
            last_wrench: Wrench::zeroed(),
            sensitivity: self.sensitivity,
        };
        sensor.request_next_wrench()?;

        Ok(sensor)
    }
}

/// Dynpick 6-axis force-torque sensor.
pub struct DynpickSensor {
    /// Serial port device.
    port: Box<dyn SerialPort>,
    /// The latest wrench acquired by `update_wrench`.
    last_wrench: Wrench,
    /// How much the digital value from the sensor increses per 1 Newton (for force) and per 1 NewtonMeter (for torque).
    sensitivity: Sensitivity,
}

impl DynpickSensor {
    /// Returns the latest wrench without connecting the sensor.
    ///
    /// Use `update_wrench()` to obtain a new wrench from the sensor.
    /// `Ok(sensor)` if successfully connected, `Err(reason)` if failed.
    pub fn last_wrench(&self) -> Wrench {
        self.last_wrench
    }

    /// Updates the latest wrench, communicating to the sensor.
    /// # Returns
    /// `Ok(wrench)` if succeeds, `Err(reason)` if failed.
    pub fn update(&mut self) -> Result<Wrench, Error> {
        let mut res = [0; WRENCH_RESPONSE_LENGTH];
        self.port.read_exact(&mut res).map_err(Error::IO)?;

        self.last_wrench = self.to_wrench(&res)?;

        //
        self.request_next_wrench()?;

        Ok(self.last_wrench)
    }

    pub fn zeroed_next(&mut self) -> Result<(), Error> {
        self.port.write_all(&['O' as u8]).map_err(Error::IO)
    }

    pub fn receive_product_info(&mut self) -> Result<String, Error> {
        self.port
            .clear(serialport::ClearBuffer::All)
            .map_err(Error::SerialPort)?;

        self.port.write_all(&['V' as u8]).map_err(Error::IO)?;

        std::thread::sleep(self.port.timeout());

        let bytes = self.port.bytes_to_read().map_err(Error::SerialPort)?;
        let mut res = vec![0; bytes as usize];

        let info = match self.port.read(&mut res) {
            Ok(_) => String::from_utf8(res.clone()).or(Err(Error::Utf8(res.to_vec()))),
            Err(e) => Err(Error::IO(e)),
        };

        self.request_next_wrench()?;

        info
    }

    /// Returns the reference to the serial port for the sensor.
    pub fn inner_port(&self) -> &Box<dyn SerialPort> {
        &self.port
    }

    fn request_next_wrench(&mut self) -> Result<(), Error> {
        self.port.write_all(&['R' as u8]).map_err(Error::IO)
    }

    fn to_wrench(&self, buf: &[u8; WRENCH_RESPONSE_LENGTH]) -> Result<Wrench, Error> {
        let res = std::str::from_utf8(buf).or(Err(Error::Utf8(buf.to_vec())))?;

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

        Ok(Wrench::new(force, torque))
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
            Error::Utf8(v) => write!(f, "The response is invalid for utf8: {:?}", v),
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

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sensitivity {
    digital_per_newton: Triplet<f64>,
    digital_per_newtonmeter: Triplet<f64>,
}

impl Sensitivity {
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
