# dynpick-force-torque-sensor
[![crates.io](https://img.shields.io/crates/v/dynpick-force-torque-sensor-rs.svg)](https://crates.io/crates/dynpick-force-torque-sensor)
[![Build Status](https://travis-ci.org/Amelia10007/dynpick-force-torque-sensor-rs.svg?branch=master)](https://travis-ci.org/Amelia10007/dynpick-force-torque-sensor-rs)

Device driver for [Wacoh-tech force-torque sensor](https://wacoh-tech.com/en/products/dynpick/) written in pure Rust.

Inspired the works of [Tokyo Opensource Robotics Kyokai Association](https://github.com/tork-a/dynpick_driver).

# Example
```rust
use dynpick_force_torque_sensor::DynpickSensor;

let mut sensor = DynpickSensor::open("/dev/ttyUSB0").unwrap();

let wrench = sensor.update_wrench().unwrap();

println!("Wrench: {:?}", wrench);
```

# Setup
It may be required to customize udev rules if you use usb-connected sensors.

[This shell script](./examples/setup_udev_rule.sh) can be useful for customize (see the file in detail).

# Demo
## Demo for an usb-connected sensor
1. Setup udev rule by using [this shell script](./examples/setup_udev_rule.sh).
1. Connect your sensor.
1. run example by ```cargo run --example demo```

# Note
We tested this crate only by WDF-6M200-3 sensor because we have no other dynpick sensor.
