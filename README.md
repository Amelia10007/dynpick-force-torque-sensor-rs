# dynpick-force-torque-sensor
[![crates.io](https://img.shields.io/crates/v/dynpick-force-torque-sensor.svg)](https://crates.io/crates/dynpick-force-torque-sensor)
[![Build Status](https://travis-ci.org/Amelia10007/dynpick-force-torque-sensor-rs.svg?branch=master)](https://travis-ci.org/Amelia10007/dynpick-force-torque-sensor-rs)

Unofficial device driver for [Wacoh-tech force-torque sensor](https://wacoh-tech.com/en/products/dynpick/) written in pure Rust.

Inspired the works of [Tokyo Opensource Robotics Kyokai Association](https://github.com/tork-a/dynpick_driver) (It is written in C/C++ and for ROS).

# Example
```rust
use dynpick_force_torque_sensor::DynpickSensorBuilder;

let mut sensor = DynpickSensorBuilder::open("/dev/ttyUSB0")
    .and_then(|b| b.set_sensitivity_by_builtin_data())
    .and_then(|b| b.build())
    .unwrap();

sensor.zeroed_next().unwrap(); // Calibration

let wrench = sensor.update_wrench().unwrap();
println!("Force: {}, Torque: {}", wrench.force, wrench.torque);
```

# Dependency under Linux environment
`libudev-dev` is required under Linux environment. Please install it by  
`$ sudo apt install libudev-dev`

# Setup
It may be required to customize udev rules.

[This shell script](./examples/setup_udev_rule.sh) can be useful for customize (see the file in detail).

# Run a demo for an usb-connected sensor
1. Clone this repository.
1. Setup udev rule by using [this shell script](./examples/setup_udev_rule.sh).
1. Connect your sensor.
1. Run the example by ```cargo run --example demo```

# License
MIT

# Note
I tested this crate only by WDF-6M200-3 sensor because I have no other dynpick sensor.
