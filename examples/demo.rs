use dynpick_force_torque_sensor::serialport;
use dynpick_force_torque_sensor::DynpickSensor;

fn search_usb_sensor_paths() -> Result<Option<String>, serialport::Error> {
    // Wacoh-tech vendor ID.
    let vendor_id = 0x10c4;
    // NOTE: The following product-ID may be specific for WDF-6M200-3.
    let product_id = 0xea60;

    let ports = serialport::available_ports()?;
    let path = ports
        .into_iter()
        .filter(move |port| match &port.port_type {
            // Takes only USB-connected device
            serialport::SerialPortType::UsbPort(usb) => {
                usb.vid == product_id && usb.vid == vendor_id
            }
            _ => false,
        })
        .map(|sensor_port| sensor_port.port_name)
        .next();

    Ok(path)
}

fn main() {
    println!("dynpick-force-torque-sensor demo started.");
    println!("Make sure that the sensor is connected to the computer.");
    println!("Make sure setting udev rule. See examples/setup_udev_rule.sh in detail.");

    // Search USB-connected dynpick sensor.
    let path = match search_usb_sensor_paths() {
        Ok(Some(path)) => path,
        Ok(None) => {
            println!("No dynpick sensor is connected.");
            return;
        }
        Err(e) => {
            println!("{}", e);
            return;
        }
    };
    println!("Found a sensor. Path: {}", path);

    // Connect the found sensor.
    let mut sensor = match DynpickSensor::open(path) {
        Ok(sensor) => sensor,
        Err(e) => {
            println!("{}", e);
            return;
        }
    };
    println!("Successfully opened the sensor.");

    // Correct zero-point
    match sensor.offset() {
        Ok(_) => println!("Offset the sensor."),
        Err(e) => {
            println!("An error occurred during offset: {}", e);
            return;
        }
    }

    // Repeatedly receive wrenches from the sensor.
    let measurement_count = 100;
    for i in 0..measurement_count {
        match sensor.update_wrench() {
            Ok(w) => println!("[{}/{}] wrench: {:?}", i + 1, measurement_count, w),
            Err(e) => println!("[{}/{}] {}", i + 1, measurement_count, e),
        }
    }

    println!("dynpick-force-torque-sensor demo finished.");
}
