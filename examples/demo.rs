use dynpick_force_torque_sensor::serialport;
use dynpick_force_torque_sensor::{DynpickSensorBuilder, Sensitivity, Triplet};

fn search_usb_sensor_path() -> Result<Option<String>, serialport::Error> {
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
                usb.vid == vendor_id && usb.pid == product_id
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
    let path = match search_usb_sensor_path() {
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

    // Specify the sensitivity manually.
    let sensitivity = {
        let force = Triplet::new(24.9, 24.6, 24.5);
        let torque = Triplet::new(1664.7, 1639.7, 1638.0);
        Sensitivity::new(force, torque)
    };

    // Connect the found sensor.
    let sensor = DynpickSensorBuilder::open(path)
        .map(|b| b.set_sensitivity_manually(sensitivity))
        .and_then(|b| b.build());
    let mut sensor = match sensor {
        Ok(s) => s,
        Err(e) => {
            println!("{}", e);
            return;
        }
    };
    println!("Successfully opened the sensor.");

    // Correct zero-point
    match sensor.zeroed_next() {
        Ok(_) => println!("Offset the sensor."),
        Err(e) => {
            println!("An error occurred during offset: {}", e);
            return;
        }
    }

    // Repeatedly receive wrenches from the sensor.
    let measurement_count = 1000;
    for i in 0..measurement_count {
        std::thread::sleep(sensor.inner_port().timeout());

        match sensor.update() {
            Ok(w) => println!("[{}/{}] {:?}", i + 1, measurement_count, w),
            Err(e) => println!("[{}/{}] {}", i + 1, measurement_count, e),
        }
    }

    // Info
    println!("Product info: {:?}", sensor.receive_product_info());

    println!("dynpick-force-torque-sensor demo finished.");
}
