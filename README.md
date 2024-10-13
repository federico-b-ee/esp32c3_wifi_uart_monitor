<h1 align="center">ESP32C3 WiFi UART Monitor</h1>

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Resources](#resources)


This project demonstrates how to use the ESP32-C3 microcontroller to monitor UART data over WiFi.

- BPS is set to `115200`.
- 1 Stop Bit.
- No parity.

>[!NOTE]
> The project was generated using the [esp-rs template](https://docs.esp-rs.org/book/writing-your-own-application/generate-project/index.html?highlight=dev%20container).

## Features

- **WiFi Connectivity**: Connects to a WiFi network to transmit UART data.
- **UART Monitoring**: Reads data from UART and sends it over WiFi.
- **Web Interface**: Simple web interface to view UART data in real-time.

## Requirements

- Rust
- ESP32-C3 microcontroller
- The RX pin is GPIO 20.
    - Connect GND and the RX pin to the TX pin coming from the source to be monitored.

## Installation

1. Clone the repository:
2. Navigate to the project directory:
    ```sh
    cd esp32c3_wifi_uart_monitor
    ```
3. Build and upload the firmware to your ESP32-C3:
   1. Using [espflash](https://docs.esp-rs.org/book/tooling/espflash.html).
    ```sh
    cargo install cargo-espflash
    ```
   2. With `cargo run --release` it should build and upload the firmware.
   3. If `cargo run --release` doesn't work, the following command can be run:
    ```sh
    cargo espflash flash --monitor --release
    ```

## Usage

1. Connect the ESP32-C3 to your UART device.
2. Connect your SmartPhone to the access_point `monitor_wifi` (by default), you may need to assignate a static IP to your phone:
   1. `static_ip`: `192.168.2.2`..`192.168.2.255`
   2. `gateway`: `192.168.2.1`
3. Open a web browser and navigate to the IP address assigned to the ESP32-C3 and its port(`http://192.168.2.1:8080`).

## Resources

- [The Rust on ESP Book](https://docs.esp-rs.org/book/introduction.html)
- [GitHub - esp-rs/esp-hal: no_std Hardware Abstraction Layers for ESP32 microcontrollers](https://github.com/esp-rs/esp-hal)
- [esp-hal/examples - wifi_access_point.rs](https://github.com/esp-rs/esp-hal/blob/main/examples/src/bin/wifi_access_point.rs)
