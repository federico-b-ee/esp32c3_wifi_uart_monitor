#![no_std]
#![no_main]

use core::{cell::RefCell, str::FromStr};
use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{peripherals::UART1, prelude::*, rng::Rng, time, timer::timg::TimerGroup, Blocking};

// Print over USBs
use esp_println::println;

// WIFI stack
use esp_wifi::{
    init,
    wifi::{
        utils::create_network_interface, AccessPointConfiguration, Configuration, WifiApDevice,
    },
    wifi_interface::WifiStack,
    EspWifiInitFor,
};
use smoltcp::iface::SocketStorage;

// UART
use critical_section::Mutex;
use esp_hal::{
    gpio::{Io, Level, Output},
    uart::{self, Uart},
};

mod uart_utils;
use uart_utils::UARTBuffer;

const SSID: &str = "monitor_wifi";

static SERIAL: Mutex<RefCell<Option<Uart<UART1, Blocking>>>> = Mutex::new(RefCell::new(None));
static UART_CIRC_BUFFER: Mutex<RefCell<UARTBuffer>> = Mutex::new(RefCell::new(UARTBuffer {
    data: [0; 512],
    head: 0,
    tail: 0,
}));

// HTML
const RESPONSE_HEADER: &[u8] = b"HTTP/1.0 200 OK\r\n\r\n";
const HTML_START: &[u8] = b"<html><body>";
const TITLE: &[u8] = b"<h1>WIFI UART MONITOR!</h1>";
const CLEAR_BUTTON: &[u8] = b"<form action=\"\\clear_buffer\" method=\"post\"><button type=\"submit\">Clear Buffer!</button></form>";
const NOTE_1: &[u8] = b"<h2>bps: 115200 || No Parity || 1 STOP bit || RX: PIN 20</h2>";
const NOTE_2: &[u8] = b"<h2>Usage:</h2>";
const NOTE_3: &[u8] = b"<h2>- It reads strings that end with a newline character.</h2>";
const NOTE_4: &[u8] =
    b"<h2>- Strings must be a maximum of 64 bytes, including the newline character.</h2>";
const UART_DATA_HEADER: &[u8] = b"<h2>UART data:</h2>";
const HTML_END: &[u8] = b"</body></html>\r\n";

#[entry]
fn main() -> ! {
    esp_println::logger::init_logger_from_env();

    esp_alloc::heap_allocator!(72 * 1024);

    // Initialize peripherals
    let peripherals = esp_hal::init({
        let mut config = esp_hal::Config::default();
        // Set the CPU clock to 80MHz should lower the power consumption, thus the temperature.
        config.cpu_clock = CpuClock::Clock80MHz;
        config
    });

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let init = init(
        EspWifiInitFor::Wifi,
        timg0.timer0,
        Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // UART with Interrupts
    // TXD: GPIO21, RXD: GPIO20
    let config = uart::config::Config::default()
        .rx_fifo_full_threshold(1)
        .baudrate(115200)
        .parity_none()
        .stop_bits(uart::config::StopBits::STOP1);
    let mut uart =
        Uart::new_with_config(peripherals.UART1, config, io.pins.gpio21, io.pins.gpio20).unwrap();
    uart.set_interrupt_handler(interrupt_handler);

    critical_section::with(|cs| {
        // Set if the UART should generate an interrupt when the RX FIFO
        // receives a certain byte. In this case, b'\n'.
        // The interrupt has to be reset manually in the interrupt handler.
        //uart.set_at_cmd(AtCmdConfig::new(None, None, None, b'\n', None));
        //uart.listen_at_cmd();
        uart.listen_rx_fifo_full();

        SERIAL.borrow_ref_mut(cs).replace(uart);
    });

    // LED
    let mut led = Output::new(io.pins.gpio8, Level::High);

    // WIFI
    let mut wifi = peripherals.WIFI;
    let mut socket_set_entries: [SocketStorage; 3] = Default::default();
    let (iface, device, mut controller, sockets) =
        create_network_interface(&init, &mut wifi, WifiApDevice, &mut socket_set_entries).unwrap();

    let now = || time::now().duration_since_epoch().to_millis();
    let mut wifi_stack = WifiStack::new(iface, device, sockets, now);

    let wifi_config = Configuration::AccessPoint(AccessPointConfiguration {
        ssid: heapless::String::from_str(SSID).unwrap(),
        // Can't set password for AP mode, not supported at the moment.
        // https://github.com/esp-rs/esp-hal/blob/main/esp-wifi/README.md#missing--to-be-done
        //password: heapless::String::from_str(PASSWORD).unwrap(),
        ..Default::default()
    });

    match controller.set_configuration(&wifi_config) {
        Ok(_) => println!("Wi-Fi configuration set successfully."),
        Err(e) => println!("Failed to set Wi-Fi configuration: {:?}", e),
    }

    controller.start().unwrap();

    wifi_stack
        .set_iface_configuration(&esp_wifi::wifi::ipv4::Configuration::Client(
            esp_wifi::wifi::ipv4::ClientConfiguration::Fixed(
                esp_wifi::wifi::ipv4::ClientSettings {
                    ip: esp_wifi::wifi::ipv4::Ipv4Addr::from_str("192.168.2.1").unwrap(),
                    subnet: esp_wifi::wifi::ipv4::Subnet {
                        gateway: esp_wifi::wifi::ipv4::Ipv4Addr::from_str("192.168.2.1").unwrap(),
                        mask: esp_wifi::wifi::ipv4::Mask(24),
                    },
                    dns: None,
                    secondary_dns: None,
                },
            ),
        ))
        .unwrap();

    println!(
        "Connect to the AP `{}` and point your browser to http://192.168.2.1:8080/",
        SSID
    );
    println!("You may need to set a static IP in the range of 192.168.2.2 .. 192.168.2.255, use gateway 192.168.2.1");

    let mut rx_buffer = [0u8; 2048];
    let mut tx_buffer = [0u8; 1024];
    let mut socket = wifi_stack.get_socket(&mut rx_buffer, &mut tx_buffer);

    // UART buffer
    let mut lines: heapless::Vec<heapless::String<64>, 32> = heapless::Vec::new();

    loop {
        led.toggle();

        // Parse the received data
        critical_section::with(|cs| {
            let buffer = UART_CIRC_BUFFER.borrow_ref_mut(cs);
            lines = buffer.lines();
        });

        socket.work();

        if !socket.is_open() {
            socket.listen(8080).unwrap();
        }

        if socket.is_connected() {
            let mut buffer = [0u8; 1024];
            if let Ok(len) = socket.read(&mut buffer) {
                let response = unsafe { core::str::from_utf8_unchecked(&buffer[..len]) };

                println!("{}", response);
                if response.contains("POST /clear_buffer") {
                    println!("Clearing buffer!");
                    critical_section::with(|cs| {
                        let mut buffer = UART_CIRC_BUFFER.borrow_ref_mut(cs);
                        buffer.clear_buffer();
                    });

                    // After clearing the buffer, send a redirect response
                    socket.write_all(b"HTTP/1.1 303 See Other\r\n").unwrap();
                    socket.write_all(b"Location: /\r\n").unwrap();
                    socket.write_all(b"\r\n").unwrap();
                    socket.flush().unwrap();
                } else if response.contains("GET /") {
                    println!("Sending HTML!");
                    // Prepare the HTTP response components
                    // Write the response to the socket
                    socket.write_all(RESPONSE_HEADER).unwrap();
                    socket.write_all(HTML_START).unwrap();
                    socket.write_all(TITLE).unwrap();
                    socket.write_all(CLEAR_BUTTON).unwrap();
                    socket.write_all(NOTE_1).unwrap();
                    socket.write_all(NOTE_2).unwrap();
                    socket.write_all(NOTE_3).unwrap();
                    socket.write_all(NOTE_4).unwrap();
                    socket.write_all(UART_DATA_HEADER).unwrap();

                    for line in lines.clone().into_iter() {
                        socket.write_all(b"<p>").unwrap();
                        socket.write_fmt(format_args!("{}", line)).unwrap();
                        socket.write_all(b"</p>").unwrap();
                    }

                    socket.write_all(HTML_END).unwrap();
                    socket.flush().unwrap();
                }
            }
        }

        socket.close();
        socket.work();
    }
}

#[handler]
fn interrupt_handler() {
    critical_section::with(|cs| {
        let mut serial = SERIAL.borrow_ref_mut(cs);
        let serial = serial.as_mut().unwrap();

        let mut buffer = UART_CIRC_BUFFER.borrow_ref_mut(cs);
        buffer.push(serial.read_byte().unwrap());

        serial.reset_rx_fifo_full_interrupt();
    });
}
