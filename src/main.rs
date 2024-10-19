#![no_std]
#![no_main]

use core::cell::RefCell;
use embedded_io::*;
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{delay::Delay, peripherals::UART1, prelude::*, Blocking};

// Print over USBs
use esp_println::println;

// UART
use critical_section::Mutex;
use esp_hal::{
    gpio::{Io, Level, Output},
    uart::{self, Uart},
};

mod uart_utils;
use uart_utils::UARTBuffer;

static SERIAL: Mutex<RefCell<Option<Uart<UART1, Blocking>>>> = Mutex::new(RefCell::new(None));
static UART_CIRC_BUFFER: Mutex<RefCell<UARTBuffer>> = Mutex::new(RefCell::new(UARTBuffer {
    data: [0; 512],
    head: 0,
    tail: 0,
}));

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

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    // UART with Interrupts
    // TXD: GPIO21, RXD: GPIO20
    let rx = io.pins.gpio20;
    let tx = io.pins.gpio21;
    let config = uart::config::Config::default()
        .rx_fifo_full_threshold(1)
        .baudrate(115200)
        .parity_none()
        .stop_bits(uart::config::StopBits::STOP1);
    let mut uart = Uart::new_with_config(peripherals.UART1, config, rx, tx).unwrap();
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

    // UART buffer
    let mut lines: heapless::Vec<heapless::String<64>, 32> = heapless::Vec::new();

    let delay = Delay::new();
    loop {
        led.toggle();

        // Write data to the uart
        critical_section::with(|cs| {
            println!("hello");
            let mut serial = SERIAL.borrow_ref_mut(cs);
            let serial = serial.as_mut().unwrap();

            writeln!(serial, "Hello from ESP32!").unwrap();
        });

        delay.delay_millis(1000);

        // Parse the received data
        critical_section::with(|cs| {
            let buffer = UART_CIRC_BUFFER.borrow_ref_mut(cs);
            lines = buffer.lines();
            println!("lines: {:?}", lines);
        });
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
