//! SPI slave device implementation using DMA.
//!
//! Hardware connections for ESP32 SPI slave:
//! - SCLK => GPIO10
//! - MISO => GPIO11
//! - MOSI => GPIO12
//! - CS   => GPIO13
//!
//! This example receives data via SPI in slave mode and processes it in a background task using async.
//! The circular buffer allows handling (processing) data at different rates than it's received.

//% FEATURES: esp-hal/unstable

#![no_std]
#![no_main]

use core::cell::RefCell;
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::{
    dma_buffers,
    gpio::{Event, Input, InputConfig, Io, Level, Output, OutputConfig, Pull},
    handler,
    peripheral::Peripheral,
    ram, spi,
    timer::timg::TimerGroup,
};
use esp_println::println;

use circular_buffer::CircularBuffer;

// Constants for buffer sizing
const CHUNK_SIZE: usize = 512 * 4;
const DMA_BUFFER_SIZE: usize = CHUNK_SIZE; // Must be multiple of 4 bytes -> make it the same size or bigger then the CHUNK_SIZE in the master esp32
const CIRCULAR_BUFFER_SIZE: usize = 40; // Number of chunks to store in the circular buffer

// Global synchronization primitives
static SPI_TRANSACTION_END: Watch<CriticalSectionRawMutex, bool, 6> = Watch::new(); // Transaction end notification -> you can change the number of max receivers if needed
static DATA_UPDATED_ON_RINGBUFF: Watch<CriticalSectionRawMutex, bool, 6> = Watch::new(); // Data ready in circular buffer notification -> you can change the number of max receivers if needed
static CS_PIN: critical_section::Mutex<RefCell<Option<Input>>> =
    critical_section::Mutex::new(RefCell::new(None));

// Thread-safe circular buffer for received data
static CIRCULAR_BUFFER: critical_section::Mutex<
    RefCell<Option<CircularBuffer<CIRCULAR_BUFFER_SIZE, [u8; CHUNK_SIZE]>>>,
> = critical_section::Mutex::new(RefCell::new(None));

// Buffer management functions
fn init_circular_buffer() {
    critical_section::with(|cs| {
        CIRCULAR_BUFFER
            .borrow_ref_mut(cs)
            .replace(CircularBuffer::<CIRCULAR_BUFFER_SIZE, [u8; CHUNK_SIZE]>::new());
    });
}

fn push_to_buffer(value: [u8; CHUNK_SIZE]) -> Result<(), ()> {
    critical_section::with(|cs| {
        if let Some(buf) = CIRCULAR_BUFFER.borrow_ref_mut(cs).as_mut() {
            buf.push_front(value);
            Ok(())
        } else {
            Err(())
        }
    })
}

fn pop_latest_from_buffer() -> Option<[u8; CHUNK_SIZE]> {
    critical_section::with(|cs| {
        CIRCULAR_BUFFER
            .borrow_ref_mut(cs)
            .as_mut()
            .and_then(|buf| buf.pop_front())
    })
}

fn pop_oldest_from_buffer() -> Option<[u8; CHUNK_SIZE]> {
    critical_section::with(|cs| {
        CIRCULAR_BUFFER
            .borrow_ref_mut(cs)
            .as_mut()
            .and_then(|buf| buf.pop_back())
    })
}

fn is_buffer_empty() -> bool {
    critical_section::with(|cs| {
        CIRCULAR_BUFFER
            .borrow_ref_mut(cs)
            .as_mut()
            .map_or(true, |buf| buf.is_empty())
    })
}

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Initialize system components
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Configure interrupt handling
    let mut io = Io::new(peripherals.IO_MUX);
    esp_alloc::heap_allocator!(size: 72 * 1024);
    io.set_interrupt_handler(handler);

    // Configure GPIO pins
    let config_in = InputConfig::default().with_pull(Pull::Up);
    let config_out = OutputConfig::default();

    // SPI slave pins
    let slave_sclk = Input::new(peripherals.GPIO10, config_in);
    let slave_miso = Output::new(peripherals.GPIO11, Level::Low, config_out);
    let slave_mosi = Input::new(peripherals.GPIO12, config_in);
    let mut slave_cs_clone = Input::new(
        unsafe { peripherals.GPIO13.clone_unchecked() }, // since we are gonna access it from the interrupt and inside critical section concurrent access is not possible
        config_in,
    );
    let slave_cs = Input::new(peripherals.GPIO13, config_in);

    // Set up CS interrupt to detect transaction end
    critical_section::with(|cs| {
        slave_cs_clone.listen(Event::RisingEdge);
        CS_PIN.borrow_ref_mut(cs).replace(slave_cs_clone);
    });

    // Initialize circular buffer
    init_circular_buffer();

    // Spawn SPI DMA task
    spawner
        .spawn(spi_slave_task(
            slave_cs,
            slave_sclk,
            slave_miso,
            slave_mosi,
            peripherals.DMA_CH0,
            peripherals.SPI2,
        ))
        .unwrap();
    // Spawn example data processing task
    spawner.spawn(data_processing_task()).unwrap();

    // Main loop - free for other application logic
    loop {
        println!("Main loop is running (system ready)");
        Timer::after(Duration::from_millis(1000)).await;
    }
}

// Data processing task -> Dummy example, right now it is just checking the data integrity and printing the data rate statistics
// This task runs in the background and processes data received via the CIRC buffer
#[embassy_executor::task]
async fn data_processing_task() {
    let mut data_notification = DATA_UPDATED_ON_RINGBUFF.receiver().unwrap();

    // Statistics tracking
    let mut total_data_recv = 0;
    let mut start_time = Instant::now();
    let mut everything_was_valid = true;

    loop {
        // Wait for new data notification
        data_notification.changed().await;

        // Process latest data
        let buffer = match pop_latest_from_buffer() {
            Some(data) => data,
            None => {
                println!("Error: Buffer empty despite data notification");
                continue;
            }
        };

        // Check data markers
        if buffer[2] != 0xFF {
            if buffer[2] == 0xED {
                // Statistics marker - display performance data
                let elapsed_sec = start_time.elapsed().as_millis() as f64 / 1000.0;
                let data_rate = total_data_recv as f64 / elapsed_sec;

                println!("Statistics:");
                println!("  Total data received: {} bytes", total_data_recv);
                println!("  Data rate: {:.2} bytes/second", data_rate);
                println!(
                    "  Data integrity: {}",
                    if everything_was_valid {
                        "Valid"
                    } else {
                        "Errors detected"
                    }
                );

                // Reset statistics
                start_time = Instant::now();
                total_data_recv = 0;
                everything_was_valid = true;
            } else {
                // Invalid data marker
                everything_was_valid = false;
            }
        }

        total_data_recv += buffer.len();

        // Debug data content (commented out for normal operation) -> at high data rates it is too much print output in the CMD
        /*
        if buffer.len() > 20 {
            println!(
                "Buffer data: {:x?} ... {:x?}",
                &buffer[..10],
                &buffer[buffer.len() - 10..],
            );
        } else {
            println!("Buffer data: {:x?}", &buffer[..]);
        }
        */
    }
}

// SPI slave task - handles SPI communication in the background
// uploads data to the circular buffer when received after the interrupt happens and the data is read in the dma buffer
// after the data is pushed to the circular buffer it sends a notification to the data processing task via the waker channel
// you could even make it so the data ready channel already send the CHUNK with it (then circ buffer not is really necessary), but this is up to implementation details
// To know if it the transaction is done we check the waker channel that is triggered by the CS pin rising edge Interrupt

#[embassy_executor::task]
async fn spi_slave_task(
    slave_cs: Input<'static>,
    slave_sclk: Input<'static>,
    slave_miso: Output<'static>,
    slave_mosi: Input<'static>,
    dma_channel: esp_hal::dma::DmaChannel0,
    spi_chan: esp_hal::peripherals::SPI2,
) {
    // Create DMA buffers for SPI transfers
    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE, 0);

    // Configure SPI in slave mode with DMA
    let mut spi = spi::slave::Spi::new(spi_chan, spi::Mode::_0)
        .with_sck(slave_sclk)
        .with_mosi(slave_mosi)
        .with_miso(slave_miso)
        .with_cs(slave_cs)
        .with_dma(dma_channel, rx_descriptors, tx_descriptors);

    // Get synchronization primitives
    let mut spi_transaction_end_recv = SPI_TRANSACTION_END.receiver().unwrap();
    let data_ready_send = DATA_UPDATED_ON_RINGBUFF.sender();

    loop {
        // Start a new SPI read transfer
        let mut transfer = spi.read(rx_buffer).unwrap();

        // Wait for CS rising edge (transaction end)
        spi_transaction_end_recv.changed().await;

        // Ensure DMA transfer is complete (just safety check)
        // Note: The SPI transaction is already completed when the CS pin goes high
        // but we wait for the transfer as an extra check for data integrity
        while !transfer.is_done() {
            Timer::after(Duration::from_millis(10)).await;
        }
        drop(transfer);

        // Process received data
        push_to_buffer(*rx_buffer).expect("Failed to push data to circular buffer");
        data_ready_send.send(true);
    }
}

// Interrupt handler for CS pin - detects transaction end
// This function is called when the CS pin goes high (rising edge)
// When interrupt happens it triggers the waker channel for the SPI task to know the transaction is done and that it can upload data to circ buffer
// It checks if the interrupt was triggered by the CS pin and clears the interrupt flag
#[handler]
#[ram]
fn handler() {
    let cs_pin_triggered = critical_section::with(|cs| {
        CS_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    });

    if cs_pin_triggered {
        // CS pin rising edge detected - SPI transaction complete
        SPI_TRANSACTION_END.sender().send(true);
    } else {
        println!("Warning: Interrupt triggered by unknown source");
    }

    // Clear the interrupt flag
    critical_section::with(|cs| {
        CS_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
