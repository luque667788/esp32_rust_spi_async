//! SPI slave loopback test using DMA
//!
//! The following wiring is assumed for the (bitbang) slave:
//!
//! - SCLK => GPIO10
//! - MISO => GPIO11
//! - MOSI => GPIO12
//! - CS   => GPIO13
//!
//! The following wiring is assumed for the (bitbang) master:
//! - SCLK => GPIO4
//! - MISO => GPIO5
//! - MOSI => GPIO6
//! - CS   => GPIO7
//!
//! Depending on your target and the board you are using you have to change the
//! pins.
//!
//! This example transfers data via SPI.
//!
//! Connect corresponding master and slave pins to see the outgoing data is read
//! as incoming data. The master-side pins are chosen to make these connections
//! easy for the barebones chip; all are immediate neighbors of the slave-side
//! pins except SCLK. SCLK is between MOSI and VDD3P3_RTC on the barebones chip,
//! so no immediate neighbor is available.

//% CHIPS: esp32c2 esp32c3 esp32c6 esp32h2 esp32s2 esp32s3
//% FEATURES: esp-hal/unstable

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, watch::Watch};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    dma_circular_buffers,
    gpio::{Event, Io},
    handler,
    peripheral::{self, Peripheral},
    ram,
    timer::timg::TimerGroup,
};

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    main, spi,
};
use esp_println::println;

use core::cell::RefCell;
const CHUNK_SIZE: usize = 512 * 4;

// Use AtomicUsize for integers that need to be safely modified across threads or interrupts
use core::sync::atomic::{AtomicUsize, Ordering};

// Define DMA_HEAD as an atomic
static DMA_HEAD: AtomicUsize = AtomicUsize::new(0);
static DMA_TAIL: AtomicUsize = AtomicUsize::new(0);

static SIGNAL: Watch<CriticalSectionRawMutex, bool, 6> = Watch::new();
static CS_PIN: critical_section::Mutex<RefCell<Option<Input>>> =
    critical_section::Mutex::new(RefCell::new(None));

static DMA_BUFFER: embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Option<[u8; 2048]>> =
    embassy_sync::mutex::Mutex::new(None);

#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    // Initialize the peripherals
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    // Initialize GPIO interrupts
    let mut io = Io::new(peripherals.IO_MUX);
    esp_alloc::heap_allocator!(size: 72 * 1024);
    io.set_interrupt_handler(handler);

    // Configure all the GPIO pins with appropriate types
    let config_in = InputConfig::default().with_pull(Pull::Up);
    let config_out = OutputConfig::default();

    // Convert pins to their appropriate static types
    let slave_sclk: Input<'static> = Input::new(peripherals.GPIO10, config_in);
    let slave_miso: Output<'static> = Output::new(peripherals.GPIO11, Level::Low, config_out);
    let slave_mosi: Input<'static> = Input::new(peripherals.GPIO12, config_in);
    let mut slave_cs_clone: Input<'static> =
        Input::new(unsafe { peripherals.GPIO13.clone_unchecked() }, config_in); //clone for the interrupt, race condition wont occur we are in the critical section and using mutexes
    let slave_cs: Input<'static> = Input::new(peripherals.GPIO13, config_in);

    // Set up the CS interrupt, risisng edge is when the spi transaction ends
    critical_section::with(|cs| {
        slave_cs_clone.listen(Event::RisingEdge);
        CS_PIN.borrow_ref_mut(cs).replace(slave_cs_clone);
    });

    spawner
        .spawn(slave_spi_routine(
            slave_cs,
            slave_sclk,
            slave_miso,
            slave_mosi,
            peripherals.DMA_CH0,
            peripherals.SPI2,
        ))
        .unwrap();
    spawner.spawn(background_task()).unwrap();

    loop {
        println!("Main loop is free to do other things");

        // Add a delay between iterations
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn background_task() {
    let mut wait_transfer = SIGNAL.receiver().unwrap(); // ge the wait handle for spi transaction finished

    // This task is just a placeholder for the background task
    loop {
        wait_transfer.changed().await; // async await for spi slave to have a transaction
        println!("Background task: SPI transaction finished");
        // Do something with the data received
        // Do something with the data received
        let buffer_guard = DMA_BUFFER.lock().await;
        if let Some(ref buffer) = *buffer_guard {
            // Process the data as 512-float chunks
            let offset = DMA_HEAD.load(Ordering::SeqCst);
            // Find any bytes equal to 0xFF and log their positions
            println!("Searching for 0xFF bytes in the current chunk...");
            for (i, &byte) in buffer.iter().enumerate() {
                if byte == 0xFF {
                    println!("Found 0xFF at index {} (the offset is: {})", i, offset);
                }
            }

            println!(
                "recv stuff {:x?} .. {:x?}",
                &buffer[..10],
                &buffer[buffer.len() - 10..],
            );
            
        }
    }
}

#[embassy_executor::task]
async fn slave_spi_routine(
    slave_cs: Input<'static>,
    slave_sclk: Input<'static>,
    slave_miso: Output<'static>,
    slave_mosi: Input<'static>,
    dma_channel: esp_hal::dma::DmaChannel0,
    spi_chan: esp_hal::peripherals::SPI2,
) {
    // Initialize the SPI
    // put zero in the tx buffer because slave doesn't send data (just receives)
    // choose a buffer size that is a multiple of 4 bytes
    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_circular_buffers!(2048, 0);

    {
        let mut buffer = DMA_BUFFER.lock().await;
        *buffer = Some(*rx_buffer);
    }

    let mut spi = spi::slave::Spi::new(spi_chan, spi::Mode::_0)
        .with_sck(slave_sclk)
        .with_mosi(slave_mosi)
        .with_miso(slave_miso)
        .with_cs(slave_cs)
        .with_dma(dma_channel, rx_descriptors, tx_descriptors);

    let mut recv = SIGNAL.receiver().unwrap();
    loop {
        let mut transfer = spi.read(rx_buffer).unwrap();
        recv.changed().await;
        println!("SPI GOT data in SLAVE");

        while !transfer.is_done() {
            // Wait for the transfer to complete
            println!("just waiting for dma transfer to finish, the spi transaction was already ");
            Timer::after(Duration::from_millis(10)).await; // just some little delay so it allows the transfer to finish and other routines to go on
        }
        drop(transfer);
    }
}

#[handler]
#[ram]
fn handler() {
    // Check if the interrupt was triggered by the CS pin
    if critical_section::with(|cs| {
        CS_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        // interrupt was triggered by the CS pin rising edge, firing channel
        // Update the head position with wrapping add
        {
            // Use atomic operations instead of direct mutation
            let current_head = DMA_HEAD.load(Ordering::SeqCst);
            let new_head = (current_head + CHUNK_SIZE) % 32000;
            DMA_HEAD.store(new_head, Ordering::SeqCst);

            // Update tail to follow head, maintaining proper distance
            let current_tail = DMA_TAIL.load(Ordering::SeqCst);

            if new_head > current_tail && new_head - current_tail > 3 * CHUNK_SIZE {
                // If head is ahead by more than 3 chunks, move tail forward
                DMA_TAIL.store(new_head - 3 * CHUNK_SIZE, Ordering::SeqCst);
            } else if new_head < current_tail && (32000 - current_tail + new_head) > 3 * CHUNK_SIZE
            {
                // Handle wrap-around case
                DMA_TAIL.store(
                    (new_head + 32000 - 3 * CHUNK_SIZE) % 32000,
                    Ordering::SeqCst,
                );
            }
        }
        let sender = SIGNAL.sender();
        sender.send(true);
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        CS_PIN
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

// Helper function to convert bytes to floats and process them
fn process_float_chunk(chunk_bytes: &[u8]) {
    // Make sure we have enough bytes for 512 floats
    if chunk_bytes.len() != 512 * 4 {
        println!("Unexpected chunk size: {} bytes", chunk_bytes.len());
        return;
    }

    // Create a buffer to hold the floats
    let mut floats = [0.0f32; 512];

    // Convert bytes to floats (assuming little-endian)
    for i in 0..512 {
        let bytes = [
            chunk_bytes[i * 4],
            chunk_bytes[i * 4 + 1],
            chunk_bytes[i * 4 + 2],
            chunk_bytes[i * 4 + 3],
        ];
        floats[i] = f32::from_le_bytes(bytes);
    }

    // Now you have your 512 floats in the 'floats' array
    println!("First few floats: {:?}", &floats[..5]);
    println!("Last few floats: {:?}", &floats[507..]);

    // Process your floats here
    // For example, calculate statistics, apply algorithms, etc.
}
