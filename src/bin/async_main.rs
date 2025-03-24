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
use embassy_time::{Duration, Instant, Timer};
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
const CHUNK_SIZE: usize = 512 * 4; //
const DMA_BUFFER_SIZE: usize = CHUNK_SIZE; // Size of the DMA buffer i making it equal to the chunk size (make sure it is always multiple of 4 bytes)
const CIRCULAR_BUFFER_SIZE: usize = 40; // Size of the circular buffer

static SPI_TRANSACTION_END: Watch<CriticalSectionRawMutex, bool, 6> = Watch::new(); // be free to change the number of channel consummers according to your needs
static DATA_UPDATED_ON_RINGBUFF: Watch<CriticalSectionRawMutex, bool, 6> = Watch::new(); // be free to change the number of channel consummers according to your needs
static CS_PIN: critical_section::Mutex<RefCell<Option<Input>>> =
    critical_section::Mutex::new(RefCell::new(None));

use circular_buffer::CircularBuffer;

// Thread-safe circular buffer
static CIRCULAR_BUFFER: critical_section::Mutex<
    RefCell<Option<CircularBuffer<CIRCULAR_BUFFER_SIZE, [u8; CHUNK_SIZE]>>>,
> = critical_section::Mutex::new(RefCell::new(None));

// Initialize the buffer in main
fn init_circular_buffer() {
    critical_section::with(|cs| {
        CIRCULAR_BUFFER
            .borrow_ref_mut(cs)
            .replace(CircularBuffer::<CIRCULAR_BUFFER_SIZE, [u8; CHUNK_SIZE]>::new());
    });
}

// Helper functions to safely access the buffer
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
// get latest value from the buffer
fn pop_latest_from_buffer() -> Option<[u8; CHUNK_SIZE]> {
    critical_section::with(|cs| {
        CIRCULAR_BUFFER
            .borrow_ref_mut(cs)
            .as_mut()
            .and_then(|buf| buf.pop_front())
    })
}
// get oldest value from the buffer
fn pop_oldest_from_buffer() -> Option<[u8; CHUNK_SIZE]> {
    critical_section::with(|cs| {
        CIRCULAR_BUFFER
            .borrow_ref_mut(cs)
            .as_mut()
            .and_then(|buf| buf.pop_back())
    })
}

// check if the buffer is empty
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
    // Initialize the circular buffer
    init_circular_buffer();

    // Initialize the SPI and DMA
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

    // Background task that "works" on the data
    spawner.spawn(background_task()).unwrap();

    loop {
        println!("Main loop is free to do other things");

        // Add a delay between iterations
        Timer::after(Duration::from_millis(1000)).await;
    }
}

#[embassy_executor::task]
async fn background_task() {
    let mut wait_new_data = DATA_UPDATED_ON_RINGBUFF.receiver().unwrap(); // ge the wait handle for spi transaction finished
                                                                          // we can also use different types of channels if you like or even send the data over the channel instead of putting in the buffer
                                                                          // but this is a simple example

    // This task is just a placeholder for the background task
    // if the background task cannot handle the speed the data is sent the data will be stored in the circular buffer (right now it fits 40 times the CHUNK_SIZE)
    let mut total_data_recv = 0;
    let mut start_time = Instant::now();
    let mut everything_was_valid = true;
    loop {
        wait_new_data.changed().await; // async await for spi slave to have a transaction
                                       //println!("Background task: GOT new data in the buffer to display");
        let buff = match pop_latest_from_buffer() {
            Some(buffer) => buffer,
            None => {
                println!("No data available in buffer");
                continue; // Skip the rest of this iteration
            }
        };
        if buff[2] != 0xFF {
            if buff[2] == 0xED {
                // marker to display the statistics data
                let data_rate =
                    total_data_recv as f64 / (start_time.elapsed().as_millis() as f64 / 1000.0);
                println!("Total data received: {} bytes", total_data_recv);
                println!("Data rate: {:.2} bytes/second", data_rate);

                start_time = Instant::now(); // Reset start time
                total_data_recv = 0; // Reset total data sent

                if everything_was_valid {
                    println!("Everything was valid");
                } else {
                    println!("Something was wrong with the data");
                }
                everything_was_valid = true;
            } else {
                everything_was_valid = false;
            }
        }

        total_data_recv += buff.len(); // update the total data received

        // if buff.len() > 20 {
        //     println!(
        //         "stuff in the buffer {:x?} .. {:x?}",
        //         &buff[..10],
        //         &buff[buff.len() - 10..],
        //     );
        // } else {
        //     println!("stuff in the buffer {:x?}", &buff[..]);
        // }
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
    let (rx_buffer, rx_descriptors, _, tx_descriptors) = dma_buffers!(DMA_BUFFER_SIZE, 0);

    let mut spi = spi::slave::Spi::new(spi_chan, spi::Mode::_0)
        .with_sck(slave_sclk)
        .with_mosi(slave_mosi)
        .with_miso(slave_miso)
        .with_cs(slave_cs)
        .with_dma(dma_channel, rx_descriptors, tx_descriptors);

    let mut spi_transaction_end_recv = SPI_TRANSACTION_END.receiver().unwrap();
    let data_ready_send = DATA_UPDATED_ON_RINGBUFF.sender();
    loop {
        let mut transfer = spi.read(rx_buffer).unwrap();
        spi_transaction_end_recv.changed().await;
        //println!("SPI GOT data in SLAVE");

        while !transfer.is_done() {
            // Wait for the transfer to complete
            //println!("just waiting for dma transfer to finish, the spi transaction was already ");
            Timer::after(Duration::from_millis(10)).await; // just some little delay so it allows the transfer to finish and other routines to go on
        }
        drop(transfer);

        push_to_buffer(*rx_buffer).expect("Failed to push data to circular buffer"); // push the received data to the circular buffer
        data_ready_send.send(true); // send signal that data is ready
                                    //println!("Pushed data to circular buffer");
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
        let sender = SPI_TRANSACTION_END.sender();
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
