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
    gpio::{Event, Io},
    handler,
    peripheral::Peripheral,
    ram,
    timer::timg::TimerGroup,
};

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    dma_buffers,
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    main,
    spi,
};
use esp_println::println;

use core::cell::RefCell;

static SIGNAL: Watch<CriticalSectionRawMutex, bool, 6> = Watch::new();
static BUTTON: critical_section::Mutex<RefCell<Option<Input>>> =
    critical_section::Mutex::new(RefCell::new(None));
#[esp_hal_embassy::main]
async fn main(spawner: Spawner) {
    println!("SPI slave loopback test using DMA");
    println!("This example transfers data via SPI.");

    let peripherals = esp_hal::init(esp_hal::Config::default());
    
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let mut io = Io::new(peripherals.IO_MUX);
    esp_alloc::heap_allocator!(size: 72 * 1024);
    io.set_interrupt_handler(handler);



    let slave_sclk = peripherals.GPIO10;
    let slave_miso = peripherals.GPIO11;
    let slave_mosi = peripherals.GPIO12;
    let slave_cs = peripherals.GPIO13;

    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32s2")] {
            let dma_channel = peripherals.DMA_SPI2;
        } else {
            let dma_channel = peripherals.DMA_CH0;
        }
    }
    let slave_cs2;
    unsafe {
        slave_cs2 = slave_cs.clone_unchecked();
    }

    let (rx_buffer, rx_descriptors, tx_buffer, tx_descriptors) = dma_buffers!(32000);

    let mut spi = spi::slave::Spi::new(peripherals.SPI2, spi::Mode::_0)
        .with_sck(slave_sclk)
        .with_mosi(slave_mosi)
        .with_miso(slave_miso)
        .with_cs(slave_cs2)
        .with_dma(dma_channel, rx_descriptors, tx_descriptors);


    // DMA buffer require a static life-time

    let mut slave_send = tx_buffer;
    let mut slave_receive = rx_buffer;
    let mut i = 0;



    let config = InputConfig::default().with_pull(Pull::Up);
    let mut cs_interrupt: Input<'_> = Input::new(slave_cs, config);

    critical_section::with(|cs| {
        cs_interrupt.listen(Event::RisingEdge);
        BUTTON.borrow_ref_mut(cs).replace(cs_interrupt);
    });

    // Set up the CS interrupt
    spawner.spawn(get_slave_dma_buffer(slave_receive)).unwrap();

    let mut recv = SIGNAL.receiver().unwrap();

    loop {

        slave_receive.fill(0xff);

        println!("SPI READ");
        let transfer = spi.read(&mut slave_receive).unwrap();

        // Wait for the transfer to complete
        println!("CS WAIT in MAIN");
        recv.changed().await;
        drop(transfer);
        println!("SPI GOT data in MAIN");


        println!(
            "slave got {:x?} .. {:x?}",
            &slave_receive[..10],
            &slave_receive[slave_receive.len() - 10..],
        );

        // Add a delay between iterations
        //Timer::after(Duration::from_millis(1000)).await;


    }
}


#[embassy_executor::task]
async fn get_slave_dma_buffer(slave_receive_ptr: *const [u8]) {
    let mut recv = SIGNAL.receiver().unwrap();
    let slave_receive = unsafe { &*slave_receive_ptr };
    loop {
        println!("Wait CS in TASK ");
        recv.changed().await;
        println!("GOT CS in TASK ");

        // UNSafely dereference the pointer to access the buffer (this is just an example showing you can acces the data even from a different task)
        // In a real application, you should ensure that the pointer is valid and properly synchronized.
        

        
    }
}

#[handler]
#[ram]
fn handler() {
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
            esp_println::println!(
                "GPIO Interrupt with priority {}",
                esp_hal::xtensa_lx::interrupt::get_level()
            );
        } else {
            esp_println::println!("GPIO Interrupt");
        }
    }

    if critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        esp_println::println!("Button was the source of the INTERRUPT");
        let sender = SIGNAL.sender();
        sender.send(true);
    } else {
        esp_println::println!("Button was not the source of the interrupt");
    }

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}
