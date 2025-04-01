# SPI Slave Example for ESP32

## Overview
This project demonstrates how to use an ESP32 as an SPI slave device with DMA (Direct Memory Access). It asynchronously receives data from an SPI master device and stores it in a ring buffer, leveraging the Embassy framework in a `no_std` environment. Refer to the SPI master repository for more instructions on running the example test provided in the source code.

## Features
- SPI slave implementation with DMA buffer, receives chunks with `CHUNK_SIZE` batches of data each time.
- Interrupt-driven approach using CS pin for transaction end detection
- Circular buffer allows different speeds between receiving data and processing data
- Nearly everything is asynchronous with separate tasks and waker channels for notifications
- Performance monitoring with data rate tracking and very simple data integrity checks


## Hardware Setup
Connect your SPI master device to these ESP32 pins:
- **SCLK**: GPIO10
- **MISO**: GPIO11 (output from ESP32) // the slave output is not really used as communication is unidirectional
- **MOSI**: GPIO12 (input to ESP32)
- **CS**: GPIO13
- **GND**: Remember to connect the ground of both boards and ensure they are operating at the same voltage.

### Pin Connections (feel free to change them)
| Signal | Master (ESP32) | Slave |
|--------|----------------|-------|
| SCLK   | GPIO27         | GPIO10|
| MISO   | GPIO35         | GPIO11|
| MOSI   | GPIO22         | GPIO12|
| CS     | GPIO21         | GPIO13|

## How It Works
1. The ESP32 configures itself as an SPI slave and waits for data.
2. When the CS pin goes low, the master starts sending data.
3. Data is received via DMA into a buffer.
4. When the CS pin goes rising edge (transaction complete), an interrupt is triggered. Interrupt triggers `SPI_TRANSACTION_END` waker chan.
5. The SPI task wakes up with `SPI_TRANSACTION_END` copies the received data to a circular buffer, `CIRCULAR_BUFFER`. After that it fires the `DATA_UPDATED_ON_RINGBUFF` waker chan, signaling that there is new data in the `CIRCULAR_BUFFER`
6. A separate dummy data processing task wakes up with `DATA_UPDATED_ON_RINGBUFF` and then "processes" the `CIRCULAR_BUFFER` latest data and prints the simple statistics.

## Memory Usage (Configurable through constants - see comments in the source code)
- **Chunk Size**: 2048 bytes (512 * 4)
- **DMA Buffer**: 2048 bytes
- **Circular Buffer**: Stores 40 chunks 

## Code Structure
The application consists of three main tasks:
- **Main Task**: Initializes hardware and spawns worker tasks
- **SPI Slave Task**: Handles SPI communication and DMA transfers
- **Data Processing Task**: Processes received data and displays statistics

- **Circular Buffer Helper Functions**: Provides thread-safe access to the ring buffer using critical section mutexes

## Building and Running
This project uses the Embassy runtime framework and ESP32 HAL. Build with Rust using appropriate feature flags and toolchains (see https://docs.esp-rs.org/book/installation/riscv-and-xtensa.html for complete setup instructions). Flash to your ESP32 using standard ESP tools with `cargo run`.

Remember to change to your specific ESP32 version in the `Cargo.toml` file and config the build setup accordingly.

## Notes
- You may need to change the number of channels if you want to have more concurrent tasks waiting for the data.
- You can also change the buffer size, `CHUNK_SIZE`, etc.
- `DMA_BUFFER` size needs to be greater than the `CHUNK_SIZE` (both master and slave should have the same `CHUNK_SIZE`) and `DMA_BUFFER` needs to be a multiple of 4 bytes according to ESP-HAL docs.

## Limitations
- Only supports read operations (ESP32 as slave receiver). ->  as per the requirements unidirectional data transfer master -> slave
- The CS pin is manually controlled by the master and is triggered per chunk of data sent.
- Fixed buffer sizes must be set at compile time, and the DMA buffer must be a multiple of 4 bytes and the same size as the master `CHUNK_SIZE`.
- Requires ESP32 HAL with unstable features.
- Not the most ideal solution due to limitations in the esp-hal, such as the inability to handle DMA interrupts or use the provided DMA circular buffer due to bugs and other issues in the HAL. The ESP-HAL crate is unfortunately not very developed in this area, and it would be out of the project scope to make my custom version of the HAL with all the necessary features.

## Troubleshooting
If you encounter issues:
- Check hardware connections and pin configuration.
- Ensure the CS pin is properly controlled by the master device and that you are using the provided helper functions for the SPI master in the other board (master).
- Ensure that the ground pin is connected on both boards
- If it is a problem in the build/flashing remember to change the configs (mainly Cargo.toml) for your specific ESP32 version
- Verify that master and slave use compatible SPI modes and the same `CHUNK_SIZE`, and that the `CHUNK_SIZE` is a multiple of 4 bytes.


See the source code comments and the corresponding master implementation docs for more information.
