# SPI Slave Example for ESP32

## Overview
This project demonstrates how to use an ESP32 as an SPI slave device with DMA (Direct Memory Access). It receives data from an SPI master device and makes it available asynchronously in a ring buffer using Embassy and no_std environment.

## Features
- SPI slave implementation with DMA buffer, receives chunks `CHUNK_SIZE` of data each time.
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
4. When the CS pin goes high (transaction complete), an interrupt is triggered. Interrupt triggers `SPI_TRANSACTION_END` waker chan.
5. The SPI task wakes up with `SPI_TRANSACTION_END` copies the received data to a circular buffer, `CIRCULAR_BUFFER`. After that it fires the `DATA_UPDATED_ON_RINGBUFF` waker chan
6. A separate dummy data processing task wakes up with `DATA_UPDATED_ON_RINGBUFF` and then "processes"CIRCULAR_BUFFER the data and prints the simple statistics.

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
This project requires the Embassy RTOS framework and ESP32 HAL. Use Rust with appropriate feature flags. Flash to your ESP32 using standard ESP tools (e.g., `cargo run`).

Remember to change to your specific ESP32 version in the `Cargo.toml` file and config the build setup accordingly.

## Notes
- You may need to change the number of channels if you want to receive more data.
- You can also change the buffer size, `CHUNK_SIZE`, etc.
- `DMA_BUFFER` size needs to be greater than the `CHUNK_SIZE` (both master and slave should have the same `CHUNK_SIZE`).

## Limitations
- Only supports read operations (ESP32 as slave receiver).
- The CS pin is manually controlled by the master and is triggered per chunk of data sent.
- Fixed buffer sizes must be set at compile time, and the DMA buffer must be a multiple of 4 bytes and the same size as the master `CHUNK_SIZE`.
- Requires ESP32 HAL with unstable features.
- Not the most ideal solution due to limitations in the esp-hal, such as the inability to handle DMA interrupts or use the provided DMA circular buffer due to bugs and other issues in the HAL.

## Troubleshooting
If you encounter issues:
- Check hardware connections and pin configuration.
- Ensure the CS pin is properly controlled by the master device and that you are using the provided master implementation in the other board.
- Verify that master and slave use compatible SPI modes and the same `CHUNK_SIZE`, and that the `CHUNK_SIZE` is a multiple of 4 bytes.


See the source code comments and the master implementation docs for more information.

Feel free to contact me if you have any issues, doubts, or questions!
