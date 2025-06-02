# BlackBox Firmware

This project is a firmware for an STM32-based BlackBox data recorder that controls multiple cameras (e.g. GoPro) and logs flight data. Commands are received over **UART3 (STLink Virtual COM Port)** using **HUART3**.

## TODOS
* currently there are no checks for how long goprobutton is pressed (so all gopro commands are basicly the same). Think turning on/off, start stop have different timings.
* Get gopro button timings for start,stop,turnon,turnoff.
* write tests in tests.c/h
* refactor flash driver(low priority)

## Good to know
* canbus is setup with callbacks. So no need to call any get message function etc.
* logging via serial can be enabled in main.h (ENABLE_SERIAL_PRINTF)
* 
## Features

- Camera control: Start/stop recording, format, reboot, debug
- GoPro GPIO control
- Flash memory logging and management
- System reset
- FDCAN interface for vehicle communication
- SPI flash memory driver for W25N01GVSFIG

## Serial Communication

All commands are 1-byte values sent over **UART3**.

## Command Table

| Command (Hex) | Description                     |
|---------------|---------------------------------|
| `0x00`        | No operation                    |
| `0x41`        | Set all cameras to IDLE         |
| `0x42`        | Start recording on all cameras  |
| `0x43`        | Format SD cards on all cameras  |
| `0x44`        | Reboot all cameras              |
| `0x45`        | Enable debug (Wi-Fi) mode       |
| `0x47`        | Reboot MCU                      |
| `0x48`        | Start GoPro filming             |
| `0x49`        | Stop GoPro filming              |
| `0x4A`        | Power on GoPro                  |
| `0x4B`        | Power off GoPro                 |
| `0x4C`        | Erase flash memory              |
| `0x4D`        | Start flight recording          |
| `0x4E`        | Stop flight recording           |
| `0x52`        | Read recorded flight data       |

## Communication Interface

- **UART3 (STLink)** is used to receive control commands
- **Other UARTs (e.g., UART5, UART8, UART2)** are used to communicate with individual camera modules
- **FDCAN1** is initialized for receiving/sending external CAN messages
- **SPI1** is used for flash memory operations


## Flash Memory

Flash chip: W25N01GVSFIG