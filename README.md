
# ESP32s3APRS

The ESP32s3APRS project aims to develop an open-source APRS (Automatic Packet Reporting System) beacon/tracker/logger, USB kiss TNC , USB audio FM tranceiver with Micropython shell based on the ESP32-S3 microcontroller.

## Project Overview

The ESP32s3APRS is a comprehensive solution that combines hardware and software components to create a versatile packet radio. The main features include:

- **APRS and AX.25 Capabilities**: The device can transmit and receive APRS and AX.25 frames, allowing for bidirectional communication over radio.
- **USB Connectivity**: The tracker connects to a computer via USB, providing two CDC-ACM ports:
  - One port for a TNC KISS protocol interface
  - One port for a MicroPython console
- **UAC2 Audio Support**: The USB implementation includes the UAC2 protocol, enabling direct audio streaming between the device and the computer.
- **GPS**: Integrated GPS with embeded antenna.
- **Sensor Integration**: The tracker can integrate various sensors, to enrich the data transmitted over the APRS network.
- **USB-C battery charger**: can accept power input from 5v to 14V 
- **User Interface**: The device features an e-paper display and three buttons for configuration and control.
- **Power Management**: The system is designed for low power consumption, allowing for extended autonomous operation.

## Hardware Components

The ESP32s3APRS hardware consists of the following main components:

- ESP32-S3 microcontroller
- SA868 radio module
- Quectel L86 GPS receiver
- E-paper display
- 3 buttons
- USB-C battery charger

The schematic and PCB are designed with Kicad (https://www.kicad.org).
The hardware design files are distributed under the CERN-OHL-S v2 license (or later)

## Software

The software for the ESP32s3APRS is released under the GPL v3 (or later) license. It includes:

- Embedded firmware for the ESP32-S3
- APRS and AX.25 frame processing
- Data storage in flash memory
- Configuration via the user interface and MicroPython console

## Getting Started

To get started with the ESP32s3APRS project, please follow these steps:

1. Clone the repository: `git clone https://github.com/mcapdeville/esp32s3aprs.git`
2. Change to project directory: cd esp32s3aprs
3. Initialize submodules : ./init.sh

## Compile

From the project directory :

1. Load the environment variables: . ./esp-idf/export.sh
2. Compile the project: idf.py build

## Flash the device

1. Put the switch on 'Boot' position
2. Connect USB cable
3. Run: idf.py flash

## Contributing

The ESP32s3APRS project welcomes contributions from the community. You can contribute by:

- Reporting bugs or issues
- Suggesting new features or improvements
- Submitting pull requests for code or documentation changes

## License

The ESP32s3APRS software is licensed under the GPL v3 (or later) license.
The hardware design files are licensed under the CERN-OHL-S v2 license.
See the `LICENSE-GPL-V3.txt` and `LICENSE-CERN-OHL-S-V2.txt` files for more details.

## Acknowledgments

The ESP32s3APRS project was created by Marc CAPDEVILLE (F4JMZ).
