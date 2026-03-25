# PKE-Car-System

# ESP32-S3 Based PKE System for Vehicle Access and Ignition

## Project Overview
This project presents the design and implementation of a Passive Keyless Entry (PKE) system for a vehicle, based on the ESP32-S3 microcontroller.  
The system enables keyless door locking and unlocking, mirror folding and unfolding, and vehicle ignition control through a Start-Stop push button, using BLE proximity detection.

The project combines hardware, firmware, PCB design, and practical testing into a complete embedded automotive-oriented solution.  
A dedicated PCB was designed to integrate the ESP32-S3 controller, protection circuits, input conditioning stages, relay drivers, and relay blocks required for interfacing with the vehicle.

In addition to the PKE functionality, the system was designed to operate alongside the original vehicle key without interfering with its normal operation.

## System Architecture
The system is built around the following main blocks:

- ESP32-S3 controller
- BLE-based proximity detection
- Start-Stop push button interface
- Vehicle input conditioning circuits
- Protection and voltage level adaptation circuits
- Relay drivers and relay blocks
- Dedicated custom PCB
- Power management and DC-DC conversion

## Main Inputs
- BLE tag for proximity detection
- Brake pedal signal
- Handbrake signal
- Door status signal
- Ignition status signal
- Start-Stop button feedback

## Main Outputs
- Door lock control
- Door unlock control
- Mirror fold and unfold control
- ACC control
- IGN control
- START control
- Turn signal indication outputs
- Start-Stop button LED control

## Key Features
- Passive keyless vehicle access using BLE proximity detection
- Start-Stop ignition control based on defined safety conditions
- Support for vehicle functions such as mirrors, blinkers, and ignition lines
- Dedicated PCB for structured and reliable hardware integration
- State-based firmware logic for clear and predictable system behavior
- Practical implementation and testing on prototype and final PCB
- Compatibility with the original vehicle key system

## Repository Structure
- `Book/` - Final project report and documentation
- `Code/` - ESP32-S3 firmware source code
- `KiCad/` - Schematic and PCB design files
- `Multisim/` - Simulation files
- `BOM/` - Bill of Materials
- `Photos/` - Project images, prototype, PCB, and assembly photos
- `Poster/` - Project poster and presentation materials

## Tools and Technologies
- **Microcontroller:** ESP32-S3
- **Firmware:** Arduino IDE, C/C++
- **PCB Design:** KiCad
- **Simulation:** Multisim
- **Communication:** BLE
- **Hardware:** Relay drivers, relays, protection circuits, DC-DC converter
- **Application Domain:** Embedded systems, automotive electronics

## Project Outcome
The project demonstrates a complete embedded hardware-software solution for implementing a custom PKE system in a vehicle environment.  
It includes concept definition, hardware design, PCB implementation, firmware development, simulation, prototype testing, and final integration.

## Author
Final project in Electrical and Electronics Engineering  
Developer: Neriya Shalom
