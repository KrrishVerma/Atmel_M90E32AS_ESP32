# Atmel_M90E32AS_ESP32_Library
ESP32 Compatible Library for Poly-Phase High-Performance Wide-Span Energy Metering IC

# Overview
The EnergyMeteringIC library enables precise monitoring of electrical parameters in a three-phase power system. It utilizes SPI communication to configure the ATM90E32 IC, set line frequency and gains, and retrieve data such as voltage, current, power, power factor, frequency, and temperature. The library includes functions like readWriteRegister() and read32BitRegister() for efficient register access, with error detection for SPI failures (e.g., disconnected wires) to enhance reliability. Initial versions include EnergyMonitorIC.h and EnergyMonitorIC.cpp, released as of June 20, 2025.

# Features

    Supports ESP32 architecture.
    Configures ATM90E32 for three-phase energy monitoring.
    Retrieves voltage, current, power, power factor, frequency, and temperature.
    Implements SPI-based communication with error handling.
    Modular design for easy integration into Arduino projects.

# Installation
Via Arduino Library Manager

    Open the Arduino IDE.
    Go to Sketch > Include Library > Manage Libraries....
    Search for EnergyMeteringIC.
    Click Install Button.

# Manual Installation

    Download the latest release from the Releases page.
    Extract the .zip file.
    Move the EnergyMeteringIC folder to your Arduino libraries directory (e.g., ~/Documents/Arduino/libraries/).
    Restart the Arduino IDE.
