# Arduino Pro Mini Interrupt and Sleep Project

This project demonstrates how to use interrupts and deep sleep mode on an Arduino Pro Mini (5V, 16MHz). It monitors rising and falling edge signals on specific pins and toggles a power pin based on the signal state.

## Table of Contents
- [About](#about)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## About
This project is designed for the Arduino Pro Mini (5V, 16MHz) and is written in VS Code using PlatformIO. It uses interrupts to wake the microcontroller from deep sleep mode and toggles a power pin based on the state of the monitored pins.

### Key Functionality:
- Monitors rising and falling edge signals on pins 2 and 3.
- Puts the microcontroller into deep sleep mode to save power.
- Wakes up on external interrupts and checks the state of the monitored pins.
- Toggles pin 10 (PowerPin) to generate a 500ms LOW signal when specific conditions are met.

## Features
- **Interrupt Handling**: Uses external interrupts on pins 2 and 3 to wake the microcontroller.
- **Deep Sleep Mode**: Saves power by putting the microcontroller into deep sleep mode.
- **Signal Monitoring**: Monitors the state of pins 2 and 3 for rising and falling edges.
- **Power Pin Control**: Toggles pin 10 (PowerPin) based on signal stability and state changes.

## Installation
1. Install [PlatformIO](https://platformio.org/) in VS Code.
2. Connect a USB-to-serial (TTL) adapter to program the Arduino Pro Mini.
3. Clone this repository or copy the code into your PlatformIO project.

## Usage
1. Upload the code to your Arduino Pro Mini using PlatformIO.
2. Connect external signals to pins 2 and 3 for rising and falling edge detection.
3. Monitor the serial output for debugging information.
4. Pin 10 will generate a 500ms LOW signal when the monitored pin state changes and remains stable for 10 checks.

## Configuration
- **Pin Assignments**:
  - `WAKE_PIN_RISING` (Pin 2): Monitors rising edge signals.
  - `WAKE_PIN_FALLING` (Pin 3): Monitors falling edge signals.
  - `LED_PIN` (Pin 13): Indicates activity with an LED.
  - `PowerPin` (Pin 10): Generates a 500ms LOW signal when triggered.
- **Timing**:
  - The pin state is checked 10 times with a 100ms delay between checks to ensure stability.

## Contributing
Contributions are welcome! Feel free to submit issues or pull requests to improve the project.

## License
This project is licensed under the [MIT License](LICENSE).

## Contact
For questions or feedback, please contact the project maintainer.


