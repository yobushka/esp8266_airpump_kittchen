# ESP8266 Air Pump Kitchen ( kitchen hood )


This project is a smart air pump and light controller for a kitchen environment, built using an ESP8266 microcontroller. It features WiFi connectivity, MQTT integration, and a web-based control interface.

## Features

- **WiFi Connectivity**: Connects to a WiFi network for remote control and monitoring.
- **MQTT Integration**: Communicates with an MQTT broker for smart home automation.
- **Web Interface**: Provides a web-based control panel for manual operation and configuration.
- **DHT Sensor**: Monitors temperature and humidity using a DHT22 sensor.
- **Relay Control**: Controls fan speeds and light using relays.
- **OTA Updates**: Supports over-the-air firmware updates.
- **EEPROM Storage**: Saves configuration settings in EEPROM.

## ESP8266 Technical Specifications

### Microcontroller Overview
The ESP8266 is a low-cost Wi-Fi microchip produced by Espressif Systems. This project specifically uses the NodeMCU variant, which integrates the ESP-12E module containing the ESP8266 chip with:

- **Processor**: L106 32-bit RISC microprocessor core running at 80MHz (can be overclocked to 160MHz)
- **Memory**: 
  - 4MB Flash memory (typical for NodeMCU v1.0)
  - 80KB user data RAM
  - 32KB instruction RAM
  - 1KB EEPROM emulation in flash
- **Wireless Connectivity**: 
  - IEEE 802.11 b/g/n Wi-Fi
  - Integrated TCP/IP protocol stack
  - WPA/WPA2 authentication
  - Support for STA/AP/STA+AP operation modes
- **Peripheral Interfaces**:
  - 17 GPIO pins (multiplexed with other functions)
  - SPI, I2C, I2S, UART
  - 10-bit ADC (1 pin only)
- **Operating Voltage**: 3.3V (not 5V tolerant)
- **Power Consumption**:
  - ~70mA in active transmission
  - ~20μA in deep sleep mode

### Pin Mapping Used in This Project

| Function | ESP8266 GPIO | NodeMCU Pin |
|----------|-------------|-------------|
| Fan Speed 1 Relay | GPIO1 | D1 |
| Fan Speed 2 Relay | GPIO3 | D3 |
| Fan Speed 3 Relay | GPIO5 | D5 |
| Light Relay | GPIO4 | D4 |
| Button 1 (Fan Off) | GPIO0 | D0 |
| Button 2 (Fan 1) | GPIO2 | D2 |
| Button 3 (Fan 2) | GPIO13 | D13 |
| Button 4 (Fan 3) | GPIO12 | D12 |
| Button 5 (Light) | GPIO14 | D14 |
| DHT22 Sensor | GPIO16 | D16 |

## Hardware Setup

### Required Components
- NodeMCU ESP8266 development board
- DHT22 temperature and humidity sensor
- 4-channel relay module (for fan speeds and light control)
- 5 momentary push buttons (for manual control)
- 10KΩ resistors (for button pull-ups)
- 4.7KΩ resistor (for DHT22 pull-up)
- Micro USB cable and power supply
- Breadboard and jumper wires (for prototyping)
- Enclosure (recommended for final installation)

### Wiring Diagram
```
                             +------+
                             |DHT22 |
                             +--+---+
                                |
                                | Data -> GPIO16
                                |
+----------+                 +--+-------+                 +-----------+
|          |                 |          |                 | 4-Channel |
|   5V     +---------------->|   3.3V   |                 |   Relay   |
|  Power   |                 |          |                 |  Module   |
|  Supply  |                 | ESP8266  |                 |           |
|          |                 | NodeMCU  |                 |           |
|          |                 |          +-- GPIO1 ------->| Relay 1   |
|          |                 |          |                 | (Fan 1)   |
|          |                 |          +-- GPIO3 ------->| Relay 2   |
|          |                 |          |                 | (Fan 2)   |
|          |                 |          +-- GPIO5 ------->| Relay 3   |
|          |                 |          |                 | (Fan 3)   |
|          |                 |          +-- GPIO4 ------->| Relay 4   |
|          |                 |          |                 | (Light)   |
+----------+                 |          |                 +-----------+
                             |          |
                             |          |                 +-----------+
                             |          +-- GPIO0 <-------+ Button 1  |
                             |          |                 +-----------+
                             |          +-- GPIO2 <-------+ Button 2  |
                             |          |                 +-----------+
                             |          +-- GPIO13 <------+ Button 3  |
                             |          |                 +-----------+
                             |          +-- GPIO12 <------+ Button 4  |
                             |          |                 +-----------+
                             |          +-- GPIO14 <------+ Button 5  |
                             |          |                 +-----------+
                             +----------+
```

## Software Architecture

This project uses an object-oriented approach with several key classes:

- **ConfigManager**: Handles EEPROM configuration storage and retrieval
- **WiFiHandler**: Manages WiFi connections, AP mode, and OTA updates
- **DHTSensor**: Interfaces with the DHT22 sensor for temperature and humidity readings
- **Button**: Debounces and detects button presses
- **RelaySwitch**: Controls relay outputs
- **MQTTHandler**: Handles MQTT communication, auto-discovery, and state reporting
- **WebServerHandler**: Provides the web interface and API endpoints
- **AirPumpController**: Main application controller integrating all components

## MQTT Integration

### Topics Structure

The device uses the following MQTT topics (where `location` and `hostname` are configurable):

- **Availability**: `location/hostname/status` - Reports device online/offline status
- **State**: `location/hostname/state` - Reports device state in JSON format
- **Commands**:
  - Main: `location/hostname/command` - General commands
  - Light: `location/hostname/lamp/command` - Light control (ON/OFF)
  - Fan: `location/hostname/fan/command` - Fan control (0-100% or named speeds)
- **Reporting**: `json/sensors/airpump/location` - Detailed JSON sensor reports

### Home Assistant Integration

The device supports Home Assistant MQTT auto-discovery, automatically creating:
- Temperature sensor
- Humidity sensor
- WiFi signal strength sensor
- Light switch
- Fan control

## Project Structure

```
platformio.ini            # PlatformIO configuration
include/                  # Header files
  README
lib/                      # External libraries
  README
src/
  esp8266_airpump_kittchen.ino  # Main application source code
  clean.cmd               # Cleaning script
  upload.cmd              # Upload script
  build/                  # Build artifacts
    esp8266.esp8266.nodemcuv2/
      esp8266_airpump_kittchen.ino.bin
      esp8266_airpump_kittchen.ino.elf
      esp8266_airpump_kittchen.ino.map
test/                     # Test files
  README
```

## Setup Instructions

1. **Install PlatformIO**: Ensure you have PlatformIO installed in your development environment.
2. **Clone the Repository**: Clone this project to your local machine.
3. **Configure WiFi and MQTT**:
   - Access the device's web interface in setup mode.
   - Configure WiFi credentials and MQTT server details.
4. **Build and Upload**:
   - Use PlatformIO to build and upload the firmware to the ESP8266.
5. **Connect to MQTT Broker**: Ensure the MQTT broker is running and accessible.

## Usage

- **Web Interface**: Access the web interface to control fan speed, light, and view sensor data.
- **MQTT Commands**: Use MQTT topics to control the device remotely.
- **OTA Updates**: Update the firmware over the air using the Arduino IDE or PlatformIO.
- **Physical Controls**: Use the attached buttons for direct manual control.

### Web Interface Features

The modern web interface (built with Bulma CSS) provides:
- Real-time status dashboard
- Fan speed controls (Off, Speed 1, 2, 3)
- Light control (On/Off)
- Temperature and humidity monitoring
- Debug information and MQTT logs
- Device configuration

### First-Time Setup

When first powered on, the ESP8266 will create an access point named "SETUPME" (or your configured hostname). Connect to this access point and navigate to `http://192.168.97.1` to configure the device.

## Troubleshooting

### Common Issues

1. **Device not connecting to WiFi**: 
   - Check WiFi credentials
   - Ensure WiFi network is within range
   - Reset configuration and try again

2. **MQTT not connecting**:
   - Verify MQTT server address and port
   - Check authentication settings if enabled
   - Confirm broker is running and accessible

3. **DHT sensor readings incorrect**:
   - Check wiring connections
   - Ensure proper pull-up resistor is installed
   - Verify DHT sensor is not exposed to direct heat sources

4. **Relays not switching properly**:
   - Check relay module connections
   - Verify relay module voltage requirements match power supply
   - Test relays with manual control buttons

## Future Enhancements

- Battery-backed RTC for time-based operations
- Integration with additional sensors (CO2, air quality)
- Power consumption optimization
- Enhanced automation rules based on temperature/humidity

## Dependencies

- [ESP8266WiFi](https://github.com/esp8266/Arduino)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [ArduinoJson](https://arduinojson.org/)
- [DHT Sensor Library](https://github.com/RobTillaart/DHTNew)
- [uptime_formatter](https://github.com/YiannisBourkelis/Uptime-Library)

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

- Inspired by DIY smart home projects.
- Uses open-source libraries and tools for development.
- Special thanks to the ESP8266 and Arduino communities for their extensive documentation and examples.