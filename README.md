# ESP8266 Air Pump Kitchen

This project is a smart air pump and light controller for a kitchen environment, built using an ESP8266 microcontroller. It features WiFi connectivity, MQTT integration, and a web-based control interface.

## Features

- **WiFi Connectivity**: Connects to a WiFi network for remote control and monitoring.
- **MQTT Integration**: Communicates with an MQTT broker for smart home automation.
- **Web Interface**: Provides a web-based control panel for manual operation and configuration.
- **DHT Sensor**: Monitors temperature and humidity using a DHT22 sensor.
- **Relay Control**: Controls fan speeds and light using relays.
- **OTA Updates**: Supports over-the-air firmware updates.
- **EEPROM Storage**: Saves configuration settings in EEPROM.


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

## Dependencies

- [ESP8266WiFi](https://github.com/esp8266/Arduino)
- [PubSubClient](https://github.com/knolleary/pubsubclient)
- [ArduinoJson](https://arduinojson.org/)
- [DHT Sensor Library](https://github.com/RobTillaart/DHTNew)

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

- Inspired by DIY smart home projects.
- Uses open-source libraries and tools for development.