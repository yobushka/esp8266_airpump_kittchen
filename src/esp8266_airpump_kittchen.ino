#define MQTT_MAX_PACKET_SIZE 2048
#define VERSION "0.6.0-oop" // Updated version

#include <uptime_formatter.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <dhtnew.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// --- Pin Definitions ---
#define SWITCH1_PIN 1 // fan speed 1
#define SWITCH2_PIN 3 // fan speed 2
#define SWITCH3_PIN 5 // fan speed 3
#define SWITCH4_PIN 4 // light

#define BUTTON1_PIN 0
#define BUTTON2_PIN 2
#define BUTTON3_PIN 13
#define BUTTON4_PIN 12
#define BUTTON5_PIN 14

#define DHT22_PIN 16

// --- Constants ---
const IPAddress AP_IP(192, 168, 97, 1);
const long DEBOUNCE_DELAY = 600;
const int REPORT_INTERVAL_MS = 25000; // Approx 25 seconds (5000 * 5ms delay in original reporter)
const int DHT_ERROR_COUNT_MAX = 30;
const int DHT_TOTAL_ERROR_COUNT_MAX = 2400;

// --- Forward Declarations ---
class ConfigManager;
class WiFiHandler;
class MQTTHandler;
class DHTSensor;
class Button;
class RelaySwitch;
class WebServerHandler;
class AirPumpController;

// --- Global Instances ---
WiFiClient wifiClient;
AirPumpController* controllerInstance = nullptr;

// --- Interface for Controller access ---
// This breaks the circular dependency between MQTTHandler and AirPumpController
class ControllerInterface {
public:
    virtual ~ControllerInterface() {}
    virtual void setFanSpeed(int speed, bool manual) = 0;
    virtual void setLight(bool state) = 0;
    virtual void reportState() = 0;
    virtual int getFanSpeed() const = 0;
    virtual bool isLightOn() const = 0;
};

// --- MQTT Message Buffer ---
// Global variables to pass MQTT messages from callback to controller
bool newMqttMessage = false;
String mqttTopicBuffer;
String mqttPayloadBuffer;

// Global MQTT callback function
void mqttCallbackFunction(char* topic, byte* payload, unsigned int length) {
    mqttTopicBuffer = String(topic);
    mqttPayloadBuffer = "";
    for (unsigned int i = 0; i < length; i++) {
        mqttPayloadBuffer += (char)payload[i];
    }
    newMqttMessage = true;
    Serial.println("MQTT message received: " + mqttTopicBuffer + " = " + mqttPayloadBuffer);
}

// --- Helper Functions ---
String IpAddress2String(const IPAddress& ipAddress) {
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

String urlDecode(String input); // Keep urlDecode as a utility function or move to WebServerHandler

// --- Class Definitions ---

class ConfigManager {
public:
    String ssid = "";
    String pass = "";
    String hostname = "SETUPME";
    String location = "UNKNOWN";
    String mqtt_server = "";
    String mqtt_port = "";
    bool mqtt_auth = false;
    String mqtt_user = "";
    String mqtt_password = "";

    void begin() {
        EEPROM.begin(512);
    }

    bool load() {
        Serial.println("Reading EEPROM...");
        if (EEPROM.read(0) == 0) {
             Serial.println("Config not found (first byte is 0).");
             return false;
        }

        ssid = readStringFromEEPROM(0, 32);
        pass = readStringFromEEPROM(32, 64);
        hostname = readStringFromEEPROM(96, 32);
        location = readStringFromEEPROM(128, 32);
        mqtt_server = readStringFromEEPROM(160, 32);
        mqtt_port = readStringFromEEPROM(192, 8);
        mqtt_auth = (EEPROM.read(200) == '1');
        mqtt_user = readStringFromEEPROM(208, 32);
        mqtt_password = readStringFromEEPROM(230, 32);

        if (hostname == "") hostname = "SETUPME"; // Default if empty
        if (location == "") location = "UNKNOWN"; // Default if empty

        Serial.println("Config loaded:");
        Serial.println("  SSID: " + ssid);
        Serial.println("  Hostname: " + hostname);
        Serial.println("  Location: " + location);
        Serial.println("  MQTT Server: " + mqtt_server);
        // ... print other loaded values if needed ...

        return ssid != ""; // Consider a config valid if SSID is set
    }

    void save(const String& new_ssid, const String& new_pass, const String& new_hostname, const String& new_location,
              const String& new_mqtt_server, const String& new_mqtt_port, const String& new_mqtt_auth,
              const String& new_mqtt_user, const String& new_mqtt_password)
    {
        Serial.println("Saving configuration to EEPROM...");
        clear(); // Clear previous data

        writeStringToEEPROM(0, new_ssid, 32);
        writeStringToEEPROM(32, new_pass, 64);
        writeStringToEEPROM(96, new_hostname, 32);
        writeStringToEEPROM(128, new_location, 32);
        writeStringToEEPROM(160, new_mqtt_server, 32);
        writeStringToEEPROM(192, new_mqtt_port, 8);
        EEPROM.write(200, new_mqtt_auth == "1" || new_mqtt_auth == "true" ? '1' : '0');
        writeStringToEEPROM(208, new_mqtt_user, 32);
        writeStringToEEPROM(230, new_mqtt_password, 32);

        if (EEPROM.commit()) {
            Serial.println("EEPROM successfully committed");
            // Update runtime values after saving
            load();
        } else {
            Serial.println("ERROR! EEPROM commit failed");
        }
    }

    void clear() {
        Serial.println("Clearing EEPROM...");
        for (int i = 0; i < 262; ++i) {
            EEPROM.write(i, 0);
        }
        EEPROM.commit();
    }

private:
    String readStringFromEEPROM(int addrOffset, int maxLen) {
        String str = "";
        for (int i = 0; i < maxLen; ++i) {
            char c = EEPROM.read(addrOffset + i);
            if (c == 0) break; // Null terminator
            str += c;
        }
        return str;
    }

    void writeStringToEEPROM(int addrOffset, const String& str, int maxLen) {
        int len = min((int)str.length(), maxLen - 1); // Reserve space for null terminator
        for (int i = 0; i < len; ++i) {
            EEPROM.write(addrOffset + i, str[i]);
        }
        EEPROM.write(addrOffset + len, 0); // Null terminate
    }
};

class WiFiHandler {
private:
    ConfigManager& config;
    DNSServer dnsServer;
    bool settingMode = false;
    String ssidListHtml = "";

public:
    WiFiHandler(ConfigManager& cfg) : config(cfg) {}

    bool setup() {
        WiFi.hostname(config.hostname.c_str());
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);

        if (config.ssid != "") {
            Serial.println("Attempting to connect to saved WiFi: " + config.ssid);
            WiFi.begin(config.ssid.c_str(), config.pass.c_str());
            if (waitForConnection(30)) { // Wait 15 seconds (30 * 500ms)
                Serial.println("WiFi connected!");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                setupOTA();
                settingMode = false;
                return true;
            } else {
                Serial.println("Failed to connect to saved WiFi.");
                setupAPMode();
                return false;
            }
        } else {
            Serial.println("No WiFi SSID configured.");
            setupAPMode();
            return false;
        }
    }

    void setupAPMode() {
        Serial.println("Entering WiFi Setup Mode (Access Point).");
        settingMode = true;
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
        WiFi.softAP(config.hostname.c_str()); // Use hostname or a default setup name
        dnsServer.start(53, "*", AP_IP);
        scanNetworks(); // Scan for networks to show on the config page
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        Serial.println("Connect to AP '" + config.hostname + "' to configure.");
        setupOTA(); // Enable OTA even in AP mode for potential recovery
    }

    bool waitForConnection(int maxRetries) {
        int count = 0;
        Serial.print("Waiting for Wi-Fi connection");
        while (count < maxRetries) {
            if (WiFi.status() == WL_CONNECTED) {
                Serial.println();
                return true;
            }
            delay(500);
            Serial.print(".");
            count++;
        }
        Serial.println(" Timed out.");
        return false;
    }

    void setupOTA() {
        ArduinoOTA.setHostname(config.hostname.c_str());
        ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
        ArduinoOTA.onEnd([]() { Serial.println("\nOTA End"); });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("OTA Progress: %u%%\r", (progress / (total / 100)));
        });
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("OTA Error[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });
        ArduinoOTA.begin();
        Serial.println("OTA Ready");
    }

    void handle() {
        ArduinoOTA.handle();
        if (settingMode) {
            dnsServer.processNextRequest();
        }
    }

     void scanNetworks() {
        Serial.println("Scanning for WiFi networks...");
        int n = WiFi.scanNetworks();
        Serial.println("Scan done.");
        ssidListHtml = "";
        if (n == 0) {
            Serial.println("No networks found");
        } else {
            Serial.print(n);
            Serial.println(" networks found:");
            for (int i = 0; i < n; ++i) {
                ssidListHtml += "<option value=\"";
                ssidListHtml += WiFi.SSID(i);
                ssidListHtml += "\">";
                ssidListHtml += WiFi.SSID(i);
                ssidListHtml += "</option>";
                 Serial.printf("  %d: %s (%d)%s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), (WiFi.encryptionType(i) == ENC_TYPE_NONE ? " " : "*"));
                delay(10); // Small delay between scans
            }
        }
    }

    bool isSettingMode() const { return settingMode; }
    String getSsidListHtml() const { return ssidListHtml; }
    IPAddress getLocalIp() const { return WiFi.localIP(); }
    String getSsid() const { return WiFi.SSID(); }
    int getRssi() const { return WiFi.RSSI(); }
    bool isConnected() const { return WiFi.status() == WL_CONNECTED; } // Added isConnected method
};

class DHTSensor {
private:
    DHTNEW dht;
    int pin;
    float temperature = 0.0;
    float humidity = 0.0;
    String status = "Uninitialized";
    unsigned long lastReadMillis = 0;
    long readTimeMicros = 0;
    int errorCount = 0;
    int totalErrorCount = 0;

public:
    DHTSensor(int dhtPin) : dht(dhtPin), pin(dhtPin) {}

    void setup() {
        // Optional: pinMode(pin, INPUT_PULLUP); // DHTNEW might handle this
        Serial.println("DHT Sensor initialized on pin " + String(pin));
    }

    bool read() {
        // Avoid reading too frequently
        if (millis() - lastReadMillis < 2000) {
            return false; // Not an error, just skipped read
        }

        unsigned long start = micros();
        int chk = dht.read();
        readTimeMicros = micros() - start;
        lastReadMillis = millis();

        bool success = false;
        switch (chk) {
            case DHTLIB_OK:
                status = "OK";
                temperature = dht.getTemperature();
                humidity = dht.getHumidity();
                // Basic sanity check
                if (temperature < -40 || temperature > 80 || humidity < 0 || humidity > 100) {
                     status = "Invalid Data";
                     errorCount++;
                     totalErrorCount++;
                } else {
                    errorCount = 0; // Reset error count on successful read
                    success = true;
                }
                break;
            case DHTLIB_ERROR_CHECKSUM:
                status = "Checksum Error";
                errorCount++;
                totalErrorCount++;
                break;
            case DHTLIB_ERROR_TIMEOUT_C: // Corrected constant name
                 status = "Timeout Error";
                 errorCount++;
                 totalErrorCount++;
                 break;
            default:
                status = "Unknown Error";
                errorCount++;
                totalErrorCount++;
                break;
        }

        if (errorCount >= DHT_ERROR_COUNT_MAX) {
            Serial.println("DHT consecutive error limit reached!");
            // Consider adding reboot logic here if desired, maybe via controller callback
        }
         if (totalErrorCount >= DHT_TOTAL_ERROR_COUNT_MAX) {
            Serial.println("DHT total error limit reached!");
            // Consider adding reboot logic here if desired
        }

        // Serial.printf("DHT Read: Status=%s, Temp=%.1f, Hum=%.1f, Time=%ldus, ErrCnt=%d, TotalErr=%d\n",
        //               status.c_str(), temperature, humidity, readTimeMicros, errorCount, totalErrorCount);

        return success;
    }

    float getTemperature() const { return temperature; }
    float getHumidity() const { return humidity; }
    String getStatus() const { return status; }
    int getErrorCount() const { return errorCount; }
    int getTotalErrorCount() const { return totalErrorCount; }
    long getReadTimeMicros() const { return readTimeMicros; }
    bool isReadingValid() const { return status == "OK"; } // More specific check
};

class Button {
private:
    int pin;
    bool lastReading = HIGH;
    unsigned long lastDebounceTime = 0;
    bool currentState = HIGH; // HIGH = not pressed

public:
    Button(int buttonPin) : pin(buttonPin) {}

    void setup() {
        pinMode(pin, INPUT_PULLUP);
    }

    // Returns true only on the transition from HIGH to LOW (pressed)
    bool checkPressed() {
        bool reading = digitalRead(pin);
        bool pressedEvent = false;

        if (reading != lastReading) {
            lastDebounceTime = millis();
        }

        if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
            if (reading != currentState) {
                currentState = reading;
                if (currentState == LOW) { // Button was just pressed
                    pressedEvent = true;
                }
            }
        }

        lastReading = reading;
        return pressedEvent;
    }
};

class RelaySwitch {
private:
    int pin;
    bool stateOn = false; // false = OFF (HIGH), true = ON (LOW)

public:
    RelaySwitch(int switchPin) : pin(switchPin) {}

    void setup() {
        pinMode(pin, OUTPUT);
        off(); // Default state is off
    }

    void on() {
        digitalWrite(pin, LOW);
        stateOn = true;
        Serial.println("Switch Pin " + String(pin) + " ON (LOW)");
    }

    void off() {
        digitalWrite(pin, HIGH);
        stateOn = false;
        Serial.println("Switch Pin " + String(pin) + " OFF (HIGH)");
    }

    void setState(bool turnOn) {
        if (turnOn) {
            on();
        } else {
            off();
        }
    }

    bool isOn() const {
        return stateOn;
    }

    // Returns 1 for ON, 0 for OFF (for compatibility with original logic)
    int getStateInt() const {
        return stateOn ? 1 : 0;
    }
};

class MQTTHandler {
private:
    PubSubClient client;
    ConfigManager& config;
    // No more reference to AirPumpController
    long lastReconnectAttempt = 0;
    const long reconnectInterval = 5000;
    bool autoConfGenerated = false;
    String logBuffer = ""; // Simple log buffer

    // Topic Strings
    String identifier = "";
    String baseTopic = "";
    String topicAvailability = "";
    String topicState = "";
    String topicCommand = ""; // General command topic
    String topicLampCommand = "";
    String topicFanCommand = "";
    String topicFanState = ""; // For reporting fan state separately if needed
    String topicDebug = "";
    String topicJsonReport = "";

    // Auto-discovery Topics & Payloads
    String topicAutoConfHumidity = ""; String payloadAutoConfHumidity = "";
    String topicAutoConfTemp = ""; String payloadAutoConfTemp = "";
    String topicAutoConfWifi = ""; String payloadAutoConfWifi = "";
    String topicAutoConfFan = ""; String payloadAutoConfFan = "";
    String topicAutoConfLamp = ""; String payloadAutoConfLamp = "";

    void log(const String& message) {
        Serial.println("[MQTT] " + message);
        logBuffer += message + "\n";
        if (logBuffer.length() > 1024) { // Limit buffer size
            logBuffer = logBuffer.substring(logBuffer.length() - 1024);
        }
    }

public:
    // Constructor no longer needs controller reference
    MQTTHandler(WiFiClient& wc, ConfigManager& cfg) :
        client(wc), config(cfg) {
    }

    void setup() {
        // Use global callback function
        client.setCallback(mqttCallbackFunction);
        
        // ...rest of existing setup code...
        if (config.mqtt_server == "" || config.mqtt_port == "") {
            log("MQTT server or port not configured. MQTT disabled.");
            return;
        }

        identifier = config.hostname; // Use hostname as unique identifier
        baseTopic = config.location + "/" + identifier;
        topicAvailability = baseTopic + "/status";
        topicState = baseTopic + "/state";
        topicCommand = baseTopic + "/command"; // Main command topic
        topicLampCommand = baseTopic + "/lamp/command"; // Specific lamp command
        topicFanCommand = baseTopic + "/fan/command";   // Specific fan command
        topicFanState = baseTopic + "/fan/state";     // Specific fan state reporting
        topicDebug = "esp/debug/" + identifier;
        topicJsonReport = "json/sensors/airpump/" + config.location; // Original JSON report topic

        log("Setting MQTT server: " + config.mqtt_server + ":" + config.mqtt_port);
        client.setServer(config.mqtt_server.c_str(), config.mqtt_port.toInt());
        client.setBufferSize(MQTT_MAX_PACKET_SIZE);
    }

    bool connect() {
        if (config.mqtt_server == "" || config.mqtt_port == "") return false; // Don't try if not configured

        if (client.connected()) {
            return true;
        }

        long now = millis();
        if (now - lastReconnectAttempt > reconnectInterval) {
            lastReconnectAttempt = now;
            log("Attempting MQTT connection...");
            String clientId = config.hostname + "-" + String(random(0xffff), HEX);

            bool connected = false;
            if (config.mqtt_auth && config.mqtt_user != "") {
                log("Connecting with username: " + config.mqtt_user);
                connected = client.connect(clientId.c_str(), config.mqtt_user.c_str(), config.mqtt_password.c_str(),
                                           topicAvailability.c_str(), 0, true, "offline");
            } else {
                log("Connecting without authentication.");
                connected = client.connect(clientId.c_str(), topicAvailability.c_str(), 0, true, "offline");
            }

            if (connected) {
                log("MQTT Connected!");
                publish(topicAvailability.c_str(), "online", true);
                subscribe(topicCommand.c_str());
                subscribe(topicLampCommand.c_str());
                subscribe(topicFanCommand.c_str());
                // Subscribe to legacy topics if needed for compatibility?
                // subscribe(String("esp/"+config.hostname+"/"+config.location+"/switch/fan").c_str());
                // subscribe(String("esp/"+config.hostname+"/"+config.location+"/switch/lamp").c_str());
                // subscribe("esp/airpump/cmd/reboot"); // Example legacy topic

                if (!autoConfGenerated) {
                    generateAutoDiscoveryConfig();
                    publishAutoDiscoveryConfig();
                }
                return true;
            } else {
                log("MQTT Connection Failed, rc=" + String(client.state()));
                // Print specific error message based on client.state()
                return false;
            }
        }
        return false; // Not time to retry yet
    }

    void loop() {
        if (config.mqtt_server == "" || config.mqtt_port == "") return; // Don't loop if not configured

        if (!client.connected()) {
            connect(); // Attempt reconnect if disconnected
        } else {
            client.loop(); // Process MQTT messages
        }
    }

    bool publish(const char* topic, const char* payload, bool retained = false) {
        if (!client.connected()) {
            log("Cannot publish, MQTT not connected.");
            return false;
        }
        // log("Publishing to " + String(topic) + ": " + String(payload));
        return client.publish(topic, payload, retained);
    }

     bool publish(const String& topic, const String& payload, bool retained = false) {
        return publish(topic.c_str(), payload.c_str(), retained);
     }

    bool subscribe(const char* topic) {
        if (!client.connected()) {
            log("Cannot subscribe, MQTT not connected.");
            return false;
        }
        log("Subscribing to: " + String(topic));
        return client.subscribe(topic);
    }

    void generateAutoDiscoveryConfig() {
        log("Generating Home Assistant Auto-Discovery config...");
        // DynamicJsonDocument deviceDoc(256); // Deprecated
        JsonDocument deviceDoc; // Use JsonDocument
        deviceDoc["identifiers"] = identifier;
        deviceDoc["name"] = config.hostname; // Or a more descriptive name like "Kitchen Air Pump"
        deviceDoc["manufacturer"] = "DIY";
        deviceDoc["model"] = "ESP8266 AirPump+Light";
        deviceDoc["sw_version"] = VERSION;
        deviceDoc["configuration_url"] = "http://" + IpAddress2String(WiFi.localIP());

        JsonObject device = deviceDoc.as<JsonObject>();

        // --- Humidity Sensor ---
        topicAutoConfHumidity = "homeassistant/sensor/" + baseTopic + "_humidity/config";
        // DynamicJsonDocument humDoc(512); // Deprecated
        JsonDocument humDoc; // Use JsonDocument
        humDoc["device"] = device;
        humDoc["name"] = config.hostname + " Humidity";
        humDoc["unique_id"] = identifier + "_humidity";
        humDoc["state_topic"] = topicState;
        humDoc["availability_topic"] = topicAvailability;
        humDoc["device_class"] = "humidity";
        humDoc["unit_of_measurement"] = "%";
        humDoc["value_template"] = "{{ value_json.humidity | default(0) }}";
        serializeJson(humDoc, payloadAutoConfHumidity);

        // --- Temperature Sensor ---
        topicAutoConfTemp = "homeassistant/sensor/" + baseTopic + "_temperature/config";
        // DynamicJsonDocument tempDoc(512); // Deprecated
        JsonDocument tempDoc; // Use JsonDocument
        tempDoc["device"] = device;
        tempDoc["name"] = config.hostname + " Temperature";
        tempDoc["unique_id"] = identifier + "_temperature";
        tempDoc["state_topic"] = topicState;
        tempDoc["availability_topic"] = topicAvailability;
        tempDoc["device_class"] = "temperature";
        tempDoc["unit_of_measurement"] = "°C";
        tempDoc["value_template"] = "{{ value_json.temperature | default(0) }}";
        serializeJson(tempDoc, payloadAutoConfTemp);

        // --- WiFi Sensor ---
        topicAutoConfWifi = "homeassistant/sensor/" + baseTopic + "_wifi/config";
        // DynamicJsonDocument wifiDoc(768); // Deprecated
        JsonDocument wifiDoc; // Use JsonDocument
        wifiDoc["device"] = device;
        wifiDoc["name"] = config.hostname + " WiFi Signal";
        wifiDoc["unique_id"] = identifier + "_wifi_signal";
        wifiDoc["state_topic"] = topicState;
        wifiDoc["availability_topic"] = topicAvailability;
        wifiDoc["device_class"] = "signal_strength";
        wifiDoc["unit_of_measurement"] = "dBm";
        wifiDoc["value_template"] = "{{ value_json.wifi.rssi | default(0) }}";
        wifiDoc["json_attributes_topic"] = topicState;
        wifiDoc["json_attributes_template"] = "{{ {'ip': value_json.wifi.ip, 'ssid': value_json.wifi.ssid} | tojson }}";
        wifiDoc["icon"] = "mdi:wifi";
        wifiDoc["entity_category"] = "diagnostic";
        serializeJson(wifiDoc, payloadAutoConfWifi);

        // --- Lamp Switch ---
        topicAutoConfLamp = "homeassistant/switch/" + baseTopic + "_lamp/config";
        // DynamicJsonDocument lampDoc(512); // Deprecated
        JsonDocument lampDoc; // Use JsonDocument
        lampDoc["device"] = device;
        lampDoc["name"] = config.hostname + " Light";
        lampDoc["unique_id"] = identifier + "_lamp";
        lampDoc["state_topic"] = topicState;
        lampDoc["command_topic"] = topicLampCommand;
        lampDoc["availability_topic"] = topicAvailability;
        lampDoc["payload_on"] = "ON"; // Use "ON"/"OFF" commands
        lampDoc["payload_off"] = "OFF";
        lampDoc["state_on"] = "ON";   // Expect "ON"/"OFF" in state
        lampDoc["state_off"] = "OFF";
        lampDoc["value_template"] = "{{ value_json.light | default('OFF') }}"; // Extract light state
        lampDoc["icon"] = "mdi:lightbulb";
        serializeJson(lampDoc, payloadAutoConfLamp);

        // --- Fan Control (using Fan component) ---
        topicAutoConfFan = "homeassistant/fan/" + baseTopic + "_fan/config";
        // DynamicJsonDocument fanDoc(1024); // Deprecated
        JsonDocument fanDoc; // Use JsonDocument
        fanDoc["device"] = device;
        fanDoc["name"] = config.hostname + " Fan";
        fanDoc["unique_id"] = identifier + "_fan";
        fanDoc["state_topic"] = topicState; // Get state from main state topic
        fanDoc["command_topic"] = topicFanCommand; // Send commands here
        fanDoc["availability_topic"] = topicAvailability;
        fanDoc["state_value_template"] = "{% if value_json.fan_speed > 0 %}ON{% else %}OFF{% endif %}"; // Determine ON/OFF state
        fanDoc["percentage_state_topic"] = topicState;
        fanDoc["percentage_value_template"] = "{{ value_json.fan_percentage | default(0) }}"; // Use percentage
        fanDoc["percentage_command_topic"] = topicFanCommand;
        fanDoc["percentage_command_template"] = "{{ value }}"; // Send percentage directly
        fanDoc["speed_range_min"] = 1; // Assuming speed 1 maps to lowest percentage > 0
        fanDoc["speed_range_max"] = 100; // Map speeds to percentage range
        // Optional: Add presets if you want named speeds in HA
        // JsonArray presets = fanDoc.createNestedArray("preset_modes");
        // presets.add("off"); presets.add("low"); presets.add("medium"); presets.add("high");
        // fanDoc["preset_mode_state_topic"] = topicState;
        // fanDoc["preset_mode_value_template"] = "{{ value_json.fan_mode }}"; // e.g., "low", "medium"
        // fanDoc["preset_mode_command_topic"] = topicFanCommand;
        // fanDoc["preset_mode_command_template"] = "{{ value }}"; // Send preset name
        serializeJson(fanDoc, payloadAutoConfFan);

        autoConfGenerated = true;
        log("Auto-Discovery config generated.");
    }

    void publishAutoDiscoveryConfig() {
        if (!autoConfGenerated) {
            log("Cannot publish Auto-Discovery, not generated yet.");
            return;
        }
        log("Publishing Auto-Discovery config...");
        publish(topicAutoConfHumidity, payloadAutoConfHumidity, true);
        publish(topicAutoConfTemp, payloadAutoConfTemp, true);
        publish(topicAutoConfWifi, payloadAutoConfWifi, true);
        publish(topicAutoConfLamp, payloadAutoConfLamp, true);
        publish(topicAutoConfFan, payloadAutoConfFan, true);
        log("Auto-Discovery config published.");
    }

    // Publishes the main state JSON
    bool publishState(int fanSpeed, bool lightState, float temp, float hum, const String& wifi_ssid, const IPAddress& wifi_ip, int wifi_rssi) {
        // DynamicJsonDocument stateDoc(512); // Deprecated
        JsonDocument stateDoc; // Use JsonDocument
        stateDoc["temperature"] = round(temp * 10) / 10.0; // One decimal place
        stateDoc["humidity"] = round(hum * 10) / 10.0;     // One decimal place
        stateDoc["light"] = lightState ? "ON" : "OFF";
        stateDoc["fan_speed"] = fanSpeed; // Report raw speed 0-3

        // Map speed to percentage for HA fan component
        int percentage = 0;
        if (fanSpeed == 1) percentage = 33;
        else if (fanSpeed == 2) percentage = 66;
        else if (fanSpeed == 3) percentage = 100;
        stateDoc["fan_percentage"] = percentage;
        stateDoc["fan_state"] = (fanSpeed > 0) ? "ON" : "OFF"; // Explicit ON/OFF state

        // Optional: Add preset mode if using presets in HA config
        // String mode = "off";
        // if (fanSpeed == 1) mode = "low";
        // else if (fanSpeed == 2) mode = "medium";
        // else if (fanSpeed == 3) mode = "high";
        // stateDoc["fan_mode"] = mode;

        // JsonObject wifiObj = stateDoc.createNestedObject("wifi"); // Deprecated
        JsonObject wifiObj = stateDoc["wifi"].to<JsonObject>(); // New syntax
        wifiObj["ssid"] = wifi_ssid;
        wifiObj["ip"] = IpAddress2String(wifi_ip);
        wifiObj["rssi"] = wifi_rssi;

        String payload;
        serializeJson(stateDoc, payload);
        return publish(topicState, payload, false); // State is not usually retained
    }

     // Publishes the legacy JSON report
    bool publishJsonReport(int fanSpeed, bool lightState, const DHTSensor& dht, bool manualFan, unsigned long currentMillis, const String& wifi_ssid, const IPAddress& wifi_ip, int wifi_rssi) {
        // DynamicJsonDocument doc(1024); // Deprecated
        JsonDocument doc; // Use JsonDocument

        doc["device"] = config.hostname;
        doc["name"]   = config.hostname;
        doc["device_ip"] = IpAddress2String(wifi_ip);
        doc["location"] = config.location;

        doc["dht22_temp"] = round(dht.getTemperature() * 10) / 10.0;
        doc["dht22_hum"] = round(dht.getHumidity() * 10) / 10.0;
        doc["dht22_ok"] = dht.isReadingValid();
        doc["dht22_error"] = dht.getErrorCount();
        doc["dht22_timing"] = dht.getReadTimeMicros(); // Report micros now
        doc["dht22_error_total"] = dht.getTotalErrorCount();
        doc["dht22_status"] = dht.getStatus();

        doc["manual_fan"] = manualFan;
        doc["fan"] = fanSpeed;
        doc["light"] = lightState ? 1 : 0; // Use 1/0 for legacy compatibility

        doc["millis"] = currentMillis;
        doc["version"] = VERSION;

        // Add WiFi details to legacy report too
        doc["wifi_ssid"] = wifi_ssid;
        doc["wifi_rssi"] = wifi_rssi;

        String buffer;
        serializeJson(doc, buffer); // Removed unused 'n' variable
        return publish(topicJsonReport.c_str(), buffer.c_str(), false); // Not retained
    }


    bool isConnected() /* removed const */ {
        return client.connected();
    }

    String getLog() const {
        return logBuffer;
    }

    void clearLog() {
        logBuffer = "";
    }

    String getAvailabilityTopic() const { return topicAvailability; }
    String getStateTopic() const { return topicState; }
    String getCommandTopic() const { return topicCommand; }
    String getLampCommandTopic() const { return topicLampCommand; }
    String getFanCommandTopic() const { return topicFanCommand; }
    String getJsonReportTopic() const { return topicJsonReport; }
};


class WebServerHandler {
private:
    ESP8266WebServer server;
    ConfigManager& config;
    WiFiHandler& wifi;
    MQTTHandler& mqtt;
    DHTSensor& dht;
    
    // Store values directly instead of pointer to controller
    int currentFanSpeed = 0;
    bool currentLightState = false;
    ControllerInterface* controller = nullptr; // Interface pointer

    String makeMenu() {
        String s = "<nav><ul class=\"primary\">";
        s += makeLink("/", "Main", false); // Simplified menu
        if (!wifi.isSettingMode()) {
            s += makeLink("/control", "Control", false);
            s += makeLink("/debug", "Debug", false);
        }
        s += makeLink("/settings", "Settings", false);
        s += makeLink("/reboot", "Reboot", false);
        if (wifi.isSettingMode()) {
             s += makeLink("/reset", "Reset WiFi", false);
        } else {
             s += makeLink("/reset", "Reset All", false);
        }
        s += "</ul></nav>";
        return s;
    }

    String makeLink(const String& href, const String& name, bool isDropdown) {
        // Simplified link generation
        return "<li><a href=\"" + href + "?t=" + String(millis()) + "\">" + name + "</a></li>";
    }

    String makePage(const String& title, const String& contents) {
        String s = "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\"><title>";
        s += config.hostname + " - " + title;
        s += "</title><style>";
        // Basic CSS (condensed from original)
        s += "body{font-family:sans-serif;background:#eee;margin:0;}";
        s += "nav ul{list-style:none;padding:0;margin:0;background:#333;} nav li{display:inline-block;} nav a{display:block;padding:10px 15px;color:#fff;text-decoration:none;} nav a:hover{background:#555;}";
        s += ".wrap{max-width:800px;margin:20px auto;padding:20px;background:#fff;box-shadow:0 0 10px rgba(0,0,0,0.1);}";
        s += "h1{color:#333;} pre{background:#f4f4f4;padding:10px;border:1px solid #ddd;overflow-x:auto;}";
        s += "label{display:block;margin-top:10px;} input[type=text],input[type=password],select{width:100%;padding:8px;margin-top:5px;box-sizing:border-box;}";
        s += "input[type=submit]{background:#333;color:#fff;padding:10px 15px;border:none;cursor:pointer;margin-top:15px;}";
        s += "footer{text-align:center;margin-top:20px;font-size:0.8em;color:#777;}";
        s += "</style>";
        // Add jQuery if needed for specific pages like debug
         if (title.indexOf("Debug") != -1) {
             s += "<script src=\"https://code.jquery.com/jquery-3.6.0.min.js\"></script>";
             s += "<script>$(document).ready(function(){var b=$('#jsonbuffer');if(b.length){try{var j=JSON.parse(b.text());b.text(JSON.stringify(j,null,2));}catch(e){console.error('Error parsing JSON:',e);}}});</script>";
         }
        s += "</head><body>";
        s += makeMenu();
        s += "<div class=\"wrap\"><h1>" + title + "</h1>";
        s += contents;
        s += "<footer>";
        s += "Version: " + String(VERSION) + " | Host: " + config.hostname + " | Location: " + config.location + "<br>";
        s += "Uptime: " + uptime_formatter::getUptime() + "<br>";
        s += "WiFi: " + (wifi.isConnected() ? wifi.getSsid() + " (" + String(wifi.getRssi()) + " dBm) - " + IpAddress2String(wifi.getLocalIp()) : (wifi.isSettingMode() ? "Setup Mode AP" : "Disconnected")) + "<br>"; // Use wifi.isConnected()
        s += String("MQTT: ") + String(mqtt.isConnected() ? "Connected" : "Disconnected") + "<br>"; // Fix string concatenation
        s += "DHT Status: " + dht.getStatus() + " (T: " + String(dht.getTemperature(), 1) + "C, H: " + String(dht.getHumidity(), 1) + "%)<br>";
        s += "</footer>";
        s += "</div></body></html>";
        return s;
    }

    // --- Route Handlers ---
    void handleRoot() {
        String s = "<h2>Device Status</h2>";
        s += "<p>Welcome to the " + config.hostname + " control panel.</p>";
        s += "<p>Current Fan Speed: " + String(currentFanSpeed) + "</p>"; // Use stored value
        s += String("<p>Light Status: ") + String(currentLightState ? "ON" : "OFF") + "</p>"; // Use stored value
        s += "<p>Temperature: " + String(dht.getTemperature(), 1) + " &deg;C</p>";
        s += "<p>Humidity: " + String(dht.getHumidity(), 1) + " %</p>";
        if (wifi.isSettingMode()) {
            s += "<p><b>Device is in Setup Mode.</b> Connect to the '" + config.hostname + "' WiFi network and go to <a href='/settings'>Settings</a> to configure.</p>";
        }
        server.send(200, "text/html", makePage("Main", s));
    }

    void handleControl() {
         if (wifi.isSettingMode()) { handleRoot(); return; } // Redirect if in setup mode

        String s = "<h2>Manual Control</h2>";
        s += "<form method='get' action='/set_control'>";
        // Fan Control
        s += "<label for='fan_speed'>Fan Speed:</label>";
        s += "<select name='fan_speed' id='fan_speed'>";
        s += "<option value='0'" + String(currentFanSpeed == 0 ? " selected" : "") + ">Off</option>"; // Use stored value
        s += "<option value='1'" + String(currentFanSpeed == 1 ? " selected" : "") + ">Speed 1</option>";
        s += "<option value='2'" + String(currentFanSpeed == 2 ? " selected" : "") + ">Speed 2</option>";
        s += "<option value='3'" + String(currentFanSpeed == 3 ? " selected" : "") + ">Speed 3</option>";
        s += "</select><br>";
        // Light Control
        s += "<label for='light_state'>Light:</label>";
        s += "<select name='light_state' id='light_state'>";
        s += "<option value='0'" + String(!currentLightState ? " selected" : "") + ">Off</option>"; // Use stored value
        s += "<option value='1'" + String(currentLightState ? " selected" : "") + ">On</option>";
        s += "</select><br>";
        s += "<input type='submit' value='Set Control'>";
        s += "</form>";
        server.send(200, "text/html", makePage("Control", s));
    }

     void handleSetControl() {
        if (wifi.isSettingMode()) { handleRoot(); return; }

        int fanSpeed = server.arg("fan_speed").toInt();
        bool lightState = server.arg("light_state").toInt() == 1;

        // Update stored values
        currentFanSpeed = fanSpeed;
        currentLightState = lightState;
        
        // Use interface to update controller
        if (controller) {
            controller->setFanSpeed(fanSpeed, true);
            controller->setLight(lightState);
            controller->reportState();
        }

        String s = "<h2>Control Updated</h2>";
        s += "<p>Fan speed set to: " + String(fanSpeed) + "</p>";
        s += String("<p>Light state set to: ") + String(lightState ? "ON" : "OFF") + "</p>"; // Fix string concatenation 
        s += "<p><a href='/control'>Back to Control</a></p>";
        s += "<p><a href='/'>Back to Main</a></p>";
        server.send(200, "text/html", makePage("Control Set", s));
    }


    void handleSettings() {
        String s = "<h2>Configuration Settings</h2>";
        s += "<form method='get' action='/save_settings'>";

        // WiFi Settings
        s += "<h3>WiFi</h3>";
        s += "<label for='ssid'>SSID:</label>";
        if (wifi.isSettingMode()) {
             s += "<select name='ssid' id='ssid'>" + wifi.getSsidListHtml() + "</select><br>";
        } else {
             s += "<input type='text' name='ssid' id='ssid' value='" + config.ssid + "'><br>";
        }
        s += "<label for='pass'>Password:</label>";
        s += "<input type='password' name='pass' id='pass' value='" + config.pass + "'><br>";

        // Device Settings
        s += "<h3>Device</h3>";
        s += "<label for='hostname'>Hostname:</label>";
        s += "<input type='text' name='hostname' id='hostname' value='" + config.hostname + "'><br>";
        s += "<label for='location'>Location (for MQTT topic):</label>";
        s += "<input type='text' name='location' id='location' value='" + config.location + "'><br>";

        // MQTT Settings
        s += "<h3>MQTT</h3>";
        s += "<label for='mqtt_server'>Server:</label>";
        s += "<input type='text' name='mqtt_server' id='mqtt_server' value='" + config.mqtt_server + "'><br>";
        s += "<label for='mqtt_port'>Port:</label>";
        s += "<input type='text' name='mqtt_port' id='mqtt_port' value='" + config.mqtt_port + "'><br>";
        s += "<label for='mqtt_auth'>Use Authentication:</label>";
        s += "<select name='mqtt_auth' id='mqtt_auth'>";
        s += "<option value='false'" + String(!config.mqtt_auth ? " selected" : "") + ">No</option>";
        s += "<option value='true'" + String(config.mqtt_auth ? " selected" : "") + ">Yes</option>";
        s += "</select><br>";
        s += "<label for='mqtt_user'>Username:</label>";
        s += "<input type='text' name='mqtt_user' id='mqtt_user' value='" + config.mqtt_user + "'><br>";
        s += "<label for='mqtt_password'>Password:</label>";
        s += "<input type='password' name='mqtt_password' id='mqtt_password' value='" + config.mqtt_password + "'><br>";

        s += "<input type='submit' value='Save Settings & Reboot'>";
        s += "</form>";
        server.send(200, "text/html", makePage("Settings", s));
    }

    void handleSaveSettings() {
        String ssid_val = urlDecode(server.arg("ssid"));
        String pass_val = urlDecode(server.arg("pass"));
        String host_val = urlDecode(server.arg("hostname"));
        String loc_val = urlDecode(server.arg("location"));
        String mqtt_srv_val = urlDecode(server.arg("mqtt_server"));
        String mqtt_port_val = urlDecode(server.arg("mqtt_port"));
        String mqtt_auth_val = urlDecode(server.arg("mqtt_auth"));
        String mqtt_user_val = urlDecode(server.arg("mqtt_user"));
        String mqtt_pass_val = urlDecode(server.arg("mqtt_password"));

        config.save(ssid_val, pass_val, host_val, loc_val, mqtt_srv_val, mqtt_port_val, mqtt_auth_val, mqtt_user_val, mqtt_pass_val);

        String s = "<h2>Settings Saved</h2>";
        s += "<p>Configuration has been saved to EEPROM.</p>";
        s += "<p>The device will now reboot to apply the changes.</p>";
        server.send(200, "text/html", makePage("Settings Saved", s));
        delay(2000);
        ESP.restart();
    }

    void handleReset() {
        config.clear();
        String s = "<h2>Configuration Reset</h2>";
        s += "<p>All settings have been cleared from EEPROM.</p>";
        s += "<p>The device will now reboot into Setup Mode.</p>";
        server.send(200, "text/html", makePage("Reset", s));
        delay(2000);
        ESP.restart();
    }

    void handleReboot() {
        String s = "<h2>Rebooting</h2>";
        s += "<p>The device will reboot shortly.</p>";
        server.send(200, "text/html", makePage("Reboot", s));
        delay(1000);
        ESP.restart();
    }

     void handleDebug() {
        if (wifi.isSettingMode()) { handleRoot(); return; }

        String s = "<h2>Debug Information</h2>";
        s += "<h3>MQTT</h3>";
        s += "<p>Status: " + String(mqtt.isConnected() ? "Connected" : "Disconnected") + "</p>"; // Use isConnected()
        s += "<p>Availability Topic: " + mqtt.getAvailabilityTopic() + "</p>";
        s += "<p>State Topic: " + mqtt.getStateTopic() + "</p>";
        s += "<p>Command Topic: " + mqtt.getCommandTopic() + "</p>";
        s += "<p>Lamp Command Topic: " + mqtt.getLampCommandTopic() + "</p>";
        s += "<p>Fan Command Topic: " + mqtt.getFanCommandTopic() + "</p>";
        s += "<p>JSON Report Topic: " + mqtt.getJsonReportTopic() + "</p>";
        s += "<h4>MQTT Log (<a href='/debug_clearlog'>Clear</a>)</h4>";
        s += "<pre>" + mqtt.getLog() + "</pre>";

        s += "<h3>DHT Sensor</h3>";
        s += "<p>Status: " + dht.getStatus() + "</p>";
        s += "<p>Temperature: " + String(dht.getTemperature(), 2) + " C</p>";
        s += "<p>Humidity: " + String(dht.getHumidity(), 2) + " %</p>";
        s += "<p>Last Read Time: " + String(dht.getReadTimeMicros()) + " us</p>";
        s += "<p>Consecutive Errors: " + String(dht.getErrorCount()) + "</p>";
        s += "<p>Total Errors: " + String(dht.getTotalErrorCount()) + "</p>";

        s += "<h3>System</h3>";
        s += "<p>Free Heap: " + String(ESP.getFreeHeap()) + " bytes</p>";
        s += "<p>Chip ID: " + String(ESP.getChipId(), HEX) + "</p>";
        s += "<p>SDK Version: " + String(ESP.getSdkVersion()) + "</p>";

        server.send(200, "text/html", makePage("Debug", s));
    }

     void handleDebugClearLog() {
        mqtt.clearLog();
        // Redirect back to debug page
        server.sendHeader("Location", "/debug", true);
        server.send(302, "text/plain", "");
    }


    void handleNotFound() {
        if (wifi.isSettingMode()) {
            // Captive portal redirection
            server.sendHeader("Location", String("http://") + IpAddress2String(wifi.getLocalIp()), true);
            server.send(302, "text/plain", ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
        } else {
            server.send(404, "text/plain", "Not Found");
        }
    }


public:
    WebServerHandler(ConfigManager& cfg, WiFiHandler& wf, MQTTHandler& mq, DHTSensor& dh) :
        server(80), config(cfg), wifi(wf), mqtt(mq), dht(dh) {}

    void setup() {
        // Common routes
        server.on("/", HTTP_GET, [this]() { this->handleRoot(); });
        server.on("/settings", HTTP_GET, [this]() { this->handleSettings(); });
        server.on("/save_settings", HTTP_GET, [this]() { this->handleSaveSettings(); });
        server.on("/reboot", HTTP_GET, [this]() { this->handleReboot(); });
        server.on("/reset", HTTP_GET, [this]() { this->handleReset(); });

        if (!wifi.isSettingMode()) {
            // Routes only available in normal operating mode
            server.on("/control", HTTP_GET, [this]() { this->handleControl(); });
            server.on("/set_control", HTTP_GET, [this]() { this->handleSetControl(); });
            server.on("/debug", HTTP_GET, [this]() { this->handleDebug(); });
            server.on("/debug_clearlog", HTTP_GET, [this]() { this->handleDebugClearLog(); });
        }

        server.onNotFound([this]() { this->handleNotFound(); });

        server.begin();
        Serial.println("Web Server started.");
    }

    void handleClient() {
        server.handleClient();
    }
    
    // Method to update stored values from controller
    void updateValues(int fanSpeed, bool lightState) {
        currentFanSpeed = fanSpeed;
        currentLightState = lightState;
    }
    
    // Method to set controller interface
    void setController(ControllerInterface* ctrl) {
        controller = ctrl;
    }
};


// --- Main Application Controller ---
// Defined AFTER MQTTHandler and WebServerHandler
class AirPumpController : public ControllerInterface {
private:
    ConfigManager configManager;
    WiFiHandler wifiHandler;
    DHTSensor dhtSensor;
    MQTTHandler mqttHandler;
    WebServerHandler webServerHandler;

    Button buttonFanOff; // Button 1
    Button buttonFan1;   // Button 2
    Button buttonFan2;   // Button 3
    Button buttonFan3;   // Button 4
    Button buttonLight;  // Button 5

    RelaySwitch switchFan1;
    RelaySwitch switchFan2;
    RelaySwitch switchFan3;
    RelaySwitch switchLight;

    int currentFanSpeed = 0; // 0=Off, 1, 2, 3
    bool currentLightState = false; // false=Off, true=On
    bool manualFanMode = true; // Start in manual mode

    unsigned long lastReportTime = 0;
    unsigned long lastSensorReadTime = 0;

    // Variables for auto mode (example)
    const int avgSamples = 5;
    float humiditySamples[5]; // Moved array declaration inside the class
    int sampleIndex = 0;

    // Process received MQTT messages
    void processMqttMessages() {
        if (newMqttMessage) {
            String topic = mqttTopicBuffer;
            String payload = mqttPayloadBuffer;
            
            // Process message - similar to old mqttCallbackDelegate
            if (topic == mqttHandler.getLampCommandTopic()) {
                if (payload == "ON") {
                    setLight(true);
                } else if (payload == "OFF") {
                    setLight(false);
                }
                reportState();
            }
            else if (topic == mqttHandler.getFanCommandTopic()) {
                // Check for percentage command
                int percentage = payload.toInt();
                bool isNumeric = true;
                for (unsigned int i = 0; i < payload.length(); i++) {
                    if (!isDigit(payload[i])) {
                        isNumeric = false;
                        break;
                    }
                }

                if (isNumeric) {
                    int targetSpeed = 0;
                    if (percentage == 0) targetSpeed = 0;
                    else if (percentage <= 33) targetSpeed = 1;
                    else if (percentage <= 66) targetSpeed = 2;
                    else targetSpeed = 3;
                    setFanSpeed(targetSpeed, true);
                }
                else if (!isNumeric) {
                    if (payload == "0") setFanSpeed(0, true);
                    else if (payload == "1") setFanSpeed(1, true);
                    else if (payload == "2") setFanSpeed(2, true);
                    else if (payload == "3") setFanSpeed(3, true);
                }
                reportState();
            }
            // Reset flag
            newMqttMessage = false;
        }
    }

public:
    // Constructor initializes members
    AirPumpController() :
        wifiHandler(configManager),
        dhtSensor(DHT22_PIN),
        mqttHandler(wifiClient, configManager),
        webServerHandler(configManager, wifiHandler, mqttHandler, dhtSensor),
        buttonFanOff(BUTTON1_PIN), buttonFan1(BUTTON2_PIN), buttonFan2(BUTTON3_PIN),
        buttonFan3(BUTTON4_PIN), buttonLight(BUTTON5_PIN),
        switchFan1(SWITCH1_PIN), switchFan2(SWITCH2_PIN), switchFan3(SWITCH3_PIN),
        switchLight(SWITCH4_PIN)
    {
        controllerInstance = this; // Set the global pointer
        webServerHandler.setController(this); // Pass interface to web server
    }

    // AirPumpController implementation of ControllerInterface methods
    virtual void setFanSpeed(int speed, bool manual = false) override {
        // Existing implementation
        if (speed < 0 || speed > 3) return;

        if (manual) {
            manualFanMode = true;
            Serial.println("Fan mode set to MANUAL");
        }

        if (speed != currentFanSpeed || manual) {
            Serial.println("Setting Fan Speed to " + String(speed) + (manual ? " (Manual)" : ""));
            currentFanSpeed = speed;

            switchFan1.off();
            switchFan2.off();
            switchFan3.off();
            delay(50);

            if (speed == 1) switchFan1.on();
            else if (speed == 2) switchFan2.on();
            else if (speed == 3) switchFan3.on();
            
            // Update WebServerHandler's stored values
            webServerHandler.updateValues(currentFanSpeed, currentLightState);
        }
    }

    virtual void setLight(bool state) override {
        // Existing implementation
        if (state != currentLightState) {
            Serial.println("Setting Light to " + String(state ? "ON" : "OFF"));
            currentLightState = state;
            switchLight.setState(state);
            
            // Update WebServerHandler's stored values
            webServerHandler.updateValues(currentFanSpeed, currentLightState);
        }
    }

    virtual void reportState() override {
        // Existing implementation
        if (!wifiHandler.isConnected()) return;

        Serial.println("Reporting state...");

        float temp = dhtSensor.getTemperature();
        float hum = dhtSensor.getHumidity();
        String ssid = wifiHandler.getSsid();
        IPAddress ip = wifiHandler.getLocalIp();
        int rssi = wifiHandler.getRssi();

        mqttHandler.publishState(currentFanSpeed, currentLightState, temp, hum, ssid, ip, rssi);
        mqttHandler.publishJsonReport(currentFanSpeed, currentLightState, dhtSensor, manualFanMode, millis(), ssid, ip, rssi);
    }

    virtual int getFanSpeed() const override {
        return currentFanSpeed;
    }

    virtual bool isLightOn() const override {
        return currentLightState;
    }

    void setup() {
        Serial.begin(115200);
        Serial.println("\n\n--- ESP8266 AirPump Controller ---");
        Serial.println("Version: " + String(VERSION));

        configManager.begin();
        configManager.load();

        wifiHandler.setup();
        dhtSensor.setup();
        mqttHandler.setup();

        buttonFanOff.setup();
        buttonFan1.setup();
        buttonFan2.setup();
        buttonFan3.setup();
        buttonLight.setup();
        switchFan1.setup();
        switchFan2.setup();
        switchFan3.setup();
        switchLight.setup();

        setFanSpeed(0, true);
        setLight(false);

        webServerHandler.setup();

        Serial.println("Setup complete.");
        lastReportTime = millis();
        lastSensorReadTime = millis();
    }

    void loop() {
        wifiHandler.handle();
        mqttHandler.loop();
        webServerHandler.handleClient();
        
        // Process any received MQTT messages
        processMqttMessages();
        
        handleButtons();

        if (millis() - lastSensorReadTime > 2000) {
            dhtSensor.read();
            lastSensorReadTime = millis();
        }

        if (millis() - lastReportTime > REPORT_INTERVAL_MS) {
            reportState();
            lastReportTime = millis();
        }

        delay(5);
    }

    void handleButtons() {
        // Existing implementation
        if (buttonLight.checkPressed()) {
            Serial.println("Button: Light Toggle");
            setLight(!currentLightState);
            reportState();
        }
        if (buttonFanOff.checkPressed()) {
            Serial.println("Button: Fan Off");
            setFanSpeed(0, true);
            reportState();
        }
        if (buttonFan1.checkPressed()) {
            Serial.println("Button: Fan Speed 1");
            setFanSpeed(1, true);
            reportState();
        }
        if (buttonFan2.checkPressed()) {
            Serial.println("Button: Fan Speed 2");
            setFanSpeed(2, true);
            reportState();
        }
        if (buttonFan3.checkPressed()) {
            Serial.println("Button: Fan Speed 3");
            setFanSpeed(3, true);
            reportState();
        }
    }
};

// --- Global Setup and Loop ---
AirPumpController controller;

void setup() {
    controller.setup();
}

void loop() {
    controller.loop();
}

// --- Utility Function Implementations ---
String urlDecode(String input) {
  String s = input;
  s.replace("%20", " "); s.replace("+", " "); s.replace("%21", "!");
  s.replace("%22", "\""); s.replace("%23", "#"); s.replace("%24", "$");
  s.replace("%25", "%"); s.replace("%26", "&"); s.replace("%27", "\'");
  s.replace("%28", "("); s.replace("%29", ")"); s.replace("%2A", "*"); // %30 is 0, %2A is *
  s.replace("%2B", "+"); s.replace("%2C", ","); s.replace("%2D", "-"); // %31 is 1, %2B is +
  s.replace("%2E", "."); s.replace("%2F", "/"); s.replace("%3A", ":");
  s.replace("%3B", ";"); s.replace("%3C", "<"); s.replace("%3D", "=");
  s.replace("%3E", ">"); s.replace("%3F", "?"); s.replace("%40", "@");
  s.replace("%5B", "["); s.replace("%5C", "\\"); s.replace("%5D", "]");
  s.replace("%5E", "^"); s.replace("%5F", "_"); s.replace("%60", "`");
   // Add more if needed, e.g., for other special characters
  return s;
}
