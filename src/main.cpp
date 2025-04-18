#define MQTT_MAX_PACKET_SIZE 2048
#define VERSION "0.6.0-oop" // Updated version

#include <WiFi.h>         
#include <WebServer.h>    
#include <ESPmDNS.h>      
#include <DNSServer.h>
#include <WiFiClient.h>
#include <EEPROM.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <dhtnew.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// --- Pin Definitions ---

#define SWITCH1_PIN 33  // fan speed 1 (was 1)
#define SWITCH2_PIN 25  // fan speed 2 (was 3)
#define SWITCH3_PIN 26  // fan speed 3 (was 5)
#define SWITCH4_PIN 27  // light (was 4)

#define BUTTON1_PIN 18  // D19 (was 38)
#define BUTTON2_PIN 19  // D18 (was 35)
#define BUTTON3_PIN 5   // D5 (was 34)
#define BUTTON4_PIN 17  // TX2 (was 27)
#define BUTTON5_PIN 16  // RX2 (was 25)

// #define DHT22_PIN 22    // (was 16)

// --- Constants ---
const IPAddress AP_IP(192, 168, 97, 1);
const long DEBOUNCE_DELAY = 100;
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
        WiFi.setHostname(config.hostname.c_str()); 
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
                 Serial.printf("  %d: %s (%d)%s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), (WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? " " : "*"));
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
    // bool publishState(int fanSpeed, bool lightState, float temp, float hum, const String& wifi_ssid, const IPAddress& wifi_ip, int wifi_rssi) {
        bool publishState(int fanSpeed, bool lightState, const String& wifi_ssid, const IPAddress& wifi_ip, int wifi_rssi) {
        // DynamicJsonDocument stateDoc(512); // Deprecated
        JsonDocument stateDoc; // Use JsonDocument
        // stateDoc["temperature"] = round(temp * 10) / 10.0; // One decimal place
        // stateDoc["humidity"] = round(hum * 10) / 10.0;     // One decimal place
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
    //bool publishJsonReport(int fanSpeed, bool lightState, const DHTSensor& dht, bool manualFan, unsigned long currentMillis, const String& wifi_ssid, const IPAddress& wifi_ip, int wifi_rssi) {
        bool publishJsonReport(int fanSpeed, bool lightState, bool manualFan, unsigned long currentMillis, const String& wifi_ssid, const IPAddress& wifi_ip, int wifi_rssi) {
        // DynamicJsonDocument doc(1024); // Deprecated
        JsonDocument doc; // Use JsonDocument

        doc["device"] = config.hostname;
        doc["name"]   = config.hostname;
        doc["device_ip"] = IpAddress2String(wifi_ip);
        doc["location"] = config.location;

        // doc["dht22_temp"] = round(dht.getTemperature() * 10) / 10.0;
        // doc["dht22_hum"] = round(dht.getHumidity() * 10) / 10.0;
        // doc["dht22_ok"] = dht.isReadingValid();
        // doc["dht22_error"] = dht.getErrorCount();
        // doc["dht22_timing"] = dht.getReadTimeMicros(); // Report micros now
        // doc["dht22_error_total"] = dht.getTotalErrorCount();
        // doc["dht22_status"] = dht.getStatus();

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
    WebServer server;     
    ConfigManager& config;
    WiFiHandler& wifi;
    MQTTHandler& mqtt;
    // DHTSensor& dht;
    
    // Store values directly instead of pointer to controller
    int currentFanSpeed = 0;
    bool currentLightState = false;
    ControllerInterface* controller = nullptr; // Interface pointer

    String makePageHeader(const String& title) {
        String s = "<!DOCTYPE html><html><head><meta charset=\"utf-8\">";
        s += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
        s += "<title>" + config.hostname + " - " + title + "</title>";
        // Include Bulma CSS framework from CDN
        s += "<link rel=\"stylesheet\" href=\"https://cdn.jsdelivr.net/npm/bulma@0.9.4/css/bulma.min.css\">";
        // Include Font Awesome for icons
        s += "<link rel=\"stylesheet\" href=\"https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.2.0/css/all.min.css\">";
        // Custom styles
        s += "<style>.footer-card{margin-top:1rem} .box{margin-bottom:1rem} #toast{position:fixed;bottom:20px;right:20px;z-index:9999}</style>";
        return s;
    }

    String makeFooter() {
        String s = "<footer class=\"footer p-3 is-size-7\">";
        s += "<div class=\"content has-text-centered\">";
        s += "<p><strong>" + config.hostname + "</strong> | Version: " + String(VERSION) + "</p>";
        s += "<p>Uptime: <span id=\"uptime\"> todo </span></p>";
        s += "<div class=\"columns is-mobile is-centered\">";
        s += "<div class=\"column is-narrow\"><div class=\"card footer-card\"><div class=\"card-content p-2\">";
        s += "<p class=\"has-text-centered\"><span class=\"icon\"><i class=\"fas fa-wifi\"></i></span> WiFi: <span id=\"wifi-status\">" + 
             (wifi.isConnected() ? wifi.getSsid() + " (" + String(wifi.getRssi()) + " dBm)" : (wifi.isSettingMode() ? "Setup Mode" : "Disconnected")) + "</span></p>";
        s += "</div></div></div>";
        s += "<div class=\"column is-narrow\"><div class=\"card footer-card\"><div class=\"card-content p-2\">";
        s += "<p class=\"has-text-centered\"><span class=\"icon\"><i class=\"fas fa-server\"></i></span> MQTT: <span id=\"mqtt-status\">" + 
             String(mqtt.isConnected() ? "Connected" : "Disconnected") + "</span></p>";
        s += "</div></div></div>";
        s += "</div></div></footer>";
        return s;
    }

    String makeSPA() {
        String s = makePageHeader("Control Panel");
        
        // Add jQuery and custom JS script
        s += "<script src=\"https://code.jquery.com/jquery-3.6.0.min.js\"></script>";
        
        // Start body
        s += "</head><body>";
        s += "<div id=\"toast\" class=\"notification is-success is-hidden\">Action completed</div>";
        
        // Top Navigation
        s += "<nav class=\"navbar is-primary\" role=\"navigation\" aria-label=\"main navigation\">";
        s += "<div class=\"navbar-brand\">";
        s += "<a class=\"navbar-item\" href=\"#\">";
        s += "<strong>" + config.hostname + "</strong>";
        s += "</a>";
        s += "<a role=\"button\" class=\"navbar-burger\" aria-label=\"menu\" aria-expanded=\"false\" data-target=\"navMenu\">";
        s += "<span aria-hidden=\"true\"></span>";
        s += "<span aria-hidden=\"true\"></span>";
        s += "<span aria-hidden=\"true\"></span>";
        s += "</a>";
        s += "</div>";
        s += "<div id=\"navMenu\" class=\"navbar-menu\">";
        s += "<div class=\"navbar-start\">";
        s += "<a class=\"navbar-item tab-link\" data-tab=\"main\"><span class=\"icon\"><i class=\"fas fa-home\"></i></span><span>Main</span></a>";
        
        if (!wifi.isSettingMode()) {
            s += "<a class=\"navbar-item tab-link\" data-tab=\"control\"><span class=\"icon\"><i class=\"fas fa-sliders-h\"></i></span><span>Control</span></a>";
            s += "<a class=\"navbar-item tab-link\" data-tab=\"debug\"><span class=\"icon\"><i class=\"fas fa-bug\"></i></span><span>Debug</span></a>";
        }
        
        s += "<a class=\"navbar-item tab-link\" data-tab=\"settings\"><span class=\"icon\"><i class=\"fas fa-cog\"></i></span><span>Settings</span></a>";
        s += "</div>";
        s += "<div class=\"navbar-end\">";
        s += "<div class=\"navbar-item\">";
        s += "<div class=\"buttons\">";
        s += "<a class=\"button is-danger is-light\" onclick=\"reboot()\"><span class=\"icon\"><i class=\"fas fa-power-off\"></i></span><span>Reboot</span></a>";
        s += "<a class=\"button is-warning is-light\" onclick=\"resetConfig()\"><span class=\"icon\"><i class=\"fas fa-trash\"></i></span><span>" + 
             String(wifi.isSettingMode() ? "Reset WiFi" : "Reset All") + "</span></a>";
        s += "</div></div></div></div>";
        s += "</nav>";

        // Main container
        s += "<div class=\"container p-3\">";
        
        // Main tab content
        s += "<div id=\"main-tab\" class=\"content-tab is-active\">";
        s += "<div class=\"columns\">";
        // Status column
        s += "<div class=\"column is-half\">";
        s += "<div class=\"box\">";
        s += "<h3 class=\"title is-4\"><span class=\"icon\"><i class=\"fas fa-info-circle\"></i></span> Device Status</h3>";
        s += "<table class=\"table is-fullwidth\">";
        s += "<tr><td>Fan Speed</td><td id=\"status-fan\">" + String(currentFanSpeed) + "</td></tr>";
        s += "<tr><td>Light</td><td id=\"status-light\">" + String(currentLightState ? "ON" : "OFF") + "</td></tr>";
        s += "<tr><td>Temperature</td><td id=\"status-temp\">todo °C</td></tr>";
        s += "<tr><td>Humidity</td><td id=\"status-humidity\">todo %</td></tr>";
        s += "<tr><td>IP Address</td><td id=\"status-ip\">" + IpAddress2String(wifi.getLocalIp()) + "</td></tr>";
        s += "</table>";
        s += "</div>";
        s += "</div>";
        
        // Quick control column
        s += "<div class=\"column is-half\">";
        s += "<div class=\"box\">";
        s += "<h3 class=\"title is-4\"><span class=\"icon\"><i class=\"fas fa-bolt\"></i></span> Quick Controls</h3>";
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Light</label>";
        s += "<div class=\"control\">";
        s += "<div class=\"buttons has-addons\">";
        s += "<button id=\"light-off-btn\" class=\"button " + String(!currentLightState ? "is-info is-selected" : "") + "\" onclick=\"setLight(false)\">OFF</button>";
        s += "<button id=\"light-on-btn\" class=\"button " + String(currentLightState ? "is-info is-selected" : "") + "\" onclick=\"setLight(true)\">ON</button>";
        s += "</div>";
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Fan Speed</label>";
        s += "<div class=\"control\">";
        s += "<div class=\"buttons has-addons\">";
        s += "<button id=\"fan-0-btn\" class=\"button " + String(currentFanSpeed == 0 ? "is-info is-selected" : "") + "\" onclick=\"setFan(0)\">OFF</button>";
        s += "<button id=\"fan-1-btn\" class=\"button " + String(currentFanSpeed == 1 ? "is-info is-selected" : "") + "\" onclick=\"setFan(1)\">1</button>";
        s += "<button id=\"fan-2-btn\" class=\"button " + String(currentFanSpeed == 2 ? "is-info is-selected" : "") + "\" onclick=\"setFan(2)\">2</button>";
        s += "<button id=\"fan-3-btn\" class=\"button " + String(currentFanSpeed == 3 ? "is-info is-selected" : "") + "\" onclick=\"setFan(3)\">3</button>";
        s += "</div>";
        s += "</div>";
        s += "</div>";
        
        if (wifi.isSettingMode()) {
            s += "<div class=\"notification is-warning\">";
            s += "<p><strong>Device is in Setup Mode.</strong> Please configure WiFi settings.</p>";
            s += "</div>";
        }
        
        s += "</div>";
        s += "</div>";
        s += "</div>";
        s += "</div>";

        // Control tab content
        if (!wifi.isSettingMode()) {
            s += "<div id=\"control-tab\" class=\"content-tab is-hidden\">";
            s += "<div class=\"box\">";
            s += "<h3 class=\"title is-4\"><span class=\"icon\"><i class=\"fas fa-sliders-h\"></i></span> Manual Control</h3>";
            
            s += "<div class=\"field\">";
            s += "<label class=\"label\">Fan Speed</label>";
            s += "<div class=\"control\">";
            s += "<div class=\"select\">";
            s += "<select id=\"fan-speed-select\">";
            s += "<option value=\"0\"" + String(currentFanSpeed == 0 ? " selected" : "") + ">Off</option>";
            s += "<option value=\"1\"" + String(currentFanSpeed == 1 ? " selected" : "") + ">Speed 1</option>";
            s += "<option value=\"2\"" + String(currentFanSpeed == 2 ? " selected" : "") + ">Speed 2</option>";
            s += "<option value=\"3\"" + String(currentFanSpeed == 3 ? " selected" : "") + ">Speed 3</option>";
            s += "</select>";
            s += "</div>";
            s += "</div>";
            s += "</div>";
            
            s += "<div class=\"field\">";
            s += "<label class=\"label\">Light</label>";
            s += "<div class=\"control\">";
            s += "<div class=\"select\">";
            s += "<select id=\"light-state-select\">";
            s += "<option value=\"0\"" + String(!currentLightState ? " selected" : "") + ">Off</option>";
            s += "<option value=\"1\"" + String(currentLightState ? " selected" : "") + ">On</option>";
            s += "</select>";
            s += "</div>";
            s += "</div>";
            s += "</div>";
            
            s += "<div class=\"field\">";
            s += "<div class=\"control\">";
            s += "<button class=\"button is-primary\" id=\"apply-settings\">Apply Settings</button>";
            s += "</div>";
            s += "</div>";
            s += "</div>";
            s += "</div>";

            // Debug tab content
            s += "<div id=\"debug-tab\" class=\"content-tab is-hidden\">";
            s += "<div class=\"box\">";
            s += "<h3 class=\"title is-4\"><span class=\"icon\"><i class=\"fas fa-bug\"></i></span> Debug Information</h3>";
            s += "<h4 class=\"title is-5\">MQTT</h4>";
            s += "<div class=\"table-container\">";
            s += "<table class=\"table is-fullwidth is-striped\">";
            s += "<tr><td>Status</td><td id=\"mqtt-connection-status\">" + String(mqtt.isConnected() ? "Connected" : "Disconnected") + "</td></tr>";
            s += "<tr><td>Availability Topic</td><td>" + mqtt.getAvailabilityTopic() + "</td></tr>";
            s += "<tr><td>State Topic</td><td>" + mqtt.getStateTopic() + "</td></tr>";
            s += "<tr><td>Command Topic</td><td>" + mqtt.getCommandTopic() + "</td></tr>";
            s += "<tr><td>JSON Report Topic</td><td>" + mqtt.getJsonReportTopic() + "</td></tr>";
            s += "</table>";
            s += "</div>";
            
            s += "<h4 class=\"title is-5\">MQTT Log <button class=\"button is-small is-light\" id=\"clear-mqtt-log\">Clear</button></h4>";
            s += "<div class=\"notification\">";
            s += "<pre id=\"mqtt-log\" style=\"max-height:200px;overflow:auto\">" + mqtt.getLog() + "</pre>";
            s += "</div>";
            
            s += "<h4 class=\"title is-5\">DHT Sensor</h4>";
            s += "<div class=\"table-container\">";
            s += "<table class=\"table is-fullwidth is-striped\">";
            s += "<tr><td>Status</td><td id=\"dht-status\">todo </td></tr>";
            s += "<tr><td>Temperature</td><td id=\"dht-temp\">todo °C</td></tr>";
            s += "<tr><td>Humidity</td><td id=\"dht-hum\">todo %</td></tr>";
            s += "<tr><td>Last Read Time</td><td id=\"dht-read-time\">todo μs</td></tr>";
            s += "<tr><td>Consecutive Errors</td><td id=\"dht-err\">todo </td></tr>";
            s += "<tr><td>Total Errors</td><td id=\"dht-total-err\">todo </td></tr>";
            s += "</table>";
            s += "</div>";
            
            s += "<h4 class=\"title is-5\">System</h4>";
            s += "<div class=\"table-container\">";
            s += "<table class=\"table is-fullwidth is-striped\">";
            s += "<tr><td>Free Heap</td><td id=\"free-heap\">" + String(ESP.getFreeHeap()) + " bytes</td></tr>";
            s += "<tr><td>Chip ID</td><td>todo</td></tr>";
            s += "<tr><td>SDK Version</td><td>" + String(ESP.getSdkVersion()) + "</td></tr>";
            s += "</table>";
            s += "</div>";
            s += "</div>";
            s += "</div>";
        }

        // Settings tab content
        s += "<div id=\"settings-tab\" class=\"content-tab is-hidden\">";
        s += "<div class=\"box\">";
        s += "<h3 class=\"title is-4\"><span class=\"icon\"><i class=\"fas fa-cog\"></i></span> Configuration Settings</h3>";
        
        // WiFi Settings
        s += "<h4 class=\"title is-5\">WiFi</h4>";
        s += "<div class=\"field\">";
        s += "<label class=\"label\">SSID</label>";
        s += "<div class=\"control\">";
        
        if (wifi.isSettingMode()) {
            s += "<div class=\"select is-fullwidth\">";
            s += "<select id=\"ssid\">" + wifi.getSsidListHtml() + "</select>";
            s += "</div>";
        } else {
            s += "<input class=\"input\" type=\"text\" id=\"ssid\" value=\"" + config.ssid + "\">";
        }
        
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Password</label>";
        s += "<div class=\"control\">";
        s += "<input class=\"input\" type=\"password\" id=\"pass\" value=\"" + config.pass + "\">";
        s += "</div>";
        s += "</div>";
        
        // Device Settings
        s += "<h4 class=\"title is-5\">Device</h4>";
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Hostname</label>";
        s += "<div class=\"control\">";
        s += "<input class=\"input\" type=\"text\" id=\"hostname\" value=\"" + config.hostname + "\">";
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Location (for MQTT topic)</label>";
        s += "<div class=\"control\">";
        s += "<input class=\"input\" type=\"text\" id=\"location\" value=\"" + config.location + "\">";
        s += "</div>";
        s += "</div>";
        
        // MQTT Settings
        s += "<h4 class=\"title is-5\">MQTT</h4>";
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Server</label>";
        s += "<div class=\"control\">";
        s += "<input class=\"input\" type=\"text\" id=\"mqtt_server\" value=\"" + config.mqtt_server + "\">";
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Port</label>";
        s += "<div class=\"control\">";
        s += "<input class=\"input\" type=\"text\" id=\"mqtt_port\" value=\"" + config.mqtt_port + "\">";
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Use Authentication</label>";
        s += "<div class=\"control\">";
        s += "<div class=\"select\">";
        s += "<select id=\"mqtt_auth\">";
        s += "<option value=\"false\"" + String(!config.mqtt_auth ? " selected" : "") + ">No</option>";
        s += "<option value=\"true\"" + String(config.mqtt_auth ? " selected" : "") + ">Yes</option>";
        s += "</select>";
        s += "</div>";
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Username</label>";
        s += "<div class=\"control\">";
        s += "<input class=\"input\" type=\"text\" id=\"mqtt_user\" value=\"" + config.mqtt_user + "\">";
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<label class=\"label\">Password</label>";
        s += "<div class=\"control\">";
        s += "<input class=\"input\" type=\"password\" id=\"mqtt_password\" value=\"" + config.mqtt_password + "\">";
        s += "</div>";
        s += "</div>";
        
        s += "<div class=\"field\">";
        s += "<div class=\"control\">";
        s += "<button class=\"button is-primary\" id=\"save-settings\">Save Settings & Reboot</button>";
        s += "</div>";
        s += "</div>";
        
        s += "</div>";
        s += "</div>";
        
        // End container
        s += "</div>";
        
        // Add footer
        s += makeFooter();
        
        // Add JavaScript for SPA functionality
        s += "<script>";
        
        // Tab handling
        s += "document.addEventListener('DOMContentLoaded', function() {";
        s += "  // Tab handling";
        s += "  const tabLinks = document.querySelectorAll('.tab-link');";
        s += "  tabLinks.forEach(link => {";
        s += "    link.addEventListener('click', () => {";
        s += "      const tabId = link.getAttribute('data-tab');";
        s += "      document.querySelectorAll('.content-tab').forEach(tab => {";
        s += "        tab.classList.add('is-hidden');";
        s += "      });";
        s += "      document.querySelectorAll('.tab-link').forEach(tab => {";
        s += "        tab.classList.remove('is-active');";
        s += "      });";
        s += "      document.getElementById(tabId + '-tab').classList.remove('is-hidden');";
        s += "      link.classList.add('is-active');";
        s += "    });";
        s += "  });";
        
        // Mobile menu toggle
        s += "  // Mobile menu";
        s += "  const burger = document.querySelector('.navbar-burger');";
        s += "  burger.addEventListener('click', () => {";
        s += "    const target = document.getElementById(burger.dataset.target);";
        s += "    burger.classList.toggle('is-active');";
        s += "    target.classList.toggle('is-active');";
        s += "  });";
        
        // Button handlers
        s += "  // Event handlers";
        s += "  const saveSettingsBtn = document.getElementById('save-settings');";
        s += "  if (saveSettingsBtn) {";
        s += "    saveSettingsBtn.addEventListener('click', saveSettings);";
        s += "  }";
        s += "  const applyBtn = document.getElementById('apply-settings');";
        s += "  if (applyBtn) {";
        s += "    applyBtn.addEventListener('click', applySettings);";
        s += "  }";
        s += "  const clearLogBtn = document.getElementById('clear-mqtt-log');";
        s += "  if (clearLogBtn) {";
        s += "    clearLogBtn.addEventListener('click', clearMqttLog);";
        s += "  }";
        
        // Set up periodic data updates
        s += "  // Periodic updates";
        s += "  setInterval(updateData, 2000);";
        s += "});";
        
        // AJAX Functions
        s += "function updateData() {";
        s += "  fetch('/api/status')";
        s += "    .then(response => response.json())";
        s += "    .then(data => {";
        s += "      document.getElementById('status-fan').textContent = data.fan_speed;";
        s += "      document.getElementById('status-light').textContent = data.light ? 'ON' : 'OFF';";
        s += "      document.getElementById('status-temp').textContent = data.temperature.toFixed(1) + ' °C';";
        s += "      document.getElementById('status-humidity').textContent = data.humidity.toFixed(1) + ' %';";
        s += "      document.getElementById('status-ip').textContent = data.wifi.ip;";
        s += "      document.getElementById('wifi-status').textContent = data.wifi.ssid + ' (' + data.wifi.rssi + ' dBm)';";
        s += "      document.getElementById('mqtt-status').textContent = data.mqtt_connected ? 'Connected' : 'Disconnected';";
        s += "      document.getElementById('uptime').textContent = data.uptime;";
        
        // Update debug info if available
        s += "      if (document.getElementById('dht-status')) {";
        s += "        document.getElementById('dht-status').textContent = data.dht.status;";
        s += "        document.getElementById('dht-temp').textContent = data.temperature.toFixed(2) + ' °C';";
        s += "        document.getElementById('dht-hum').textContent = data.humidity.toFixed(2) + ' %';";
        s += "        document.getElementById('dht-read-time').textContent = data.dht.read_time + ' μs';";
        s += "        document.getElementById('dht-err').textContent = data.dht.error_count;";
        s += "        document.getElementById('dht-total-err').textContent = data.dht.total_error_count;";
        s += "        document.getElementById('free-heap').textContent = data.free_heap + ' bytes';";
        s += "        document.getElementById('mqtt-connection-status').textContent = data.mqtt_connected ? 'Connected' : 'Disconnected';";
        s += "        document.getElementById('mqtt-log').textContent = data.mqtt_log;";
        s += "      }";
        
        // Update button states
        s += "      updateButtonStates(data.fan_speed, data.light);";
        s += "    })";
        s += "    .catch(error => console.error('Error updating data:', error));";
        s += "}";
        
        s += "function updateButtonStates(fanSpeed, lightOn) {";
        s += "  // Update fan buttons";
        s += "  for (let i = 0; i <= 3; i++) {";
        s += "    const btn = document.getElementById('fan-' + i + '-btn');";
        s += "    if (btn) {";
        s += "      if (i === fanSpeed) {";
        s += "        btn.classList.add('is-info', 'is-selected');";
        s += "      } else {";
        s += "        btn.classList.remove('is-info', 'is-selected');";
        s += "      }";
        s += "    }";
        s += "  }";
        
        s += "  // Update light buttons";
        s += "  const lightOffBtn = document.getElementById('light-off-btn');";
        s += "  const lightOnBtn = document.getElementById('light-on-btn');";
        s += "  if (lightOffBtn && lightOnBtn) {";
        s += "    if (lightOn) {";
        s += "      lightOnBtn.classList.add('is-info', 'is-selected');";
        s += "      lightOffBtn.classList.remove('is-info', 'is-selected');";
        s += "    } else {";
        s += "      lightOffBtn.classList.add('is-info', 'is-selected');";
        s += "      lightOnBtn.classList.remove('is-info', 'is-selected');";
        s += "    }";
        s += "  }";
        
        s += "  // Update select controls too";
        s += "  const fanSelect = document.getElementById('fan-speed-select');";
        s += "  const lightSelect = document.getElementById('light-state-select');";
        s += "  if (fanSelect) fanSelect.value = fanSpeed;";
        s += "  if (lightSelect) lightSelect.value = lightOn ? '1' : '0';";
        s += "}";

        // Control functions
        s += "function setFan(speed) {";
        s += "  fetch('/api/control?fan=' + speed)";
        s += "    .then(response => response.json())";
        s += "    .then(data => {";
        s += "      showToast('Fan speed set to ' + speed);";
        s += "      updateButtonStates(data.fan_speed, data.light);";
        s += "    });";
        s += "}";

        s += "function setLight(state) {";
        s += "  fetch('/api/control?light=' + (state ? '1' : '0'))";
        s += "    .then(response => response.json())";
        s += "    .then(data => {";
        s += "      showToast('Light turned ' + (state ? 'ON' : 'OFF'));";
        s += "      updateButtonStates(data.fan_speed, data.light);";
        s += "    });";
        s += "}";

        s += "function applySettings() {";
        s += "  const fanSpeed = document.getElementById('fan-speed-select').value;";
        s += "  const lightState = document.getElementById('light-state-select').value === '1';";
        s += "  fetch(`/api/control?fan=${fanSpeed}&light=${lightState ? '1' : '0'}`)";
        s += "    .then(response => response.json())";
        s += "    .then(data => {";
        s += "      showToast('Settings applied');";
        s += "      updateButtonStates(data.fan_speed, data.light);";
        s += "    });";
        s += "}";

        s += "function saveSettings() {";
        s += "  const data = {";
        s += "    ssid: document.getElementById('ssid').value,";
        s += "    pass: document.getElementById('pass').value,";
        s += "    hostname: document.getElementById('hostname').value,";
        s += "    location: document.getElementById('location').value,";
        s += "    mqtt_server: document.getElementById('mqtt_server').value,";
        s += "    mqtt_port: document.getElementById('mqtt_port').value,";
        s += "    mqtt_auth: document.getElementById('mqtt_auth').value,";
        s += "    mqtt_user: document.getElementById('mqtt_user').value,";
        s += "    mqtt_password: document.getElementById('mqtt_password').value";
        s += "  };";
        s += "  fetch('/api/settings', {";
        s += "    method: 'POST',";
        s += "    headers: {'Content-Type': 'application/json'},";
        s += "    body: JSON.stringify(data)";
        s += "  })";
        s += "  .then(response => response.json())";
        s += "  .then(data => {";
        s += "    if (data.success) {";
        s += "      showToast('Settings saved. Device will reboot.');";
        s += "      setTimeout(() => { window.location.reload(); }, 5000);";
        s += "    } else {";
        s += "      showToast('Error saving settings', 'is-danger');";
        s += "    }";
        s += "  });";
        s += "}";

        s += "function clearMqttLog() {";
        s += "  fetch('/api/clear_log')";
        s += "    .then(response => response.json())";
        s += "    .then(data => {";
        s += "      document.getElementById('mqtt-log').textContent = '';";
        s += "      showToast('Log cleared');";
        s += "    });";
        s += "}";

        s += "function reboot() {";
        s += "  if (confirm('Are you sure you want to reboot the device?')) {";
        s += "    fetch('/api/reboot')";
        s += "      .then(() => {";
        s += "        showToast('Rebooting...');";
        s += "        setTimeout(() => { window.location.reload(); }, 10000);";
        s += "      });";
        s += "  }";
        s += "}";

        s += "function resetConfig() {";
        s += "  if (confirm('Are you sure you want to reset all settings?')) {";
        s += "    fetch('/api/reset')";
        s += "      .then(() => {";
        s += "        showToast('Resetting configuration...');";
        s += "        setTimeout(() => { window.location.reload(); }, 10000);";
        s += "      });";
        s += "  }";
        s += "}";

        s += "function showToast(message, type = 'is-success') {";
        s += "  const toast = document.getElementById('toast');";
        s += "  toast.className = 'notification ' + type;";
        s += "  toast.textContent = message;";
        s += "  toast.classList.remove('is-hidden');";
        s += "  setTimeout(() => { toast.classList.add('is-hidden'); }, 3000);";
        s += "}";

        s += "</script>";
        
        s += "</body></html>";
        return s;
    }

    // --- API Route Handlers ---
    void handleApiStatus() {
        // DynamicJsonDocument doc(1024);
        JsonDocument doc;

        // doc["temperature"] = dht.getTemperature();
        // doc["humidity"] = dht.getHumidity();
        doc["fan_speed"] = currentFanSpeed;
        doc["light"] = currentLightState;
        doc["mqtt_connected"] = mqtt.isConnected();
        doc["uptime"] = "need to fix";
        doc["free_heap"] = ESP.getFreeHeap();

        
        // WiFi info
        JsonObject wifiObj = doc["wifi"].to<JsonObject>();
        wifiObj["ssid"] = wifi.getSsid();
        wifiObj["ip"] = IpAddress2String(wifi.getLocalIp());
        wifiObj["rssi"] = wifi.getRssi();
        
        // DHT info
        // JsonObject dhtObj = doc["dht"].to<JsonObject>();
        // dhtObj["status"] = dht.getStatus();
        // dhtObj["read_time"] = dht.getReadTimeMicros();
        // dhtObj["error_count"] = dht.getErrorCount();
        // dhtObj["total_error_count"] = dht.getTotalErrorCount();
        
        // MQTT log in debug mode
        doc["mqtt_log"] = mqtt.getLog();
        
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json", jsonResponse);
    }

    void handleApiControl() {
        bool changed = false;
        
        if (server.hasArg("fan")) {
            int fanSpeed = server.arg("fan").toInt();
            if (fanSpeed >= 0 && fanSpeed <= 3 && controller) {
                controller->setFanSpeed(fanSpeed, true);
                currentFanSpeed = fanSpeed;
                changed = true;
            }
        }
        
        if (server.hasArg("light")) {
            bool lightState = server.arg("light").toInt() == 1;
            if (controller) {
                controller->setLight(lightState);
                currentLightState = lightState;
                changed = true;
            }
        }
        
        if (changed && controller) {
            controller->reportState();
        }
        
        // DynamicJsonDocument doc(256);
        JsonDocument doc;
        doc["success"] = true;
        doc["fan_speed"] = currentFanSpeed;
        doc["light"] = currentLightState;
        
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json", jsonResponse);
    }

    void handleApiSettings() {
        if (server.hasArg("plain")) { // Content sent as application/json
            String json = server.arg("plain");
            // DynamicJsonDocument doc(1024);
            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, json);
            
            if (!error) {
                String ssid_val = doc["ssid"].as<String>();
                String pass_val = doc["pass"].as<String>();
                String host_val = doc["hostname"].as<String>();
                String loc_val = doc["location"].as<String>();
                String mqtt_srv_val = doc["mqtt_server"].as<String>();
                String mqtt_port_val = doc["mqtt_port"].as<String>();
                String mqtt_auth_val = doc["mqtt_auth"].as<String>();
                String mqtt_user_val = doc["mqtt_user"].as<String>();
                String mqtt_pass_val = doc["mqtt_password"].as<String>();
                
                config.save(ssid_val, pass_val, host_val, loc_val, mqtt_srv_val, mqtt_port_val, 
                           mqtt_auth_val, mqtt_user_val, mqtt_pass_val);
                
                // DynamicJsonDocument response(128);
                JsonDocument response;
                response["success"] = true;
                response["message"] = "Settings saved. Device will reboot.";
                String jsonResponse;
                serializeJson(response, jsonResponse);
                
                server.sendHeader("Access-Control-Allow-Origin", "*");
                server.send(200, "application/json", jsonResponse);
                
                delay(1000);
                ESP.restart();
            } else {
                // DynamicJsonDocument response(128);
                JsonDocument response;
                response["success"] = false;
                response["message"] = "JSON parsing error";
                String jsonResponse;
                serializeJson(response, jsonResponse);
                
                server.sendHeader("Access-Control-Allow-Origin", "*");
                server.send(400, "application/json", jsonResponse);
            }
        } else {
            // DynamicJsonDocument response(128);
            JsonDocument response;
            response["success"] = false;
            response["message"] = "No JSON data received";
            String jsonResponse;
            serializeJson(response, jsonResponse);
            
            server.sendHeader("Access-Control-Allow-Origin", "*");
            server.send(400, "application/json", jsonResponse);
        }
    }

    void handleApiReboot() {
        // DynamicJsonDocument doc(128);
        JsonDocument doc;
        doc["success"] = true;
        doc["message"] = "Rebooting...";
        
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json", jsonResponse);
        
        delay(1000);
        ESP.restart();
    }

    void handleApiReset() {
        // DynamicJsonDocument doc(128);
        JsonDocument doc;
        doc["success"] = true;
        doc["message"] = "Configuration reset. Rebooting...";
        
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json", jsonResponse);
        
        config.clear();
        delay(1000);
        ESP.restart();
    }

    void handleApiClearLog() {
        mqtt.clearLog();
        // DynamicJsonDocument doc(128);
        JsonDocument doc;
        doc["success"] = true;
        doc["message"] = "Log cleared";
        
        String jsonResponse;
        serializeJson(doc, jsonResponse);
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.send(200, "application/json", jsonResponse);
    }

    // CORS preflight handler
    void handleCORS() {
        server.sendHeader("Access-Control-Allow-Origin", "*");
        server.sendHeader("Access-Control-Allow-Methods", "GET, POST, PUT, DELETE, OPTIONS");
        server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
        server.send(204);
    }

    void handleLegacyRoot() {
        server.send(200, "text/html", makeSPA());
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
    // WebServerHandler(ConfigManager& cfg, WiFiHandler& wf, MQTTHandler& mq, DHTSensor& dh) :
    WebServerHandler(ConfigManager& cfg, WiFiHandler& wf, MQTTHandler& mq) :
        server(80), config(cfg), wifi(wf), mqtt(mq) {}

    void setup() {
        // API endpoints
        server.on("/api/status", HTTP_GET, [this]() { this->handleApiStatus(); });
        server.on("/api/control", HTTP_GET, [this]() { this->handleApiControl(); });
        server.on("/api/settings", HTTP_POST, [this]() { this->handleApiSettings(); });
        server.on("/api/reboot", HTTP_GET, [this]() { this->handleApiReboot(); });
        server.on("/api/reset", HTTP_GET, [this]() { this->handleApiReset(); });
        server.on("/api/clear_log", HTTP_GET, [this]() { this->handleApiClearLog(); });
        
        // CORS support
        server.on("/api/status", HTTP_OPTIONS, [this]() { this->handleCORS(); });
        server.on("/api/control", HTTP_OPTIONS, [this]() { this->handleCORS(); });
        server.on("/api/settings", HTTP_OPTIONS, [this]() { this->handleCORS(); });
        server.on("/api/reboot", HTTP_OPTIONS, [this]() { this->handleCORS(); });
        server.on("/api/reset", HTTP_OPTIONS, [this]() { this->handleCORS(); });
        server.on("/api/clear_log", HTTP_OPTIONS, [this]() { this->handleCORS(); });
        
        // Main SPA
        server.on("/", HTTP_GET, [this]() { this->handleLegacyRoot(); });
        
        // Legacy redirects (to maintain compatibility with bookmarks)
        server.on("/control", HTTP_GET, [this]() { this->handleLegacyRoot(); });
        server.on("/debug", HTTP_GET, [this]() { this->handleLegacyRoot(); });
        server.on("/settings", HTTP_GET, [this]() { this->handleLegacyRoot(); });
        server.on("/reboot", HTTP_GET, [this]() { this->handleLegacyRoot(); });
        server.on("/reset", HTTP_GET, [this]() { this->handleLegacyRoot(); });
        
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
    // DHTSensor dhtSensor;
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
        // dhtSensor(DHT22_PIN),
        mqttHandler(wifiClient, configManager),
        // webServerHandler(configManager, wifiHandler, mqttHandler, dhtSensor),
        webServerHandler(configManager, wifiHandler, mqttHandler),
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

        // float temp = dhtSensor.getTemperature();
        // float hum = dhtSensor.getHumidity();
        String ssid = wifiHandler.getSsid();
        IPAddress ip = wifiHandler.getLocalIp();
        int rssi = wifiHandler.getRssi();

        // mqttHandler.publishState(currentFanSpeed, currentLightState, temp, hum, ssid, ip, rssi);
        mqttHandler.publishState(currentFanSpeed, currentLightState, ssid, ip, rssi);
        // mqttHandler.publishJsonReport(currentFanSpeed, currentLightState, dhtSensor, manualFanMode, millis(), ssid, ip, rssi);
        mqttHandler.publishJsonReport(currentFanSpeed, currentLightState, manualFanMode, millis(), ssid, ip, rssi);
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
        // dhtSensor.setup();
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
            // dhtSensor.read();
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
