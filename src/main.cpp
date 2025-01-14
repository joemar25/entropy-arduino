#include <PMS.h>
#include "DFRobot_MultiGasSensor.h"
#include "DHT.h"
#include <SoftwareSerial.h>
#include <MQ131.h>
#include <WiFiEsp.h>  // Use WiFiEsp for Arduino Mega and ESP8266
#include <EEPROM.h>   // Include EEPROM library to store configuration

// DIP Switch Configuration
#define DIP_SWITCH_3_PIN 3
#define DIP_SWITCH_4_PIN 4
#define DIP_SWITCH_5_PIN 5
#define DIP_SWITCH_6_PIN 6
#define DIP_SWITCH_7_PIN 7

// DHT22 configuration
#define DHTPIN 7            // DHT22 data pin connected to pin 7
#define DHTTYPE DHT22       // DHT22 type
DHT dht(DHTPIN, DHTTYPE);

// I2C addresses for the gas sensors
#define SENSOR_1_ADDR 0x74
#define SENSOR_2_ADDR 0x75
#define SENSOR_3_ADDR 0x76

// Gas sensor objects
DFRobot_GAS_I2C sensor1(&Wire, SENSOR_1_ADDR);
DFRobot_GAS_I2C sensor2(&Wire, SENSOR_2_ADDR);
DFRobot_GAS_I2C sensor3(&Wire, SENSOR_3_ADDR);

// MQ-131 Ozone Sensor configuration
#define MQ131_PIN A0

// ZP07-MP503 VOC Sensor configuration
#define MP503_PIN A1
#define VCC 5.0  // Supply voltage
#define RL 20.0  // Load resistance in kΩ
#define R0 1.0   // Sensor resistance in clean air (needs calibration)

// PMS5003 Particulate Matter Sensor configuration
#define PMS_RX_PIN 8
#define PMS_TX_PIN 9
SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN);
PMS pms(pmsSerial);
PMS::DATA pmsData;

// Gravity CO2 Sensor configuration
#define CO2_SENSOR_PIN A2
#define DC_GAIN 8.5                 // DC gain of amplifier
#define READ_SAMPLE_INTERVAL 50     // Time interval between samples in milliseconds
#define READ_SAMPLE_TIMES 5         // Number of samples to take
#define ZERO_POINT_VOLTAGE 0.220    // Output voltage at 400PPM CO2
#define REACTION_VOLTAGE 0.030      // Voltage drop in 1000ppm CO2
float CO2Curve[3] = {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTAGE / (2.602 - 3))};

// WiFi configuration
#define WIFI_RX 10  // RX for SoftwareSerial to ESP8266
#define WIFI_TX 11  // TX for SoftwareSerial to ESP8266
SoftwareSerial wifiSerial(WIFI_RX, WIFI_TX);
WiFiEspClient client;

// EEPROM addresses
#define DEVICE_ID_ADDR 0
#define SSID_ADDR 50
#define PASSWORD_ADDR 100

// Global variables
String deviceID;
String ssid;
String password;

// Sensor read delay (in milliseconds)
#define SENSOR_READ_DELAY 2000

// Function prototypes
void initializeSensor(DFRobot_GAS_I2C &sensor, const char *name);
void connectToWiFi();
void configureDevice();
void loadConfiguration();
void saveConfiguration();
void displayDHT22Data();
void displayMQ131Data();
void displayMP503Data();
void displayPMS5003Data();
void displayCO2Data();
void displayGasSensorData(const char *sensorName, DFRobot_GAS_I2C &sensor);
bool areSensorsEnabled();
bool isWiFiEnabled();
String readStringFromEEPROM(int addr);
void writeStringToEEPROM(int addr, const String &data);

void setup() {
  Serial.begin(115200);          // Initialize debug Serial
  wifiSerial.begin(9600);        // Initialize WiFi ESP8266 Serial
  WiFi.init(&wifiSerial);        // Initialize WiFiEsp library
  dht.begin();                   // Initialize DHT22 sensor
  pmsSerial.begin(115200);       // Initialize PMS5003
  Wire.begin();                  // Initialize I2C bus

  // Initialize gas sensors
  initializeSensor(sensor1, "Sensor 1");
  initializeSensor(sensor2, "Sensor 2");
  initializeSensor(sensor3, "Sensor 3");

  // Initialize MQ131 Ozone Sensor
  MQ131.begin(6, MQ131_PIN, LOW_CONCENTRATION, 1000000);
  MQ131.setTimeToRead(20);  // Set how many seconds we will read from the Ozone sensor
  MQ131.setR0(9000);        // Calibrated R0 value

  // Initialize DIP switch pins as inputs
  pinMode(DIP_SWITCH_3_PIN, INPUT);
  pinMode(DIP_SWITCH_4_PIN, INPUT);
  pinMode(DIP_SWITCH_5_PIN, INPUT);
  pinMode(DIP_SWITCH_6_PIN, INPUT);
  pinMode(DIP_SWITCH_7_PIN, INPUT);

  // Load configuration
  loadConfiguration();

  // Configure device if required
  if (deviceID.length() == 0 || ssid.length() == 0 || password.length() == 0) {
    configureDevice();
  }

  // Connect to WiFi if enabled
  if (isWiFiEnabled()) {
    connectToWiFi();
  }
}

void loop() {
  if (areSensorsEnabled()) {
    Serial.println("========== All Sensor Data ==========");

    // Read and display sensor data
    displayDHT22Data();
    displayMQ131Data();
    displayMP503Data();
    displayPMS5003Data();
    displayCO2Data();
    displayGasSensorData("Sensor 1", sensor1);
    displayGasSensorData("Sensor 2", sensor2);
    displayGasSensorData("Sensor 3", sensor3);

    Serial.println("=====================================");
  } else {
    Serial.println("Sensors are not enabled. No data will be printed.");
  }

  delay(SENSOR_READ_DELAY);
}

void displayDHT22Data() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();

  if (isnan(temp) || isnan(humidity)) {
    Serial.println("DHT22: Failed to read data!");
  } else {
    Serial.print("DHT22 - Temperature: ");
    Serial.print(temp);
    Serial.println(" °C");
    Serial.print("DHT22 - Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  }
}

void displayMQ131Data() {
  int ozonePpm = MQ131.getO3(PPB);
  Serial.println("========================");
  Serial.println("MQ-131 Ozone Sensor:");
  Serial.print("Ozone Concentration: ");
  Serial.print(ozonePpm, 2);
  Serial.println(" PPM");
  Serial.println("========================");
}

void displayMP503Data() {
  int sensorValue = analogRead(MP503_PIN);
  float voltage = sensorValue * (VCC / 1023.0);
  float rs = ((VCC * RL) / voltage) - RL;
  float ratio = rs / R0;
  float ppm = 300.0 * pow(ratio, -1.7); // 300.0 and -1.7 are calibration constants
  ppm = constrain(ppm, 0, 1000);

  Serial.print("MP503 VOC Sensor - Raw: ");
  Serial.print(sensorValue);
  Serial.print(", Voltage: ");
  Serial.print(voltage, 3);
  Serial.print("V, Rs: ");
  Serial.print(rs, 1);
  Serial.print("kΩ, Rs/R0: ");
  Serial.print(ratio, 3);
  Serial.print(", VOC: ");
  Serial.print(ppm, 1);
  Serial.println(" PPM");
}

void displayPMS5003Data() {
  if (pms.read(pmsData)) {
    Serial.println("PMS5003 Particulate Matter Data:");
    Serial.print("PM2.5: ");
    Serial.print(pmsData.PM_AE_UG_2_5);
    Serial.println(" µg/m³");
  } else {
    Serial.println("PMS5003: No data available");
  }
}

void displayCO2Data() {
  float volts = MGRead(CO2_SENSOR_PIN);
  int percentage = MGGetPercentage(volts, CO2Curve);
  Serial.print("Gravity CO2 Sensor - Concentration:");
  if (percentage == -1) {
    Serial.print("<400");
  } else {
    Serial.print(percentage);
  }
  Serial.println("ppm");
}

float MGRead(int mg_pin) {
  float v = 0;
  for (int i = 0; i < READ_SAMPLE_TIMES; i++) {
    v += analogRead(mg_pin);
    delay(READ_SAMPLE_INTERVAL);
  }
  v = (v / READ_SAMPLE_TIMES) * 5 / 1024;
  return v;
}

int MGGetPercentage(float volts, float *pcurve) {
  if ((volts / DC_GAIN) >= ZERO_POINT_VOLTAGE) {
    return -1;
  } else {
    return pow(10, ((volts / DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

void initializeSensor(DFRobot_GAS_I2C &sensor, const char *name) {
  if (!sensor.begin()) {
    Serial.print(name);
    Serial.println(" not found!");
  } else {
    Serial.print(name);
    Serial.println(" initialized!");
  }
}

void displayGasSensorData(const char *sensorName, DFRobot_GAS_I2C &sensor) {
  if (sensor.dataIsAvailable()) {
    Serial.println("========================");
    Serial.print(sensorName);
    Serial.println(" Data:");

    String gasType = sensor.queryGasType();
    Serial.print("Gas type: ");
    Serial.println(gasType);

    float gasConcentration = sensor.readGasConcentrationPPM();
    Serial.print("Concentration: ");
    Serial.print(gasConcentration);
    Serial.println(gasType.equals("O2") ? " %VOL" : " PPM");

    float temperature = sensor.readTempC();
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" ℃");

    Serial.println("========================");
  } else {
    Serial.print(sensorName);
    Serial.println(": No data available.");
  }
}

void connectToWiFi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 30000) {  // 30-second timeout
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed. Check credentials.");
  }
}

bool areSensorsEnabled() {
  // Check if DIP switches 3 and 4 are enabled
  return digitalRead(DIP_SWITCH_3_PIN) == HIGH && digitalRead(DIP_SWITCH_4_PIN) == HIGH;
}

bool isWiFiEnabled() {
  // Check if DIP switches 5, 6, and 7 are enabled
  return digitalRead(DIP_SWITCH_5_PIN) == HIGH && digitalRead(DIP_SWITCH_6_PIN) == HIGH && digitalRead(DIP_SWITCH_7_PIN) == HIGH;
}

void loadConfiguration() {
  deviceID = readStringFromEEPROM(DEVICE_ID_ADDR);
  ssid = readStringFromEEPROM(SSID_ADDR);
  password = readStringFromEEPROM(PASSWORD_ADDR);

  Serial.print("Device ID: ");
  Serial.println(deviceID);
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
}

void saveConfiguration() {
  writeStringToEEPROM(DEVICE_ID_ADDR, deviceID);
  writeStringToEEPROM(SSID_ADDR, ssid);
  writeStringToEEPROM(PASSWORD_ADDR, password);
  Serial.println("Configuration saved.");
}

void configureDevice() {
  Serial.println("Enter Device ID:");
  while (Serial.available() == 0) {}
  deviceID = Serial.readStringUntil('\n');
  deviceID.trim();

  Serial.println("Enter WiFi SSID:");
  while (Serial.available() == 0) {}
  ssid = Serial.readStringUntil('\n');
  ssid.trim();

  Serial.println("Enter WiFi Password:");
  while (Serial.available() == 0) {}
  password = Serial.readStringUntil('\n');
  password.trim();

  saveConfiguration();
}

String readStringFromEEPROM(int addr) {
  String result = "";
  for (int i = addr; i < EEPROM.length(); i++) {
    char c = EEPROM.read(i);
    if (c == '\0') break;
    result += c;
  }
  return result;
}

void writeStringToEEPROM(int addr, const String &data) {
  int len = data.length();
  for (int i = 0; i < len; i++) {
    EEPROM.write(addr + i, data[i]);
  }
  EEPROM.write(addr + len, '\0');
}
