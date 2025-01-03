#include <PMS.h>
#include "DFRobot_MultiGasSensor.h"
#include "DHT.h"
#include <SoftwareSerial.h>
#include <MQ131.h>

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
#define RL 20.0  // Load resistance in kΩ
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

// Sensor read delay (in milliseconds)
#define SENSOR_READ_DELAY 2000

void setup() {
  Serial.begin(115200);     // Debug output
  Wire.begin();             // Initialize I2C bus
  dht.begin();              // Initialize the DHT22 sensor
  pmsSerial.begin(115200);  // Initialize PMS5003

  MQ131.begin(6, MQ131_PIN, LOW_CONCENTRATION, 1000000);
  MQ131.setTimeToRead(20);  // Set how many seconds we will read from the Ozone sensor
  MQ131.setR0(9000);        // Calibrated R0 value

  // Initialize gas sensors
  initializeSensor(sensor1, "Sensor 1");
  initializeSensor(sensor2, "Sensor 2");
  initializeSensor(sensor3, "Sensor 3");
}

void loop() {
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

  /* Old code for reference
  int analogValue = analogRead(MQ131_PIN);
  float voltage = (analogValue / 1023.0) * 5.0; // Convert to voltage (5V ADC)
  float ppm = MQ131_A * pow(voltage, MQ131_B);  // Calculate PPM
  Serial.println("MQ-131 Ozone Sensor:");
  Serial.print("Analog Value: ");
  Serial.println(analogValue);
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  Serial.print("Ozone Concentration: ");
  Serial.print(ppm, 2);
  Serial.println(" PPM");
  */
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

  /* Old code for reference
  int airQualityValue = analogRead(MP503_PIN);
  float voltage = airQualityValue * (5.0 / 1023.0);     // Convert to voltage
  Serial.print("MP503 Air Quality Sensor - Value: ");
  Serial.print(airQualityValue);
  Serial.print(" (");
  Serial.print(voltage);
  Serial.println(" V)");
  */
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

  /* Old code for reference
  if (pmsSerial.available() > 0) {
    for (int i = 0; i < PMS_BUFFER_SIZE; i++) {
      pmsBuffer[i] = pmsSerial.read();
    }
    if (pmsBuffer[0] == 0x42 && pmsBuffer[1] == 0x4D) {
      uint16_t pm1_0 = (pmsBuffer[10] << 8) | pmsBuffer[11];
      uint16_t pm2_5 = (pmsBuffer[12] << 8) | pmsBuffer[13];
      uint16_t pm10 = (pmsBuffer[14] << 8) | pmsBuffer[15];
      Serial.print("PMS5003 - PM1.0: "); Serial.print(pm1_0); Serial.println(" µg/m³");
      Serial.print("PMS5003 - PM2.5: "); Serial.print(pm2_5); Serial.println(" µg/m³");
      Serial.print("PMS5003 - PM10: "); Serial.print(pm10); Serial.println(" µg/m³");
    } else {
      Serial.println("PMS5003: Invalid data packet.");
    }
  } else {
    Serial.println("PMS5003: No data available.");
  }
  */
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

  /* Old code for reference
  int co2Value = analogRead(CO2_SENSOR_PIN);
  float voltage = co2Value * (5.0 / 1023.0);            // Convert to voltage
  float co2Concentration = voltage * 2000;              // Convert to ppm (based on Gravity CO2 sensor formula)
  Serial.print("Gravity CO2 Sensor - Concentration: ");
  Serial.print(co2Concentration);
  Serial.println(" ppm");
  */
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
    delay(1000);
  } else {
    Serial.print(name);
    Serial.println(" initialized!");
    sensor.changeAcquireMode(sensor.INITIATIVE); 
    delay(1000);
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

/*
 * =====================================================================================
 *                          WEB CONNECTION & DATABASE INTEGRATION
 * =====================================================================================
 * 
 * This device collects sensor data and can be connected to the web to send data to a
 * database for real-time visualization on a website.
 * 
 * 1. **Device ID**: Assign a unique ID to this device for identification.
 *    Example: const String DEVICE_ID = "DEVICE_001";
 * 
 * 2. **WiFi/Ethernet Setup**: Use a WiFi or Ethernet module (e.g., ESP8266, ESP32, or
 *    Ethernet Shield) to connect the device to the internet.
 *    Example for ESP8266:
 *      #include <ESP8266WiFi.h>
 *      const char* SSID = "Your_WiFi_SSID";
 *      const char* PASSWORD = "Your_WiFi_Password";
 *      WiFi.begin(SSID, PASSWORD);
 *      while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
 *      Serial.println("WiFi Connected!");
 * 
 * 3. **Database Setup**: Use a database (e.g., MySQL, Firebase, or MongoDB) to store
 *    sensor data. Create a table with fields like:
 *      - device_id (VARCHAR)
 *      - timestamp (DATETIME)
 *      - temperature (FLOAT)
 *      - humidity (FLOAT)
 *      - ozone_ppm (FLOAT)
 *      - voc_ppm (FLOAT)
 *      - pm2_5 (FLOAT)
 *      - co2_ppm (FLOAT)
 * 
 * 4. **API Endpoint**: Create an API endpoint to receive data from the device.
 *    Example: https://mar.com/api/sensor-data
 *    The API should accept a JSON payload like:
 *      {
 *        "device_id": "DEVICE_001",
 *        "timestamp": "2023-10-15T12:34:56Z",
 *        "temperature": 25.5,
 *        "humidity": 60.0,
 *        "ozone_ppm": 0.12,
 *        "voc_ppm": 1.5,
 *        "pm2_5": 10.2,
 *        "co2_ppm": 400.0
 *      }
 * 
 * 5. **Send Data to API**: Use HTTP POST requests to send sensor data to the API.
 *    Example for ESP8266:
 *      #include <ESP8266HTTPClient.h>
 *      HTTPClient http;
 *      http.begin("https://mar.com/api/sensor-data");
 *      http.addHeader("Content-Type", "application/json");
 *      String payload = "{\"device_id\":\"" + DEVICE_ID + "\",\"timestamp\":\"" + getTimestamp() + "\",\"temperature\":" + temp + ",\"humidity\":" + humidity + ",\"ozone_ppm\":" + ozonePpm + ",\"voc_ppm\":" + vocPpm + ",\"pm2_5\":" + pm2_5 + ",\"co2_ppm\":" + co2Ppm + "}";
 *      int httpResponseCode = http.POST(payload);
 *      if (httpResponseCode > 0) { Serial.println("Data sent successfully!"); }
 *      else { Serial.println("Failed to send data!"); }
 *      http.end();
 * 
 * 6. **Real-Time Visualization**: Use a web framework (e.g., React, Angular, or Vue.js)
 *    to create a dashboard that fetches data from the database and displays it in
 *    real-time using charts (e.g., Chart.js or D3.js).
 * 
 * 7. **Security**: Ensure the API is secured with HTTPS and authentication (e.g., API keys
 *    or OAuth) to prevent unauthorized access.
 * 
 * =====================================================================================
 */
