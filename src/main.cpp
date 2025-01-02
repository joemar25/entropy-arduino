#include "DFRobot_MultiGasSensor.h"
#include "DHT.h"
#include <SoftwareSerial.h>
#include <MQ131.h>

// DHT22 configuration
#define DHTPIN 7       // DHT22 data pin connected to pin 7
#define DHTTYPE DHT22  // DHT22 type
DHT dht(DHTPIN, DHTTYPE);

// Define I2C addresses for the gas sensors
#define SENSOR_1_ADDR 0x74
#define SENSOR_2_ADDR 0x75
#define SENSOR_3_ADDR 0x76

// Create gas sensor objects
DFRobot_GAS_I2C sensor1(&Wire, SENSOR_1_ADDR);
DFRobot_GAS_I2C sensor2(&Wire, SENSOR_2_ADDR);
DFRobot_GAS_I2C sensor3(&Wire, SENSOR_3_ADDR);

// MQ-131 Ozone Sensor configuration
#define MQ131_PIN A0

// MP503 Air Quality Sensor configuration
#define MP503_PIN A1

// Calibration constants for MQ-131 (example values)
#define MQ131_A 10.0
#define MQ131_B 1.5

// PMS5003 Particulate Matter Sensor configuration
SoftwareSerial pmsSerial(15, 14); // RX, TX for PMS5003
#define PMS_BUFFER_SIZE 32
uint8_t pmsBuffer[PMS_BUFFER_SIZE];

// Gravity CO2 Sensor configuration
#define CO2_SENSOR_PIN A2

void setup() {
  Serial.begin(115200); // Debug output
  Wire.begin();         // Initialize I2C bus
  dht.begin();          // Initialize the DHT22 sensor
  pmsSerial.begin(9600); // Initialize PMS5003

  // Initialize each gas sensor
  if (!sensor1.begin()) Serial.println("Sensor 1 not detected!");
  else Serial.println("Sensor 1 connected successfully!");

  if (!sensor2.begin()) Serial.println("Sensor 2 not detected!");
  else Serial.println("Sensor 2 connected successfully!");

  if (!sensor3.begin()) Serial.println("Sensor 3 not detected!");
  else Serial.println("Sensor 3 connected successfully!");
}

void loop() {
  Serial.println("========== All Sensor Data ==========");

  // DHT22
  displayDHT22Data();

  // MQ-131 Ozone Sensor
  displayMQ131Data();

  // MP503 Air Quality Sensor
  displayMP503Data();

  // PMS5003 Particulate Matter Sensor
  displayPMS5003Data();

  // Gravity CO2 Sensor
  displayCO2Data();

  // DFRobot Gas Sensors
  displayGasSensorData("Sensor 1", sensor1);
  displayGasSensorData("Sensor 2", sensor2);
  displayGasSensorData("Sensor 3", sensor3);

  Serial.println("=====================================");
  delay(2000); // Adjust delay as needed
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
  int analogValue = analogRead(MQ131_PIN);
  float voltage = (analogValue / 1023.0) * 5.0; // Convert to voltage (5V ADC)
  float ppm = MQ131_A * pow(voltage, MQ131_B); // Calculate PPM

  Serial.println("========================");
  Serial.println("MQ-131 Ozone Sensor:");
  Serial.print("Analog Value: ");
  Serial.println(analogValue);
  Serial.print("Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  Serial.print("Ozone Concentration: ");
  Serial.print(ppm, 2);
  Serial.println(" PPM");
  Serial.println("========================");
}

void displayMP503Data() {
  int airQualityValue = analogRead(MP503_PIN);
  float voltage = airQualityValue * (5.0 / 1023.0); // Convert to voltage
  Serial.print("MP503 Air Quality Sensor - Value: ");
  Serial.print(airQualityValue);
  Serial.print(" (");
  Serial.print(voltage);
  Serial.println(" V)");
}

void displayPMS5003Data() {
  if (pmsSerial.available() > 0) {
    // Read PMS5003 data into buffer
    for (int i = 0; i < PMS_BUFFER_SIZE; i++) {
      pmsBuffer[i] = pmsSerial.read();
    }

    // Verify packet header
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
}

void displayCO2Data() {
  int co2Value = analogRead(CO2_SENSOR_PIN);
  float voltage = co2Value * (5.0 / 1023.0); // Convert to voltage
  float co2Concentration = voltage * 2000;  // Convert to ppm (based on Gravity CO2 sensor formula)
  Serial.print("Gravity CO2 Sensor - Concentration: ");
  Serial.print(co2Concentration);
  Serial.println(" ppm");
}

// Function to initialize a gas sensor
void initializeSensor(DFRobot_GAS_I2C &sensor, const char *name) {
  if (!sensor.begin()) {
    Serial.print(name);
    Serial.println(" not found!");
    delay(1000);
  } else {
    Serial.print(name);
    Serial.println(" initialized!");
    sensor.changeAcquireMode(sensor.INITIATIVE); // Set to initiative mode
    delay(1000);
  }
}

// Function to display data for a gas sensor
void displayGasSensorData(const char *sensorName, DFRobot_GAS_I2C &sensor) {
  if (sensor.dataIsAvailable()) {
    Serial.println("========================");
    Serial.print(sensorName);
    Serial.println(" Data:");

    // Gas type
    String gasType = sensor.queryGasType();
    Serial.print("Gas type: ");
    Serial.println(gasType);

    // Gas concentration
    float gasConcentration = sensor.readGasConcentrationPPM();
    Serial.print("Concentration: ");
    Serial.print(gasConcentration);
    if (gasType.equals("O2")) {
      Serial.println(" %VOL");
    } else {
      Serial.println(" PPM");
    }

    // Temperature
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
