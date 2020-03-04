
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <WiFiNINA.h>
#include <MQTT.h>
#include <MQTTClient.h>


// Variables and constants for bme680
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C


// Variables and constants for wifi client
char ssid[] = "";    // SSID of the wifi network
char pass[] = "";   // Password of the wifi network
int keyIndex = 0;
int status = WL_IDLE_STATUS;            // Wifi status code
char server[] = "www.google.com";       // The target address to connect the wifi client
WiFiClient net;                         // Initialize the Ethernet client library


// Variables and constants for mqtt client
char mqttAddress[] = "";  // The destination server address of the mqtt broker
int mqttPort = 1883;                   // The port of the mqtt broker
const char key[] = " ";                 // Username to connect to the broker
const char secret[] = " ";              // Password of the mqtt broker
const char device[] = " ";         // Unique id to identifier the connected clients.
MQTTClient client;                      // Initialize the mqtt client instance


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (true);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // Check firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // Attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  Serial.println("Connected to wifi");
  printWifiStatus();

  // Attempt to connect to mqtt broker
  client.begin(mqttAddress, mqttPort, net);

  Serial.println("Connecting to broker...");
  while (!client.connect(device, key, secret)) {
    Serial.print(".");
    delay(1000);
  }
}

void loop() {
  // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }

  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println(F("You can do other work during BME680 measurement."));
  delay(50); // This represents parallel work.
  // There's no need to delay() until millis() >= endTime: bme.endReading()
  // takes care of that. It's okay for parallel work to take longer than
  // BME680's measurement time.

  // Obtain measurement results from BME680. Note that this operation isn't
  // instantaneous even if milli() >= endTime due to I2C/SPI latency.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.print(F("Temperature = "));
  String str_temp = String(bme.temperature);
  client.publish("/tesseract/nodes/tn-003/temperature", str_temp);
  Serial.print(str_temp);
  Serial.println(F(" *C"));

  Serial.print(F("Pressure = "));
  String str_pressure = String(bme.pressure);
  client.publish("/tesseract/nodes/tn-003/pressure", str_pressure);
  Serial.print(str_pressure);
  Serial.println(F(" hPa"));

  Serial.print(F("Humidity = "));
  String str_humidity = String(bme.humidity);
  client.publish("/tesseract/nodes/tn-003/humidity", str_humidity);
  Serial.print(str_humidity);
  Serial.println(F(" %"));

  Serial.print(F("Gas = "));
  String str_gas = String(bme.gas_resistance);
  client.publish("/tesseract/nodes/tn-003/gas", str_gas);
  Serial.print(str_gas);
  Serial.println(F(" Ohms"));

  Serial.print(F("Approx. Altitude = "));
  String str_altitude = String(bme.readAltitude(SEALEVELPRESSURE_HPA));
  client.publish("/tesseract/nodes/tn-003/altitude", str_altitude);
  Serial.print(str_altitude);
  Serial.println(F(" m"));

  Serial.println();
  delay(10000);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
