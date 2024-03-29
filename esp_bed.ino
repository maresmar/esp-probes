/*
 * ESP32 Probe to broadcast temperature.
 * Martin Mareš, 2023
 */

#include <Adafruit_BMP085.h>
#include <Adafruit_SHT31.h>
#include <ArduinoMqttClient.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>

#include "env.h"

const char wifi_ssid[] = WIFI_ESSID;
const char wifi_pass[] = WIFI_PASSWORD;
const byte wifi_chanel = WIFI_CHANEL;
const byte wifi_bssid[] = WIFI_BSSID;

const char mqtt_broker[] = MQTT_HOST;
const int mqtt_port = MQTT_PORT;
const char mqtt_topic[] = MQTT_TOPIC;

const int sleep_sec = SLEEP_SEC;
unsigned long connectionTime = 0;

static Adafruit_SHT31 sht31;
static Adafruit_BMP085 bmp;
static WiFiClient wifi_client;
static MqttClient mqtt_client(wifi_client);

void handleError()
{
  Serial.println("Going sleep...");
  Serial.flush();
  delay(5000);
  digitalWrite(D5, LOW);
  system_deep_sleep_instant((sleep_sec - 5) * 1e6);
}

void setup()
{
  // Input - output
  pinMode(D0, WAKEUP_PULLUP);
  pinMode(D5, OUTPUT);
  // Turn on sensors
  digitalWrite(D5, HIGH);

  Serial.begin(74880);

  setupSensors();
}

inline void setupWifi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass, wifi_chanel, wifi_bssid);
  if (WiFi.waitForConnectResult(10000) == -1)
  {
    Serial.println("WiFi didn't connect");
    handleError();
  }
  connectionTime = millis();
}

inline void setupSensors()
{
  // Setup  BMP180
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    handleError();
  }

  // Setup SHT31
  if (!sht31.begin(0x44))
  {
    Serial.println("Could not find a valid SHT3x sensor, check wiring!");
    handleError();
  }
}

inline void setupMqtt()
{
  if (!mqtt_client.connect(mqtt_broker, mqtt_port))
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqtt_client.connectError());
    handleError();
  }
}

template <typename T>
inline void logField(const char *prefix, const T &value, const char *suffix)
{
  Serial.print(prefix);
  Serial.print(value);
  Serial.println(suffix);
}

void loop()
{
  const float sht31_temp = sht31.readTemperature();
  const float humidity = sht31.readHumidity();
  const float hPa = bmp.readPressure() / 100.0;
  const float bmp_temp = bmp.readTemperature();

  // We have two resistors (72k + 92k) Ohms
  const int mVoltage = 3300 * (719 + 927) / 719 * analogRead(A0) / 1024;

  // Turn off sensors
  digitalWrite(D5, LOW);

#ifdef debug
  Serial.print("Temperature = ");
  Serial.print(sht31_temp);
  Serial.print(" / ");
  Serial.print(bmp_temp);
  Serial.println(" *C");
  logField("Humidity = ", humidity, " %");
  logField("Pressure = ", hPa, " hPa");
  logField("Battery = ", (float)mVoltage / 1000, " V");
#endif

  setupWifi();
  setupMqtt();
  const auto rssi = WiFi.RSSI();

#ifdef debug
  logField("RSSI = ", rssi, " db");
  logField("time = ", connectionTime, " ms");
#endif

  mqtt_client.beginMessage(mqtt_topic);
  mqtt_client.print("{\"temp\": ");
  mqtt_client.print(sht31_temp);
  mqtt_client.print(", \"bmp_temp\": ");
  mqtt_client.print(bmp_temp);
  mqtt_client.print(", \"humidity\": ");
  mqtt_client.print(humidity);
  mqtt_client.print(", \"pressure\": ");
  mqtt_client.print(hPa);
  mqtt_client.print(", \"battery\": ");
  mqtt_client.print((float)mVoltage / 1000);
  mqtt_client.print(", \"rssi\": ");
  mqtt_client.print(rssi);
  mqtt_client.print(", \"connectionTime\": ");
  mqtt_client.print(connectionTime);
  mqtt_client.print("}");
  mqtt_client.endMessage();

  mqtt_client.flush();
  wifi_client.flush();

  system_deep_sleep_instant((sleep_sec) * 1e6);
}
