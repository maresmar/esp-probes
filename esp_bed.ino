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
const byte mqtt_port = MQTT_PORT;
const char mqtt_topic[] = MQTT_TOPIC;

const int sleep_sec = 5 * 60;
int connectionTime = -1;

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
  // pinMode(LED_BUILTIN, OUTPUT);
  // digitalWrite(LED_BUILTIN, HIGH);
  pinMode(D0, WAKEUP_PULLUP);
  pinMode(D5, OUTPUT);

  Serial.begin(74880);

  system_deep_sleep_set_option(2);

  setupWifi();
  waitWifi();
  connectionTime = millis();
  digitalWrite(D5, HIGH);
  setupMqtt();
  setupSensors();
}

inline void setupWifi()
{
  WiFi.mode(WIFI_STA);
  wifi_set_sleep_type(LIGHT_SLEEP_T);
  WiFi.begin(wifi_ssid, wifi_pass, wifi_chanel, wifi_bssid);
}

inline void waitWifi()
{
  WiFi.waitForConnectResult(10000);
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("WiFi didn't connect");
    handleError();
  }
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

void loop()
{
  mqtt_client.beginMessage(mqtt_topic);

  const float sht31_temp = sht31.readTemperature();
  const float bmp_temp = bmp.readTemperature();

  Serial.print("Temperature = ");
  Serial.print(sht31_temp);
  Serial.print(" / ");
  Serial.print(bmp_temp);
  Serial.println(" *C");

  mqtt_client.print("{\"temp\": ");
  mqtt_client.print(sht31_temp);
  mqtt_client.print(", \"bmp_temp\": ");
  mqtt_client.print(bmp_temp);

  const float humidity = sht31.readHumidity();

  Serial.print("Humidity = ");
  Serial.print(humidity);
  Serial.println(" %");

  mqtt_client.print(", \"humidity\": ");
  mqtt_client.print(humidity);

  const float hPa = bmp.readPressure() / 100.0;

  Serial.print("Pressure = ");
  Serial.print(hPa);
  Serial.println(" hPa");

  mqtt_client.print(", \"pressure\": ");
  mqtt_client.print(hPa);

  // We have two resistors (72k + 92k) Ohms
  const int mVoltage = 3300 * (719 + 927) / 719 * analogRead(A0) / 1024;

  Serial.print("Battery = ");
  Serial.print((float)mVoltage / 1000);
  Serial.println(" V");
  mqtt_client.print(", \"battery\": ");
  mqtt_client.print((float)mVoltage / 1000);

  digitalWrite(D5, LOW);

  const auto rssi = WiFi.RSSI();
  Serial.print("RSSI = ");
  mqtt_client.print(", \"rssi\": ");
  mqtt_client.print(rssi);
  Serial.print(rssi);
  Serial.println(" db");

  Serial.print("connectionTime = ");
  mqtt_client.print(", \"connectionTime\": ");
  mqtt_client.print(connectionTime);
  Serial.print(connectionTime);
  Serial.println(" ms");

  Serial.println();
  mqtt_client.print("}");
  mqtt_client.endMessage();

  delay(2000);
  system_deep_sleep_instant((sleep_sec - 2) * 1e6);
}
