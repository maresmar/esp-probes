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

char wifi_ssid[] = WIFI_ESSID;
char wifi_pass[] = WIFI_PASSWORD;

const char mqtt_broker[] = MQTT_HOST;
const int  mqtt_port = MQTT_PORT;
const char mqtt_topic[] = MQTT_TOPIC;

const int sleep_sec = 5*60;

Adafruit_SHT31 sht31;
Adafruit_BMP085 bmp;
WiFiClient wifi_client;
MqttClient mqtt_client(wifi_client);

void setup()
{
  // Input - output
  pinMode(D0, WAKEUP_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(9600);

  system_deep_sleep_set_option(2);

  setupWifi();
  setupSensors();
  waitWifi();
  setupMqtt();
}

inline void setupWifi()
{
  WiFi.mode(WIFI_STA);
  wifi_set_sleep_type(LIGHT_SLEEP_T);
  WiFi.begin(wifi_ssid, wifi_pass);
}

inline void waitWifi()
{
  WiFi.waitForConnectResult(10000);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print("WiFi didn't connect");
    Serial.flush();
    system_deep_sleep_instant(sleep_sec * 1e6);
  }
}

inline void setupSensors()
{
  // Setup  BMP180
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    Serial.flush();
    system_deep_sleep_instant(sleep_sec * 1e6);
  }

  // Setup SHT31
  if (!sht31.begin(0x44))
  {
    Serial.println("Could not find a valid SHT3x sensor, check wiring!");
    Serial.flush();
    system_deep_sleep_instant(sleep_sec * 1e6);
  }
}

inline void setupMqtt()
{
  if (!mqtt_client.connect(mqtt_broker, mqtt_port))
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqtt_client.connectError());
    Serial.flush();
    system_deep_sleep_instant(sleep_sec * 1e6);
  }
}

void loop()
{
  mqtt_client.beginMessage(mqtt_topic);

  Serial.print("Temperature = ");
  mqtt_client.print("{\"temp\": ");
  Serial.print(sht31.readTemperature());
  mqtt_client.print(sht31.readTemperature());

  Serial.print(" / ");
  mqtt_client.print(", \"bmp_temp\": ");
  Serial.print(bmp.readTemperature());
  mqtt_client.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print("Humidity = ");
  mqtt_client.print(", \"humidity\": ");
  Serial.print(sht31.readHumidity());
  mqtt_client.print(sht31.readHumidity());
  Serial.println(" %");

  Serial.print("Pressure = ");
  mqtt_client.print(", \"pressure\": ");
  float hPa = bmp.readPressure() / 100.0;
  mqtt_client.print(hPa);
  Serial.print(hPa);
  Serial.println(" hPa");

  Serial.print("RSSI = ");
  mqtt_client.print(", \"rssi\": ");
  mqtt_client.print(WiFi.RSSI());
  Serial.print(WiFi.RSSI());
  Serial.println(" db");

  Serial.print("Battery = ");
  mqtt_client.print(", \"battery\": ");
  // We have two resistors (72k + 92k) Ohms
  int mVoltage = 3300 * (719 + 927) / 719 * analogRead(A0) / 1024;
  mqtt_client.print((float)mVoltage / 1000);
  Serial.print((float)mVoltage / 1000);
  Serial.println(" V");

  Serial.println();
  mqtt_client.print("}");
  mqtt_client.endMessage();

  delay(1000);
  system_deep_sleep_instant((sleep_sec - 1) * 1e6);
}
