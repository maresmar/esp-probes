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
int mqtt_port = MQTT_PORT;
const char mqtt_topic[] = MQTT_TOPIC;

Adafruit_SHT31 sht31;
Adafruit_BMP085 bmp;
WiFiClient wifi_client;
MqttClient mqtt_client(wifi_client);

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.begin(9600);

  // Setup WiFi
  Serial.print("Attempting to connect to WIFI SSID: ");
  Serial.println(wifi_ssid);

  WiFi.onEvent(onStaGotIp, WIFI_EVENT_STAMODE_GOT_IP);
  WiFi.onEvent(onStaDisconnected, WIFI_EVENT_STAMODE_DISCONNECTED);
  WiFi.begin(wifi_ssid, wifi_pass);

  // Setup  BMP180
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    while (1)
      ;
  }

  // Setup SHT31
  if (!sht31.begin(0x44))
  {
    Serial.println("Could not find a valid SHT3x sensor, check wiring!");
    while (1)
      ;
  }

  Serial.println("Waiting on WiFi connection:");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(2000);
  }
}

void onStaGotIp(WiFiEvent_t event)
{
  Serial.print("WiFi connected, ");
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
}

void onStaDisconnected(WiFiEvent_t event)
{
  Serial.println("WiFi lost");
}

void setupMqtt()
{
  Serial.print("Attempting to connect to the MQTT broker: ");
  Serial.println(mqtt_broker);

  if (!mqtt_client.connect(mqtt_broker, mqtt_port))
  {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqtt_client.connectError());
  }
  else
  {
    Serial.println("You're connected to the MQTT broker!");
    digitalWrite(LED_BUILTIN, HIGH);
  }
}

void loop()
{
  if(WiFi.isConnected() && !mqtt_client.connected()) {
    setupMqtt();
  }
  if(mqtt_client.connected()) {
    mqtt_client.poll();
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

    Serial.println();
    mqtt_client.print("}");
    mqtt_client.endMessage();
  }

  delay(60000);
}
