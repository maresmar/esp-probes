# ESP-probes

Created for ESP8266 board.

## Configuration

The `env.h` file is needed with following content:

```c
#define WIFI_ESSID    "MyWiFi"
#define WIFI_PASSWORD "secretpassword"
#define WIFI_CHANEL 1
#define WIFI_BSSID {0x12, 0x34, 0x56, 0x78, 0x90, 0xAB}

#define MQTT_HOST "192.168.0.1"
#define MQTT_PORT 1883
#define MQTT_PREFIX "home/room"
```
