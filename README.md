# ESP-probes

Created for ESP8266 board.

## Configuration

The `env.h` file is needed with following content:

```c
#define WIFI_ESSID    "MyWiFi"
#define WIFI_PASSWORD "secretpassword"

#define MQTT_HOST "192.168.0.1"
#define MQTT_PORT 1883
#define MQTT_PREFIX "home/room"
```
