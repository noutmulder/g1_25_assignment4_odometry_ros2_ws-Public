#ifndef WIFI_MQTT_CONFIG_H
#define WIFI_MQTT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

// WiFi Configuration
#define WIFI_SSID      CONFIG_WIFI_SSID
#define WIFI_PASS      CONFIG_WIFI_PASSWORD
#define WIFI_MAX_RETRY CONFIG_WIFI_MAX_RETRY

// MQTT Configuration
#define MQTT_BROKER_URI CONFIG_MQTT_BROKER_URI
#define MQTT_TOPIC      CONFIG_MQTT_TOPIC

// WiFi event handler
void wifi_init_sta(void);
bool wifi_is_connected(void);

// MQTT functions
void mqtt_app_start(void);
void mqtt_publish_imu_data(const char *data);
bool mqtt_is_connected(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_MQTT_CONFIG_H
