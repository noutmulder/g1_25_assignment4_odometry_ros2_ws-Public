#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "mqtt_client.h"
#include "wifi_mqtt_config.h"

static const char *TAG_WIFI = "WiFi";
static const char *TAG_MQTT = "MQTT";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;
static bool wifi_ap_started = false;
static esp_mqtt_client_handle_t mqtt_client = NULL;
/* MQTT connected state - only true after receiving MQTT_EVENT_CONNECTED */
static bool mqtt_connected = false;
/* Queue used to buffer outgoing MQTT messages while disconnected */
#define MQTT_PUB_QUEUE_LEN 12
#define MQTT_PUB_MSG_SIZE 256
static QueueHandle_t mqtt_pub_queue = NULL;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG_WIFI, "WiFi AP started");
        wifi_ap_started = true;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Station " MACSTR " joined, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG_WIFI, "Station " MACSTR " left, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    
    // Configure AP IP address
    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 4, 1);        // ESP32 AP IP: 192.168.4.1
    IP4_ADDR(&ip_info.gw, 192, 168, 4, 1);        // Gateway: 192.168.4.1
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0); // Netmask: 255.255.255.0
    
    esp_netif_dhcps_stop(ap_netif);
    esp_netif_set_ip_info(ap_netif, &ip_info);
    esp_netif_dhcps_start(ap_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.ap.ssid, WIFI_SSID, sizeof(wifi_config.ap.ssid));
    strncpy((char*)wifi_config.ap.password, WIFI_PASS, sizeof(wifi_config.ap.password));
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    wifi_config.ap.channel = 1;
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_WIFI, "WiFi AP initialized. SSID:%s password:%s channel:%d",
             WIFI_SSID, WIFI_PASS, 1);
    ESP_LOGI(TAG_WIFI, "AP IP address: 192.168.4.1");
    
    // Wait for AP to start
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT,
            pdFALSE,
            pdFALSE,
            pdMS_TO_TICKS(5000));  // 5 second timeout
    
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG_WIFI, "WiFi AP started successfully");
    } else {
        ESP_LOGE(TAG_WIFI, "Failed to start WiFi AP");
    }
}

bool wifi_is_connected(void)
{
    return wifi_ap_started;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_CONNECTED");
        mqtt_connected = true;
        /* Flush queued messages */
        if (mqtt_pub_queue != NULL) {
            char queued_msg[MQTT_PUB_MSG_SIZE];
            while (xQueueReceive(mqtt_pub_queue, queued_msg, 0) == pdPASS) {
                int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, queued_msg, 0, 1, 0);
                ESP_LOGI(TAG_MQTT, "Flushed queued msg id=%d", msg_id);
                vTaskDelay(pdMS_TO_TICKS(10)); /* small spacing to avoid burst */
            }
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_DISCONNECTED");
        mqtt_connected = false;
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG_MQTT, "MQTT_EVENT_ERROR");
        mqtt_connected = false;
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other event id:%d", event->event_id);
        break;
    }
}

void mqtt_app_start(void)
{
    /* Ensure the publish queue exists before starting client */
    if (mqtt_pub_queue == NULL) {
        mqtt_pub_queue = xQueueCreate(MQTT_PUB_QUEUE_LEN, MQTT_PUB_MSG_SIZE);
        if (mqtt_pub_queue == NULL) {
            ESP_LOGW(TAG_MQTT, "Failed to create MQTT publish queue - outgoing messages may be lost");
        }
    }
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = MQTT_BROKER_URI,
            },
        },
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    
    ESP_LOGI(TAG_MQTT, "MQTT client started, broker: %s", MQTT_BROKER_URI);
}

void mqtt_publish_imu_data(const char *data)
{
    if (mqtt_client != NULL && mqtt_connected) {
        int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, data, 0, 1, 0);
        ESP_LOGD(TAG_MQTT, "Published to topic %s, msg_id=%d", MQTT_TOPIC, msg_id);
        return;
    }

    /* Not connected: try to enqueue message for later flushing */
    if (mqtt_pub_queue == NULL) {
        ESP_LOGW(TAG_MQTT, "No pub queue available, dropping message");
        return;
    }

    char buf[MQTT_PUB_MSG_SIZE];
    size_t copy_len = strnlen(data, MQTT_PUB_MSG_SIZE - 1);
    memcpy(buf, data, copy_len);
    buf[copy_len] = '\0';

    /* Non-blocking send: if full, drop the oldest and enqueue new message */
    if (xQueueSend(mqtt_pub_queue, buf, 0) != pdPASS) {
        char discard[MQTT_PUB_MSG_SIZE];
        if (xQueueReceive(mqtt_pub_queue, discard, 0) == pdPASS) {
            ESP_LOGW(TAG_MQTT, "MQTT publish queue full, dropped oldest message");
        }
        /* Try enqueue again (non-blocking) */
        if (xQueueSend(mqtt_pub_queue, buf, 0) != pdPASS) {
            ESP_LOGW(TAG_MQTT, "Failed to enqueue MQTT message, dropping");
        }
    }
}

bool mqtt_is_connected(void)
{
    /* mqtt_client != NULL indicates client object exists, mqtt_connected indicates actual connection state */
    return (mqtt_client != NULL) && mqtt_connected;
}
