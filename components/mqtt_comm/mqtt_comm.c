#include "esp_event.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MQTT_TAG "MQTT"
static esp_mqtt_client_handle_t client = NULL;

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data){
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
            esp_mqtt_client_subscribe(client, "/masterBOT/UWB", 1);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGE(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
        printf("topic: %.*s\n", event->topic_len, event->topic);
        printf("message: %.*s\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(MQTT_TAG, "ERROR %s", strerror(event->error_handle->esp_transport_sock_errno));
        break;
    default:
        break;
    }
    
}

void mqtt_start(void){
    printf("WIFI Initialized\n");
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://mqtt.eclipseprojects.io:1883"
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    printf("Client Initialized\n");
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID,mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}


int mqtt_send(const char *topic, const char *payload)
{
    return esp_mqtt_client_publish(client, topic, payload, strlen(payload), 1, 0);
}
// Task to send message
void test_send_messages(void *param)
{
    
    char message[50];
    while (true)
    {
        sprintf(message, "X = 22, Y= 10, Z=0, Theta = 35");
        mqtt_send("/masterBOT/UWB", message);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}