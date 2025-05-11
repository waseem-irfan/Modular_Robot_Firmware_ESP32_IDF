#include "esp_event.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include "mqtt_comm.h"
#include "esp_timer.h" // <-- Add for esp_timer_get_time()

#define MQTT_TAG "MQTT"
#define UWB_TIMEOUT_US 100000 // 100 ms

static esp_mqtt_client_handle_t client = NULL;

QueueHandle_t uwb_distances_queue; // Single queue for main app

typedef struct
{
    double A1;
    double A2;
    double A3;
    bool got_A1;
    bool got_A2;
    bool got_A3;
    int64_t start_time;
} buffered_uwb_t;

static buffered_uwb_t distances_buffer = {0};

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_subscribe(client, "/masterBOT/UWB/A1", 1);
        esp_mqtt_client_subscribe(client, "/masterBOT/UWB/A2", 1);
        esp_mqtt_client_subscribe(client, "/masterBOT/UWB/A3", 1);
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
        printf("topic: %.*s\n", event->topic_len, event->topic);
        printf("message: %.*s\n", event->data_len, event->data);

        char topic[100] = {0};
        memcpy(topic, event->topic, event->topic_len);
        topic[event->topic_len] = '\0';

        double uwb_distance = 0.0;
        char payload[50] = {0};
        int len = (event->data_len < sizeof(payload) - 1) ? event->data_len : sizeof(payload) - 1;
        memcpy(payload, event->data, len);
        payload[len] = '\0';

        if (sscanf(payload, "DIS: %lf cm", &uwb_distance) == 1 && uwb_distance > 0 && uwb_distance < 1000)
        {
            printf("Parsed distance: %.2f from topic: %s\n", uwb_distance, topic);

            int64_t now = esp_timer_get_time();

            if (distances_buffer.start_time == 0)
            {
                distances_buffer.start_time = now;
            }

            if (strcmp(topic, "/masterBOT/UWB/A1") == 0)
            {
                distances_buffer.A1 = uwb_distance;
                distances_buffer.got_A1 = true;
            }
            if (strcmp(topic, "/masterBOT/UWB/A2") == 0)
            {
                distances_buffer.A2 = uwb_distance;
                distances_buffer.got_A2 = true;
            }
            if (strcmp(topic, "/masterBOT/UWB/A3") == 0)
            {
                distances_buffer.A3 = uwb_distance;
                distances_buffer.got_A3 = true;
            }

            bool all_received = distances_buffer.got_A1 && distances_buffer.got_A2 && distances_buffer.got_A3;
            bool timeout_passed = (now - distances_buffer.start_time) >= UWB_TIMEOUT_US;

            if (all_received || timeout_passed)
            {
                if (all_received)
                {
                    uwb_distances_t result = {
                        .A1 = distances_buffer.A1,
                        .A2 = distances_buffer.A2,
                        .A3 = distances_buffer.A3};
                    if (xQueueSend(uwb_distances_queue, &result, 0) == pdPASS)
                    {
                        printf("Sent all distances to queue\n");
                    }
                    else
                    {
                        printf("Failed to send to queue\n");
                    }
                }
                else
                {
                    printf("Timeout occurred before all distances received, discarding partial data.\n");
                }

                // Reset buffer
                distances_buffer = (buffered_uwb_t){0};
            }
        }
        else
        {
            printf("Failed to parse or invalid distance\n");
        }

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGE(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
        break;

    case MQTT_EVENT_SUBSCRIBED:
    case MQTT_EVENT_UNSUBSCRIBED:
    case MQTT_EVENT_PUBLISHED:
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(MQTT_TAG, "ERROR %s", strerror(event->error_handle->esp_transport_sock_errno));
        break;

    default:
        break;
    }
}

void mqtt_start(void)
{
    printf("WIFI Initialized\n");
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://mqtt.eclipseprojects.io:1883"};
    client = esp_mqtt_client_init(&mqtt_cfg);
    printf("Client Initialized\n");
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

int mqtt_send(const char *topic, const char *payload)
{
    return esp_mqtt_client_publish(client, topic, payload, strlen(payload), 1, 0);
}
// Task to send message
// void test_send_messages(void *param)
// {

//     char message[50];
//     while (true)
//     {
//         sprintf(message, "X = 22, Y= 10, Z=0, Theta = 35");
//         mqtt_send("/masterBOT/UWB/A1", message);
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }