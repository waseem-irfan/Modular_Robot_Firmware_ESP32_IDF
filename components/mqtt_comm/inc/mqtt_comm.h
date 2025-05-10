#ifndef MQTT_COMM_H
#define MQTT_COMM_H
#include "esp_event.h"
#include "freertos/queue.h"

extern QueueHandle_t uwb_distances_queue;

typedef struct {
    double A1;
    double A2;
    double A3;
} uwb_distances_t;

void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void mqtt_start(void);
int mqtt_send(const char *topic, const char *payload);
// void test_send_messages(void *param);

#endif