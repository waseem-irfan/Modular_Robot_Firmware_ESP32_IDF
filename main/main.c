#include <stdio.h>
#include <string.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include <stdlib.h>

// #include "wifi_manager.h"
#include "protocol_examples_common.h"
#define ESP_NOW_TAG "ESP_NOW"

uint8_t esp_1[6] = {0xC0, 0x5D, 0x89, 0xAF, 0xA0, 0x84};
uint8_t esp_2[6] = {0xC0, 0x5D, 0x89, 0xAF, 0xD6, 0x28};


/* @brief tag used for ESP serial console messages */
static const char TAG[] = "main";

/**
 * @brief RTOS task that periodically prints the heap memory available.
 * @note Pure debug information, should not be ever started on production code! This is an example on how you can integrate your code with wifi-manager
 */
void monitoring_task(void *pvParameter)
{
	for(;;){
		ESP_LOGI(TAG, "free heap: %lu",esp_get_free_heap_size());
		vTaskDelay( pdMS_TO_TICKS(10000) );
	}
}


/**
 * @brief this is an exemple of a callback that you can setup in your own app to get notified of wifi manager event.
 */
void cb_connection_ok(void *pvParameter){
	ip_event_got_ip_t* param = (ip_event_got_ip_t*)pvParameter;

	/* transform IP to human readable string */
	char str_ip[16];
	esp_ip4addr_ntoa(&param->ip_info.ip, str_ip, IP4ADDR_STRLEN_MAX);

	ESP_LOGI(TAG, "I have a connection and my IP is %s!", str_ip);
}

typedef struct
{
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} mpu6050_data_t;

char *mac_to_str(char *my_mac_str, uint8_t *my_mac){
	sprintf(my_mac_str,"%02x%02x%02x%02x%02x%02x",my_mac[0],my_mac[1],my_mac[2],my_mac[3],my_mac[4],my_mac[5]);
	return my_mac_str;
}

void on_sent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  char buffer[13];
  switch (status)
  {
  case ESP_NOW_SEND_SUCCESS:
    ESP_LOGI(TAG, "message sent to %s", mac_to_str(buffer, (uint8_t *)mac_addr));
    break;
  case ESP_NOW_SEND_FAIL:
    ESP_LOGE(TAG, "message sent to %s failed", mac_to_str(buffer, (uint8_t *)mac_addr));
    break;
  }
}

void on_receive(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    if (data_len == sizeof(mpu6050_data_t))
    {
        mpu6050_data_t received_data;
        memcpy(&received_data, data, sizeof(mpu6050_data_t));

        ESP_LOGI(ESP_NOW_TAG, "Received MPU6050 Data:");
        ESP_LOGI(ESP_NOW_TAG, "Accel: X=%.2fg, Y=%.2fg, Z=%.2fg", received_data.accel_x, received_data.accel_y, received_data.accel_z);
        ESP_LOGI(ESP_NOW_TAG, "Gyro: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s", received_data.gyro_x, received_data.gyro_y, received_data.gyro_z);
    }
}

void app_main()
{
	/* start the wifi manager */
	// wifi_manager_start();

	/* register a callback as an example to how you can integrate your code with the wifi manager */
	// wifi_manager_set_callback(WM_EVENT_STA_GOT_IP, &cb_connection_ok);

	/* your code should go here. Here we simply create a task on core 2 that monitors free heap memory */
	// xTaskCreatePinnedToCore(&monitoring_task, "monitoring_task", 2048, NULL, 1, NULL, 1);

	uint8_t my_mac[6];
	esp_efuse_mac_get_default(my_mac);
	char my_mac_str[13];
	ESP_LOGI(ESP_NOW_TAG,"My Mac Address : %s",mac_to_str(my_mac_str,my_mac));
	bool is_current_esp1 = memcmp(my_mac, esp_1, 6) == 0;
  	uint8_t *peer_mac = is_current_esp1 ? esp_2 : esp_1;

	ESP_ERROR_CHECK(nvs_flash_init());
	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(example_connect());

	ESP_ERROR_CHECK(esp_now_init());
  	ESP_ERROR_CHECK(esp_now_register_send_cb(on_sent));
  	ESP_ERROR_CHECK(esp_now_register_recv_cb(on_receive));

	   while (1)
    {
        mpu6050_data_t sensor_data;

        // Simulated MPU6050 Data (Random but realistic values)
        sensor_data.accel_x = 0.10 + ((rand() % 20) / 100.0);  // ±0.20g variation
        sensor_data.accel_y = -0.05 + ((rand() % 20) / 100.0); // ±0.20g variation
        sensor_data.accel_z = 9.81 + ((rand() % 10) / 100.0);  // ±0.10g variation
        sensor_data.gyro_x = 0.01 + ((rand() % 10) / 100.0);   // ±0.10°/s variation
        sensor_data.gyro_y = -0.01 + ((rand() % 10) / 100.0);  // ±0.10°/s variation
        sensor_data.gyro_z = 0.02 + ((rand() % 10) / 100.0);   // ±0.10°/s variation

        ESP_LOGI(ESP_NOW_TAG, "Simulated MPU6050 Data - Accel: X=%.2fg, Y=%.2fg, Z=%.2fg, Gyro: X=%.2f°/s, Y=%.2f°/s, Z=%.2f°/s",
                 sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z,
                 sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);

        ESP_ERROR_CHECK(esp_now_send(peer_mac, (uint8_t *)&sensor_data, sizeof(sensor_data)));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}
