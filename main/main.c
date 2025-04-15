#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

SemaphoreHandle_t mutex;

void I2C_Bus(char * msg){
    printf(msg);
}

void Reading_MPU6050(void * params){
    while(1){
        printf("Reading MPU6050\n");
    if(xSemaphoreTake(mutex, pdMS_TO_TICKS(100))){
        I2C_Bus("X = 88\n");
        xSemaphoreGive(mutex);
    }
    else{
        printf("MPU6050 timed out\n");
    } 
    vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void Reading_Pulses(void * params){
    while(1){
        printf("Reading Pulses\n");
    if(xSemaphoreTake(mutex, pdMS_TO_TICKS(100))){
        I2C_Bus("Pulses = 2222\n");
        xSemaphoreGive(mutex);
    }
    else{
        printf("Pulses timed out\n");
    } 
    vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void MQTT_Publisher(void* params){
    while(1){
        printf("Publishing MPU6050 & Pulses\n");
            if(xSemaphoreTake(mutex, pdMS_TO_TICKS(200))){
                I2C_Bus("published X=88\n");
                I2C_Bus("Published Pulses = 2222\n");
        }
        else{
                printf("MQTT_Publisher timed out\n");
        } 
        xSemaphoreGive(mutex);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void){
    mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(Reading_MPU6050,"Reading MPU", 1024*2, NULL,2,NULL,1);
    xTaskCreatePinnedToCore(Reading_Pulses,"Reading Pulses", 1024*2, NULL,2,NULL,1);
    xTaskCreatePinnedToCore(MQTT_Publisher,"Publisher", 1024*2, NULL,2,NULL,0);
}
