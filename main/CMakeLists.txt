idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver nvs_flash  wifi_connect mqtt_comm mpu6050_rpy servo_motor arduino-esp32 trilateration_algo)
