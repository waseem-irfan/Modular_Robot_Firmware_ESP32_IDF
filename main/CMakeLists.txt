idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES  wifi_connect mqtt_comm mpu6050_rpy servo_motor)
