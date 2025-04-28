idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES dcEncoder_motor wifi_connect mqtt_comm mpu6050_rpy pid_controller arduino-esp32)
