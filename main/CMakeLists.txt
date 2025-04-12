idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES dcEncoder_motor wifi_connect mqtt_comm mpu6050_rpy)
