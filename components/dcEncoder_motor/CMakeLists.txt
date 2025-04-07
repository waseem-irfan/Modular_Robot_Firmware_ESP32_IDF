idf_component_register(SRCS "motor.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES bdc_motor driver esp_timer)