idf_component_register(SRCS "pid_control.c"
                    INCLUDE_DIRS "inc"
                    REQUIRES espressif__pid_ctrl
                    PRIV_REQUIRES dcEncoder_motor)