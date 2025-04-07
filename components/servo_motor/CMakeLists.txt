set(srcs 
    "servo_motor.c"
    )

# "glcdfont.c" defined but not used font
idf_component_register(SRCS ${srcs}
    INCLUDE_DIRS "inc"
    REQUIRES driver)