idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES led_control ina260 ads1220 max31865 pd cJSON 
)