idf_component_register(
    SRCS "ina260.c"
    INCLUDE_DIRS "include"
    REQUIRES driver cJSON esp_timer
)