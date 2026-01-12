idf_component_register(
    SRCS "ads1220.c"
    INCLUDE_DIRS "include"
    REQUIRES driver cJSON esp_timer
)