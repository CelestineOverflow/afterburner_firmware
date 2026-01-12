idf_component_register(
    SRCS "pd.c"
    INCLUDE_DIRS "include"
    REQUIRES driver cJSON esp_timer
)