idf_component_register(
    SRCS "ads1220.c"
    INCLUDE_DIRS "include"
    REQUIRES driver  # Add any ESP-IDF components you depend on
)