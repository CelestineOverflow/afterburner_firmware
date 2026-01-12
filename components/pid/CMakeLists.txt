idf_component_register(
    SRCS "pid.c"
    INCLUDE_DIRS "include"
    REQUIRES driver  # Add any ESP-IDF components you depend on
)