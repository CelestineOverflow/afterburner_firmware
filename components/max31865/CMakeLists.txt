idf_component_register(
    SRCS "max31865.c"
    INCLUDE_DIRS "include"
    REQUIRES driver  # Add any ESP-IDF components you depend on
)