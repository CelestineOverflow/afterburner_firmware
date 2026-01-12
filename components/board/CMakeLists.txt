idf_component_register(
    SRCS "board.c"
    INCLUDE_DIRS "include"
    REQUIRES driver  # Add any ESP-IDF components you depend on
)