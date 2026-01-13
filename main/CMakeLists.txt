idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES led_control ina260 ads1220 max31865 pd cJSON pid board
)

# Add version definitions to this component
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    FW_VERSION_MAJOR=${CMAKE_PROJECT_VERSION_MAJOR}
    FW_VERSION_MINOR=${CMAKE_PROJECT_VERSION_MINOR}
    FW_VERSION_PATCH=${CMAKE_PROJECT_VERSION_PATCH}
    FW_VERSION="${CMAKE_PROJECT_VERSION}"
)