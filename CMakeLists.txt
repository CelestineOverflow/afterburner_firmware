cmake_minimum_required(VERSION 3.5)

execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

message(STATUS "git describe: '${GIT_DESCRIBE}'")

# remove 'v' prefix
string(REGEX REPLACE "^v" "" CLEAN_VERSION ${GIT_DESCRIBE})

# remove '-dirty'
string(REPLACE "-dirty" "" CLEAN_VERSION ${CLEAN_VERSION})

# OPTIONAL: remove commit hash suffix (e.g. 1.0.5-3-gabc123)
string(REGEX REPLACE "-[0-9]+-g[0-9a-f]+$" "" CLEAN_VERSION ${CLEAN_VERSION})

message(STATUS "clean version: '${CLEAN_VERSION}'")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test VERSION ${CLEAN_VERSION})

add_compile_definitions(GIT_VERSION="${GIT_DESCRIBE}")