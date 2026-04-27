cmake_minimum_required(VERSION 3.5)

# Step 1: fetch the version from git
execute_process(
    COMMAND git describe --tags --always --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)
message(STATUS "git describe: '${GIT_DESCRIBE}'")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(test VERSION 1.0.3)