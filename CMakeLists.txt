cmake_minimum_required(VERSION 3.22)
project(goldnet LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Header-only interface library
add_library(goldnet INTERFACE)
target_include_directories(goldnet INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Only the top-level project builds tests and examples by default, so a consumer's
# add_subdirectory(goldnet) gets just the library.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(GOLDNET_TOP_LEVEL ON)
else()
    set(GOLDNET_TOP_LEVEL OFF)
endif()

option(GOLDNET_BUILD_TESTS "Build goldnet tests" ${GOLDNET_TOP_LEVEL})
if(GOLDNET_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# The terminal demo needs only POSIX sockets.
option(GOLDNET_BUILD_EXAMPLES "Build the terminal demo" ${GOLDNET_TOP_LEVEL})
if(GOLDNET_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()
