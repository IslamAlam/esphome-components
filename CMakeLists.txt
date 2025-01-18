cmake_minimum_required(VERSION 3.10)
project(kalman_filter_component)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add component source files
set(COMPONENT_SRCS
    components/kalman_filter/kalman_filter.h
)

# Component library
add_library(kalman_filter INTERFACE)
target_include_directories(kalman_filter INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/components/kalman_filter
)

# Enable testing
enable_testing()

# Test executable
add_executable(test_kalman_filter tests/test_kalman_filter.cpp)
target_link_libraries(test_kalman_filter kalman_filter)

# Add test
add_test(NAME KalmanFilterTest COMMAND test_kalman_filter)