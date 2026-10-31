cmake_minimum_required(VERSION 3.16)

project(hakoniwa_sensor_adapter_gazebo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The public interface (hakoniwa-sensor-adapter), checked out next to this repository by default.
set(HAKONIWA_SENSOR_ADAPTER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-sensor-adapter"
    CACHE PATH "hakoniwa-sensor-adapter checkout")
add_subdirectory(${HAKONIWA_SENSOR_ADAPTER_ROOT} hakoniwa-sensor-adapter)

# Only the IMU component of gz-sensors (no rendering).
find_package(gz-sensors REQUIRED COMPONENTS imu)
find_package(sdformat REQUIRED)

add_library(hakoniwa_sensor_adapter_gazebo src/gz_imu_sensor_adapter.cpp)
target_include_directories(hakoniwa_sensor_adapter_gazebo
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hakoniwa_sensor_adapter_gazebo
    PUBLIC hakoniwa_sensor_adapter
    PRIVATE gz-sensors::imu sdformat::sdformat)

option(HAKONIWA_SENSOR_ADAPTER_GAZEBO_BUILD_TESTS "Build the Gazebo adapter tests" OFF)

if(HAKONIWA_SENSOR_ADAPTER_GAZEBO_BUILD_TESTS)
    enable_testing()
    add_executable(gz_imu_sensor_adapter_test test/gz_imu_sensor_adapter_test.cpp)
    target_link_libraries(gz_imu_sensor_adapter_test PRIVATE hakoniwa_sensor_adapter_gazebo)
    target_compile_definitions(gz_imu_sensor_adapter_test
        PRIVATE HAKO_GZ_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME gz_imu_sensor_adapter COMMAND gz_imu_sensor_adapter_test)
endif()
