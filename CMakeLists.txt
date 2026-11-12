cmake_minimum_required(VERSION 3.16)

project(hakoniwa_sensor_adapter_gazebo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The public interface (hakoniwa-sensor-adapter), checked out next to this repository by default.
set(HAKONIWA_SENSOR_ADAPTER_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../hakoniwa-sensor-adapter"
    CACHE PATH "hakoniwa-sensor-adapter checkout")
add_subdirectory(${HAKONIWA_SENSOR_ADAPTER_ROOT} hakoniwa-sensor-adapter)

# The non-rendering components of gz-sensors used by the adapters.
find_package(gz-sensors REQUIRED COMPONENTS imu magnetometer air_pressure navsat force_torque)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)
find_package(sdformat REQUIRED)
# JSON definition files (the form a JSON-only host reads; tools/sdf_json.py).
find_package(nlohmann_json 3.11 REQUIRED)

add_library(hakoniwa_sensor_adapter_gazebo
    src/gz_imu_sensor_adapter.cpp
    src/gz_mag_sensor_adapter.cpp
    src/gz_baro_sensor_adapter.cpp
    src/gz_gps_sensor_adapter.cpp
    src/gz_force_torque_sensor_adapter.cpp
    src/gz_joint_sensor_adapter.cpp
    src/gazebo_implementation.cpp)
target_include_directories(hakoniwa_sensor_adapter_gazebo
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(hakoniwa_sensor_adapter_gazebo
    PUBLIC hakoniwa_sensor_adapter
    PRIVATE
        gz-sensors::imu gz-sensors::magnetometer gz-sensors::air_pressure gz-sensors::navsat gz-sensors::force_torque
        gz-transport::gz-transport gz-msgs::gz-msgs sdformat::sdformat nlohmann_json::nlohmann_json)

option(HAKONIWA_SENSOR_ADAPTER_GAZEBO_BUILD_TESTS "Build the Gazebo adapter tests" OFF)

if(HAKONIWA_SENSOR_ADAPTER_GAZEBO_BUILD_TESTS)
    enable_testing()
    add_executable(gz_imu_sensor_adapter_test test/gz_imu_sensor_adapter_test.cpp)
    target_link_libraries(gz_imu_sensor_adapter_test PRIVATE hakoniwa_sensor_adapter_gazebo)
    target_compile_definitions(gz_imu_sensor_adapter_test
        PRIVATE HAKO_GZ_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME gz_imu_sensor_adapter COMMAND gz_imu_sensor_adapter_test)

    add_executable(gz_mag_baro_gps_sensor_adapter_test test/gz_mag_baro_gps_sensor_adapter_test.cpp)
    target_link_libraries(gz_mag_baro_gps_sensor_adapter_test PRIVATE hakoniwa_sensor_adapter_gazebo)
    target_compile_definitions(gz_mag_baro_gps_sensor_adapter_test
        PRIVATE HAKO_GZ_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME gz_mag_baro_gps_sensor_adapter COMMAND gz_mag_baro_gps_sensor_adapter_test)

    add_executable(gz_force_torque_sensor_adapter_test test/gz_force_torque_sensor_adapter_test.cpp)
    target_link_libraries(gz_force_torque_sensor_adapter_test PRIVATE hakoniwa_sensor_adapter_gazebo)
    target_compile_definitions(gz_force_torque_sensor_adapter_test
        PRIVATE HAKO_GZ_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
    add_test(NAME gz_force_torque_sensor_adapter COMMAND gz_force_torque_sensor_adapter_test)

    add_executable(gazebo_implementation_test test/gazebo_implementation_test.cpp)
    target_link_libraries(gazebo_implementation_test PRIVATE hakoniwa_sensor_adapter_gazebo)
    target_compile_definitions(gazebo_implementation_test
        PRIVATE HAKO_GZ_TEST_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
                HAKO_GZ_TEST_BINARY_DIR="${CMAKE_CURRENT_BINARY_DIR}")
    add_test(NAME gazebo_implementation COMMAND gazebo_implementation_test)
endif()
