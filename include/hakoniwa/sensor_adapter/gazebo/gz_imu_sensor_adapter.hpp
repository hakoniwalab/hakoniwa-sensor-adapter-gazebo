#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/imu_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::gazebo {

/**
 * @brief Creates an IMU adapter backed by gz-sensors' ImuSensor.
 *
 * setup.config_path is an SDFormat file. The first <sensor type="imu"> in it
 * (inside a model and a link) gives the noise models and the update rate.
 * Its <pose> must be zero: the host applies the mounting pose.
 *
 * No Gazebo type appears in this header. Returns nullptr and sets error on failure.
 */
std::unique_ptr<IImuSensorAdapter> create_gz_imu_sensor_adapter(
    const ImuAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::gazebo
