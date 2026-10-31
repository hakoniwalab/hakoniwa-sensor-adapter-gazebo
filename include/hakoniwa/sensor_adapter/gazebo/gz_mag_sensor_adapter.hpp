#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/mag_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::gazebo {

/**
 * @brief Creates a mag adapter backed by gz-sensors' MagnetometerSensor.
 *
 * setup.config_path is an SDFormat file; see the README for the sensor element
 * and the units. No Gazebo type appears in this header. Returns nullptr and
 * sets error on failure.
 */
std::unique_ptr<IMagSensorAdapter> create_gz_mag_sensor_adapter(const MagAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::gazebo
