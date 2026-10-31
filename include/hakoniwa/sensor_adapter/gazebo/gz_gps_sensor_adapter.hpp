#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/gps_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::gazebo {

/**
 * @brief Creates a gps adapter backed by gz-sensors' NavSatSensor.
 *
 * setup.config_path is an SDFormat file; see the README for the sensor element
 * and the units. No Gazebo type appears in this header. Returns nullptr and
 * sets error on failure.
 */
std::unique_ptr<IGpsSensorAdapter> create_gz_gps_sensor_adapter(const GpsAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::gazebo
