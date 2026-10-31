#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/baro_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::gazebo {

/**
 * @brief Creates a baro adapter backed by gz-sensors' AirPressureSensor.
 *
 * setup.config_path is an SDFormat file; see the README for the sensor element
 * and the units. No Gazebo type appears in this header. Returns nullptr and
 * sets error on failure.
 */
std::unique_ptr<IBaroSensorAdapter> create_gz_baro_sensor_adapter(const BaroAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::gazebo
