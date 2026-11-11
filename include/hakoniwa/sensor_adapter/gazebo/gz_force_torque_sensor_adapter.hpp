#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/force_torque_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::gazebo {

/**
 * @brief Creates a force-torque adapter backed by gz-sensors' ForceTorqueSensor.
 *
 * setup.config_path is an SDFormat file with a <sensor type="force_torque">; see the
 * README for the frame, the direction and the noise. No Gazebo type appears in this
 * header. Returns nullptr and sets error on failure.
 */
std::unique_ptr<IForceTorqueSensorAdapter> create_gz_force_torque_sensor_adapter(
    const ForceTorqueAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::gazebo
