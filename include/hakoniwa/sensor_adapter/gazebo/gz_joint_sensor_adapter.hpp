#pragma once

#include <memory>
#include <string>

#include "hakoniwa/sensor_adapter/joint_sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::gazebo {

/**
 * @brief Creates a joint adapter that reports the true joint position and velocity.
 *
 * gz-sensors has no joint sensor. Gazebo publishes joint states with the
 * JointStatePublisher system, which reports the true values without noise or
 * delay; this adapter does the same. setup.config_path is a JSON file
 * {"type": "joint", "update_rate": Hz}; update_rate is optional (omitted or 0:
 * every physics step). Returns nullptr and sets error on failure.
 */
std::unique_ptr<IJointSensorAdapter> create_gz_joint_sensor_adapter(
    const JointSensorAdapterSetup& setup, std::string& error);

}  // namespace hakoniwa::sensor_adapter::gazebo
