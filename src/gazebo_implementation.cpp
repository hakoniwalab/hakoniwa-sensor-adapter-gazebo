// The "gazebo" implementation descriptor: hosts that choose sensor
// implementations by name (hakoniwa-sensor-adapter
// sensor_adapter_implementation.hpp) get the Gazebo adapters through it.
#include "hakoniwa/sensor_adapter/gazebo/gz_baro_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_force_torque_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_gps_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_imu_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_joint_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_mag_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

HAKO_SENSOR_ADAPTER_DEFINE_IMPLEMENTATION(gazebo)
{
    namespace gazebo = hakoniwa::sensor_adapter::gazebo;
    static const hakoniwa::sensor_adapter::SensorAdapterImplementation implementation = [] {
        hakoniwa::sensor_adapter::SensorAdapterImplementation value;
        value.name = "gazebo";
        value.create_imu = &gazebo::create_gz_imu_sensor_adapter;
        value.create_mag = &gazebo::create_gz_mag_sensor_adapter;
        value.create_baro = &gazebo::create_gz_baro_sensor_adapter;
        value.create_gps = &gazebo::create_gz_gps_sensor_adapter;
        value.create_force_torque = &gazebo::create_gz_force_torque_sensor_adapter;
        value.create_joint = &gazebo::create_gz_joint_sensor_adapter;
        return value;
    }();
    return &implementation;
}
