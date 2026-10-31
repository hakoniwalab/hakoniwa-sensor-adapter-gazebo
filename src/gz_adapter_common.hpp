#pragma once

// Helpers shared by the Gazebo adapters (not part of the public interface).

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

namespace hakoniwa::sensor_adapter::gazebo::detail {

// Loads the SDFormat file and finds the first sensor of the type in the first
// model's links. Its <pose> must be zero: the host applies the mounting pose.
inline bool load_sensor_sdf(const std::string& path, sdf::SensorType type, const char* type_name,
                            sdf::Sensor& out, std::string& error)
{
    sdf::Root root;
    const sdf::Errors errors = root.Load(path);
    if (!errors.empty()) {
        error = "cannot load " + path + ": " + errors.front().Message();
        return false;
    }
    const sdf::Model* model = root.Model();
    if (model == nullptr) {
        error = "the SDFormat file has no <model>";
        return false;
    }
    for (std::uint64_t l = 0; l < model->LinkCount(); ++l) {
        const sdf::Link* link = model->LinkByIndex(l);
        for (std::uint64_t s = 0; s < link->SensorCount(); ++s) {
            const sdf::Sensor* sensor = link->SensorByIndex(s);
            if (sensor->Type() == type) {
                if (sensor->RawPose() != gz::math::Pose3d::Zero) {
                    error = std::string("the ") + type_name + " <pose> must be zero (the host applies the mounting pose)";
                    return false;
                }
                out = *sensor;
                return true;
            }
        }
    }
    error = std::string("the SDFormat file has no <sensor type=\"") + type_name + "\">";
    return false;
}

// Seeds gz-sensors' process-wide random generator when the host gave a seed. Called when an
// adapter is created, not on reset: a reset must not restart the sequence other sensors draw from.
inline void seed_random(const SensorAdapterSetup& setup)
{
    if (setup.has_random_seed) {
        gz::math::Rand::Seed(static_cast<unsigned int>(setup.random_seed));
    }
}

inline long process_id()
{
#ifdef _WIN32
    return static_cast<long>(_getpid());
#else
    return static_cast<long>(::getpid());
#endif
}

// A topic of its own for each instance, so that instances never share a publisher, within
// the process or with another process on the same PC (gz-transport finds topics across
// processes).
inline std::string unique_topic(const SensorAdapterSetup& setup)
{
    static std::atomic<std::uint64_t> counter{0};
    return "/hakoniwa/sensor_adapter/" + std::to_string(process_id()) + "/" + std::to_string(setup.vehicle_index) +
           "/" + setup.sensor_name + "/" + std::to_string(counter++);
}

// The interface's time rule: 0, dt, 2 dt, ...
class StepClock {
public:
    explicit StepClock(std::uint64_t step_ns) : step_ns_(step_ns) {}

    bool advance(std::uint64_t sim_time_ns, std::string& error)
    {
        if (sim_time_ns != next_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return false;
        }
        next_ns_ += step_ns_;
        return true;
    }
    void reset() { next_ns_ = 0; }

private:
    std::uint64_t step_ns_;
    std::uint64_t next_ns_{0};
};

inline std::chrono::steady_clock::duration to_gz_time(std::uint64_t sim_time_ns)
{
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::nanoseconds(sim_time_ns));
}

}  // namespace hakoniwa::sensor_adapter::gazebo::detail
