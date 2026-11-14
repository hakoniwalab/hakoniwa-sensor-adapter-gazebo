#pragma once

// Helpers shared by the Gazebo adapters (not part of the public interface).

#ifdef _WIN32
#include <process.h>
#else
#include <unistd.h>
#endif

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>

#include "hakoniwa/sensor_adapter/sensor_adapter.hpp"

#include "gz_definition_json.hpp"

namespace hakoniwa::sensor_adapter::gazebo::detail {

// Loads the definition (an SDFormat file, or its JSON form from tools/sdf_json.py)
// and finds the first sensor of the type in the first model's links. Its <pose>
// must be zero: the host applies the mounting pose.
inline bool load_sensor_sdf(const std::string& path, sdf::SensorType type, const char* type_name,
                            sdf::Sensor& out, std::string& error)
{
    sdf::Root root;
    sdf::Errors errors;
    if (is_json_definition(path)) {
        std::string sdf_text;
        if (!sdf_from_json_definition(path, type_name, sdf_text, error)) {
            return false;
        }
        errors = root.LoadSdfString(sdf_text);
    } else {
        errors = root.Load(path);
    }
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

// Match a configured gz-sensors update rate to the nearest whole number of
// physics steps (round half up, with one step as the minimum).
inline bool period_from_update_rate(double update_rate_hz, std::uint64_t physics_step_ns,
                                    std::uint64_t& period_ns, std::string& error)
{
    if (physics_step_ns == 0) {
        error = "physics_step_ns must be positive";
        return false;
    }
    if (!std::isfinite(update_rate_hz) || update_rate_hz < 0.0) {
        error = "the sensor update rate must be finite and non-negative";
        return false;
    }
    if (update_rate_hz == 0.0) {
        period_ns = physics_step_ns;
        return true;
    }

    const long double steps = (1.0e9L / static_cast<long double>(update_rate_hz)) /
                              static_cast<long double>(physics_step_ns);
    const long double rounded_steps = std::floor(steps + 0.5L);
    const long double used_steps = rounded_steps < 1.0L ? 1.0L : rounded_steps;
    const std::uint64_t max_steps = std::numeric_limits<std::uint64_t>::max() / physics_step_ns;
    if (used_steps > static_cast<long double>(max_steps)) {
        error = "the sensor update period is too large";
        return false;
    }
    period_ns = static_cast<std::uint64_t>(used_steps) * physics_step_ns;
    return true;
}

inline bool validate_update_period(std::uint64_t period_ns, std::uint64_t physics_step_ns,
                                   std::string& error)
{
    if (period_ns == 0) {
        error = "the update period must be greater than zero";
        return false;
    }
    if (period_ns < physics_step_ns) {
        error = "the update period must not be shorter than physics_step_ns";
        return false;
    }
    if (period_ns % physics_step_ns != 0) {
        error = "the update period must be a whole number of physics steps";
        return false;
    }
    return true;
}

inline double update_rate_from_period(std::uint64_t period_ns)
{
    return 1.0e9 / static_cast<double>(period_ns);
}

}  // namespace hakoniwa::sensor_adapter::gazebo::detail
