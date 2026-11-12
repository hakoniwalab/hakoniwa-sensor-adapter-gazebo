#include "hakoniwa/sensor_adapter/gazebo/gz_joint_sensor_adapter.hpp"

#include <cmath>
#include <fstream>

#include <nlohmann/json.hpp>

#include "gz_adapter_common.hpp"

namespace hakoniwa::sensor_adapter::gazebo {
namespace {

class GzJointSensorAdapter : public IJointSensorAdapter {
public:
    explicit GzJointSensorAdapter(const JointSensorAdapterSetup& setup)
        : setup_(setup), clock_(setup.physics_step_ns)
    {
    }

    bool initialize(std::string& error)
    {
        if (setup_.physics_step_ns == 0) {
            error = "physics_step_ns must be positive";
            return false;
        }
        return load_definition(error);
    }

    bool step(const JointSensorTruth& truth, JointSensorSample& out, std::string& error) override
    {
        if (!clock_.advance(truth.sim_time_ns, error)) {
            return false;
        }
        // Measure at 0 and then once the period has passed (every step without a period).
        if (truth.sim_time_ns >= next_sample_ns_) {
            last_.updated = true;
            last_.sample_time_ns = truth.sim_time_ns;
            last_.position = truth.position;
            last_.velocity = truth.velocity;
            next_sample_ns_ = truth.sim_time_ns + period_ns_;
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string&) override
    {
        clock_.reset();
        last_ = {};
        next_sample_ns_ = 0;
        return true;
    }

    JointSensorAdapterCapabilities capabilities() const override
    {
        // True values only, like Gazebo's JointStatePublisher. No random numbers are drawn.
        JointSensorAdapterCapabilities caps{};
        caps.deterministic_with_seed = true;
        caps.thread_safe_across_instances = true;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    bool load_definition(std::string& error)
    {
        std::ifstream file(setup_.config_path);
        if (!file) {
            error = "cannot open " + setup_.config_path;
            return false;
        }
        nlohmann::json root;
        try {
            root = nlohmann::json::parse(file);
        } catch (const std::exception& e) {
            error = setup_.config_path + ": " + e.what();
            return false;
        }
        if (!root.is_object()) {
            error = setup_.config_path + ": the definition must be a JSON object";
            return false;
        }
        for (const auto& [key, value] : root.items()) {
            if (key != "type" && key != "$schema" && key != "update_rate") {
                error = setup_.config_path + ": unknown key \"" + key + "\" (the gazebo joint sensor has update_rate only)";
                return false;
            }
        }
        if (root.contains("type") && root.at("type") != "joint") {
            error = setup_.config_path + ": type must be \"joint\"";
            return false;
        }
        period_ns_ = 0;
        if (root.contains("update_rate")) {
            const auto& rate = root.at("update_rate");
            if (!rate.is_number() || !std::isfinite(rate.get<double>()) || rate.get<double>() < 0) {
                error = setup_.config_path + ": update_rate must be a number >= 0 [Hz]";
                return false;
            }
            const double hz = rate.get<double>();
            if (hz > 0) {
                period_ns_ = static_cast<std::uint64_t>(std::llround(1e9 / hz));
            }
        }
        return true;
    }

    JointSensorAdapterSetup setup_;
    detail::StepClock clock_;
    std::uint64_t period_ns_{0};
    std::uint64_t next_sample_ns_{0};
    JointSensorSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IJointSensorAdapter> create_gz_joint_sensor_adapter(const JointSensorAdapterSetup& setup,
                                                                    std::string& error)
{
    auto adapter = std::make_unique<GzJointSensorAdapter>(setup);
    if (!adapter->initialize(error)) {
        return nullptr;
    }
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::gazebo
