#include "hakoniwa/sensor_adapter/gazebo/gz_force_torque_sensor_adapter.hpp"

#include <gz/math/Vector3.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/sensors/ForceTorqueSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <sdf/ForceTorque.hh>

#include "gz_adapter_common.hpp"

namespace hakoniwa::sensor_adapter::gazebo {
namespace {

class GzForceTorqueSensorAdapter : public IForceTorqueSensorAdapter {
public:
    explicit GzForceTorqueSensorAdapter(const ForceTorqueAdapterSetup& setup)
        : setup_(setup), clock_(setup.physics_step_ns)
    {
    }

    bool initialize(std::string& error)
    {
        if (setup_.physics_step_ns == 0) {
            error = "physics_step_ns must be positive";
            return false;
        }
        if (!detail::load_sensor_sdf(setup_.config_path, sdf::SensorType::FORCE_TORQUE, "force_torque", sdf_, error)) {
            return false;
        }
        sdf_.SetTopic(detail::unique_topic(setup_));
        detail::seed_random(setup_);
        return create_sensor(error);
    }

    bool step(const ForceTorqueTruth& truth, ForceTorqueSample& out, std::string& error) override
    {
        if (!clock_.advance(truth.sim_time_ns, error)) {
            return false;
        }
        // gz-sensors takes the wrench the parent exerts on the child (the joint's transmitted
        // wrench) and, with the default measure_direction child_to_parent, reports its negative.
        // The interface's truth is the tool (child) on the robot (parent), so pass its negative.
        // The parent and child rotations stay identity: the host gives the wrench in the sensor axes.
        const auto& f = truth.wrench.force;
        const auto& t = truth.wrench.torque;
        sensor_->SetForce(gz::math::Vector3d(-f.x, -f.y, -f.z));
        sensor_->SetTorque(gz::math::Vector3d(-t.x, -t.y, -t.z));

        if (sensor_->gz::sensors::Sensor::Update(detail::to_gz_time(truth.sim_time_ns), false)) {
            const auto& w = sensor_->MeasuredWrench();
            last_.updated = true;
            last_.sample_time_ns = truth.sim_time_ns;
            last_.wrench.force = {w.force().x(), w.force().y(), w.force().z()};
            last_.wrench.torque = {w.torque().x(), w.torque().y(), w.torque().z()};
            last_.flags = 0;
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    // gz-sensors has no tare.
    bool tare(std::string& error) override
    {
        error = "gz-sensors' ForceTorqueSensor has no tare";
        return false;
    }

    void clear_tare() override {}

    bool reset(std::string& error) override
    {
        clock_.reset();
        last_ = {};
        return create_sensor(error);
    }

    ForceTorqueAdapterCapabilities capabilities() const override
    {
        ForceTorqueAdapterCapabilities caps{};
        // Per-axis Gaussian noise with a constant bias, a first-order Gauss-Markov
        // dynamic bias and quantization (GAUSSIAN_QUANTIZED). No delay, filter,
        // calibration matrix, clamp, overload or tare. One process-wide random stream.
        caps.noise = true;
        caps.bias = true;
        caps.drift = true;
        caps.resolution = true;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    bool create_sensor(std::string& error)
    {
        gz::sensors::SensorFactory factory;
        sensor_ = factory.CreateSensor<gz::sensors::ForceTorqueSensor>(sdf_);
        if (!sensor_) {
            error = "gz-sensors could not create the force-torque sensor";
            return false;
        }
        return true;
    }

    ForceTorqueAdapterSetup setup_;
    detail::StepClock clock_;
    sdf::Sensor sdf_{};
    std::unique_ptr<gz::sensors::ForceTorqueSensor> sensor_;
    ForceTorqueSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IForceTorqueSensorAdapter> create_gz_force_torque_sensor_adapter(
    const ForceTorqueAdapterSetup& setup, std::string& error)
{
    auto adapter = std::make_unique<GzForceTorqueSensorAdapter>(setup);
    if (!adapter->initialize(error)) {
        return nullptr;
    }
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::gazebo
