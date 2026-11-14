#include "hakoniwa/sensor_adapter/gazebo/gz_mag_sensor_adapter.hpp"

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/MagnetometerSensor.hh>
#include <gz/sensors/SensorFactory.hh>

#include "gz_adapter_common.hpp"

namespace hakoniwa::sensor_adapter::gazebo {
namespace {

// gz-sensors works in tesla; the interface in nanotesla.
constexpr double kNanoTeslaPerTesla = 1e9;

class GzMagSensorAdapter : public IMagSensorAdapter {
public:
    explicit GzMagSensorAdapter(const MagAdapterSetup& setup) : setup_(setup), clock_(setup.physics_step_ns) {}

    bool initialize(std::string& error)
    {
        if (setup_.physics_step_ns == 0) {
            error = "physics_step_ns must be positive";
            return false;
        }
        if (!detail::load_sensor_sdf(setup_.config_path, sdf::SensorType::MAGNETOMETER, "magnetometer", sdf_, error)) {
            return false;
        }
        sdf_.SetTopic(detail::unique_topic(setup_));
        detail::seed_random(setup_);
        if (!create_sensor(error) ||
            !detail::period_from_update_rate(sensor_->UpdateRate(), setup_.physics_step_ns,
                                             configured_period_ns_, error)) {
            return false;
        }
        update_period_ns_ = configured_period_ns_;
        sensor_->SetUpdateRate(detail::update_rate_from_period(update_period_ns_));
        return true;
    }

    bool step(const MagTruth& truth, MagSample& out, std::string& error) override
    {
        if (!clock_.advance(truth.sim_time_ns, error)) {
            return false;
        }
        // gz-sensors rotates the world field into the sensor axes itself
        // (R^-1 B_world, with R the sensor-to-world rotation of the world pose).
        const auto& q = truth.orientation_world_to_sensor;
        const gz::math::Quaterniond q_ws(q.w, q.x, q.y, q.z);
        const auto& b = truth.magnetic_field_world;
        sensor_->SetWorldPose(gz::math::Pose3d(gz::math::Vector3d::Zero, q_ws.Inverse()));
        sensor_->SetWorldMagneticField(
            gz::math::Vector3d(b.x, b.y, b.z) / kNanoTeslaPerTesla);

        const bool sample_now = truth.sim_time_ns % update_period_ns_ == 0;
        if (sample_now && sensor_->gz::sensors::Sensor::Update(detail::to_gz_time(truth.sim_time_ns), true)) {
            const gz::math::Vector3d field = sensor_->MagneticField() * kNanoTeslaPerTesla;
            last_ = {true, truth.sim_time_ns, {field.X(), field.Y(), field.Z()}, 0};
        } else {
            last_.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string& error) override
    {
        clock_.reset();
        last_ = {};
        update_period_ns_ = configured_period_ns_;
        if (!create_sensor(error)) {
            return false;
        }
        sensor_->SetUpdateRate(detail::update_rate_from_period(update_period_ns_));
        return true;
    }

    bool update_period_ns(std::uint64_t& period_ns) const override
    {
        period_ns = update_period_ns_;
        return true;
    }

    bool set_update_period_ns(std::uint64_t period_ns, std::string& error) override
    {
        if (!detail::validate_update_period(period_ns, setup_.physics_step_ns, error)) {
            return false;
        }
        update_period_ns_ = period_ns;
        sensor_->SetUpdateRate(detail::update_rate_from_period(update_period_ns_));
        error.clear();
        return true;
    }

    MagAdapterCapabilities capabilities() const override
    {
        MagAdapterCapabilities caps{};
        caps.noise = true;
        // MagnetometerSensor applies its noise with a time step of 0, so the dynamic
        // bias never moves; precisions below 1e-6 (tesla) are ignored by gz-sensors.
        caps.drift = false;
        caps.resolution = false;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    bool create_sensor(std::string& error)
    {
        gz::sensors::SensorFactory factory;
        sensor_ = factory.CreateSensor<gz::sensors::MagnetometerSensor>(sdf_);
        if (!sensor_) {
            error = "gz-sensors could not create the magnetometer";
            return false;
        }
        return true;
    }

    MagAdapterSetup setup_;
    detail::StepClock clock_;
    sdf::Sensor sdf_{};
    std::unique_ptr<gz::sensors::MagnetometerSensor> sensor_;
    std::uint64_t configured_period_ns_{0};
    std::uint64_t update_period_ns_{0};
    MagSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IMagSensorAdapter> create_gz_mag_sensor_adapter(const MagAdapterSetup& setup, std::string& error)
{
    auto adapter = std::make_unique<GzMagSensorAdapter>(setup);
    if (!adapter->initialize(error)) {
        return nullptr;
    }
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::gazebo
