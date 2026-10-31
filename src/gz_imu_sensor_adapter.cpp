#include "hakoniwa/sensor_adapter/gazebo/gz_imu_sensor_adapter.hpp"

#include <chrono>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/ImuSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <sdf/Imu.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sensor.hh>

namespace hakoniwa::sensor_adapter::gazebo {
namespace {

gz::math::Vector3d to_gz(const Vector3& v)
{
    return {v.x, v.y, v.z};
}

Vector3 from_gz(const gz::math::Vector3d& v)
{
    return {v.X(), v.Y(), v.Z()};
}

// Finds the first IMU sensor of the first model's links.
bool find_imu_sensor(const sdf::Root& root, sdf::Sensor& out, std::string& error)
{
    const sdf::Model* model = root.Model();
    if (model == nullptr) {
        error = "the SDFormat file has no <model>";
        return false;
    }
    for (std::uint64_t l = 0; l < model->LinkCount(); ++l) {
        const sdf::Link* link = model->LinkByIndex(l);
        for (std::uint64_t s = 0; s < link->SensorCount(); ++s) {
            const sdf::Sensor* sensor = link->SensorByIndex(s);
            if (sensor->Type() == sdf::SensorType::IMU) {
                out = *sensor;
                return true;
            }
        }
    }
    error = "the SDFormat file has no <sensor type=\"imu\">";
    return false;
}

class GzImuSensorAdapter : public IImuSensorAdapter {
public:
    bool initialize(const ImuAdapterSetup& setup, std::string& error)
    {
        setup_ = setup;
        if (setup.physics_step_ns == 0) {
            error = "physics_step_ns must be positive";
            return false;
        }
        sdf::Root root;
        const sdf::Errors errors = root.Load(setup.config_path);
        if (!errors.empty()) {
            error = "cannot load " + setup.config_path + ": " + errors.front().Message();
            return false;
        }
        if (!find_imu_sensor(root, sdf_, error)) {
            return false;
        }
        if (sdf_.RawPose() != gz::math::Pose3d::Zero) {
            error = "the IMU <pose> must be zero (the host applies the mounting pose)";
            return false;
        }
        if (setup_.has_random_seed) {
            gz::math::Rand::Seed(static_cast<unsigned int>(setup_.random_seed));
        }
        return create_sensor(error);
    }

    bool step(const ImuTruth& truth, ImuSample& out, std::string& error) override
    {
        if (truth.sim_time_ns != next_time_ns_) {
            error = "sim_time_ns must advance by physics_step_ns from 0";
            return false;
        }
        next_time_ns_ += setup_.physics_step_ns;

        // gz-sensors takes the kinematic acceleration and subtracts the gravity
        // rotated into the sensor axes (a - R^-1 g). The host gives the specific
        // force f, so a = f + R_ws g.
        const gz::math::Quaterniond q_ws(truth.orientation_world_to_sensor.w, truth.orientation_world_to_sensor.x,
                                         truth.orientation_world_to_sensor.y, truth.orientation_world_to_sensor.z);
        const gz::math::Vector3d g_world = to_gz(truth.gravity_world);
        const gz::math::Vector3d kinematic = to_gz(truth.specific_force) + q_ws.RotateVector(g_world);
        // The sensor's world pose rotates sensor axes to world axes.
        sensor_->SetWorldPose(gz::math::Pose3d(gz::math::Vector3d::Zero, q_ws.Inverse()));
        sensor_->SetGravity(g_world);
        sensor_->SetAngularVelocity(to_gz(truth.angular_velocity));
        sensor_->SetLinearAcceleration(kinematic);

        // The base Update checks the update rate and returns true only when it measured.
        const auto now = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::nanoseconds(truth.sim_time_ns));
        const bool measured = sensor_->gz::sensors::Sensor::Update(now, false);
        if (measured) {
            last_.gyro = {true, truth.sim_time_ns, from_gz(sensor_->AngularVelocity()), 0};
            last_.accel = {true, truth.sim_time_ns, from_gz(sensor_->LinearAcceleration()), 0};
        } else {
            // The getters return the last inputs, not the last measurement, so the
            // adapter holds the measured values itself.
            last_.gyro.updated = false;
            last_.accel.updated = false;
        }
        out = last_;
        return true;
    }

    bool reset(std::string& error) override
    {
        // gz-sensors has no reset: recreate the sensor. The process-wide random generator is
        // not seeded again (that would restart the sequence of every other gz sensor), so the
        // values after a reset are not the ones after creation.
        next_time_ns_ = 0;
        last_ = {};
        return create_sensor(error);
    }

    ImuAdapterCapabilities capabilities() const override
    {
        ImuAdapterCapabilities caps{};
        caps.noise = true;
        caps.bias = true;
        caps.drift = true;       // dynamic bias (first-order Gauss-Markov)
        caps.resolution = true;  // gaussian_quantized precision
        // One update rate for both channels, no delay, no filter, no range clamp.
        // gz-sensors draws from one process-wide random generator.
        return caps;
    }

    bool calibration(ImuCalibration&) const override
    {
        return false;
    }

    const std::vector<LogColumn>& log_columns() const override
    {
        // ImuSensor does not expose its noise models (bias in use) to the outside.
        return columns_;
    }

    void log_values(std::vector<double>& values) const override
    {
        values.clear();
    }

private:
    bool create_sensor(std::string& error)
    {
        gz::sensors::SensorFactory factory;
        sensor_ = factory.CreateSensor<gz::sensors::ImuSensor>(sdf_);
        if (!sensor_) {
            error = "gz-sensors could not create the IMU sensor";
            return false;
        }
        return true;
    }

    ImuAdapterSetup setup_{};
    sdf::Sensor sdf_{};
    std::unique_ptr<gz::sensors::ImuSensor> sensor_;
    std::uint64_t next_time_ns_{0};
    ImuSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IImuSensorAdapter> create_gz_imu_sensor_adapter(const ImuAdapterSetup& setup, std::string& error)
{
    auto adapter = std::make_unique<GzImuSensorAdapter>();
    if (!adapter->initialize(setup, error)) {
        return nullptr;
    }
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::gazebo
