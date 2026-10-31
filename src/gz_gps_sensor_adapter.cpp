#include "hakoniwa/sensor_adapter/gazebo/gz_gps_sensor_adapter.hpp"

#include <gz/math/Angle.hh>
#include <gz/math/Vector3.hh>
#include <gz/sensors/NavSatSensor.hh>
#include <gz/sensors/SensorFactory.hh>

#include "gz_adapter_common.hpp"

namespace hakoniwa::sensor_adapter::gazebo {
namespace {

// The interface velocity is NWU; NavSatSensor's is ENU (east, north, up).
gz::math::Vector3d nwu_to_enu(const Vector3& v)
{
    return {-v.y, v.x, v.z};
}

Vector3 enu_to_nwu(const gz::math::Vector3d& v)
{
    return {v.Y(), -v.X(), v.Z()};
}

class GzGpsSensorAdapter : public IGpsSensorAdapter {
public:
    explicit GzGpsSensorAdapter(const GpsAdapterSetup& setup) : setup_(setup), clock_(setup.physics_step_ns) {}

    bool initialize(std::string& error)
    {
        if (setup_.physics_step_ns == 0) {
            error = "physics_step_ns must be positive";
            return false;
        }
        if (!detail::load_sensor_sdf(setup_.config_path, sdf::SensorType::NAVSAT, "navsat", sdf_, error)) {
            return false;
        }
        sdf_.SetTopic(detail::unique_topic(setup_));
        return create_sensor(error);
    }

    bool step(const GpsTruth& truth, GpsSample& out, std::string& error) override
    {
        if (!clock_.advance(truth.sim_time_ns, error)) {
            return false;
        }
        // NavSatSensor applies its noise to the stored values during Update, so
        // the true values are set again on every step.
        sensor_->SetPosition(GZ_DTOR(truth.position.latitude_deg), GZ_DTOR(truth.position.longitude_deg),
                             truth.position.altitude_m);
        sensor_->SetVelocity(nwu_to_enu(truth.velocity_nwu));

        if (sensor_->gz::sensors::Sensor::Update(detail::to_gz_time(truth.sim_time_ns), false)) {
            GpsSample s{};
            s.updated = true;
            s.sample_time_ns = truth.sim_time_ns;
            s.position = {sensor_->Latitude().Degree(), sensor_->Longitude().Degree(), sensor_->Altitude()};
            s.velocity_nwu = enu_to_nwu(sensor_->Velocity());
            // NavSatSensor models a receiver that always has a 3D fix and reports
            // no course, accuracy or satellites.
            s.fix_type = 3;
            last_ = s;
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
        return create_sensor(error);
    }

    GpsAdapterCapabilities capabilities() const override
    {
        GpsAdapterCapabilities caps{};
        caps.position_noise = true;  // in degrees of latitude and longitude, metres of altitude
        caps.velocity_noise = true;
        caps.drift = true;           // dynamic bias of the noise models
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    bool create_sensor(std::string& error)
    {
        detail::seed_random(setup_);
        gz::sensors::SensorFactory factory;
        sensor_ = factory.CreateSensor<gz::sensors::NavSatSensor>(sdf_);
        if (!sensor_) {
            error = "gz-sensors could not create the navsat sensor";
            return false;
        }
        return true;
    }

    GpsAdapterSetup setup_;
    detail::StepClock clock_;
    sdf::Sensor sdf_{};
    std::unique_ptr<gz::sensors::NavSatSensor> sensor_;
    GpsSample last_{};
    std::vector<LogColumn> columns_;
};

}  // namespace

std::unique_ptr<IGpsSensorAdapter> create_gz_gps_sensor_adapter(const GpsAdapterSetup& setup, std::string& error)
{
    auto adapter = std::make_unique<GzGpsSensorAdapter>(setup);
    if (!adapter->initialize(error)) {
        return nullptr;
    }
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::gazebo
