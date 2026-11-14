#include "hakoniwa/sensor_adapter/gazebo/gz_baro_sensor_adapter.hpp"

#include <chrono>
#include <condition_variable>
#include <mutex>

#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/sensors/AirPressureSensor.hh>
#include <gz/sensors/SensorFactory.hh>
#include <gz/transport/Node.hh>

#include "gz_adapter_common.hpp"

namespace hakoniwa::sensor_adapter::gazebo {
namespace {

// How long to wait for the sensor's own message after a measurement.
constexpr std::chrono::seconds kMessageTimeout{2};

/*
 * AirPressureSensor has no getter for the pressure it computed: the value only
 * leaves the sensor in its gz-transport message, and gz-transport delivers
 * messages to subscribers of the same process on its own thread. The adapter
 * subscribes to the sensor's topic and, after a measurement, waits for the
 * message with that measurement's time stamp.
 */
class GzBaroSensorAdapter : public IBaroSensorAdapter {
public:
    explicit GzBaroSensorAdapter(const BaroAdapterSetup& setup) : setup_(setup), clock_(setup.physics_step_ns) {}

    // Nothing was subscribed when creation failed before the topic was set.
    ~GzBaroSensorAdapter() override
    {
        if (!topic_.empty()) {
            node_.Unsubscribe(topic_);
        }
    }

    bool initialize(std::string& error)
    {
        if (setup_.physics_step_ns == 0) {
            error = "physics_step_ns must be positive";
            return false;
        }
        if (!detail::load_sensor_sdf(setup_.config_path, sdf::SensorType::AIR_PRESSURE, "air_pressure", sdf_, error)) {
            return false;
        }
        topic_ = detail::unique_topic(setup_);
        sdf_.SetTopic(topic_);
        if (!node_.Subscribe(topic_, &GzBaroSensorAdapter::on_message, this)) {
            error = "cannot subscribe to " + topic_;
            return false;
        }
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

    bool step(const BaroTruth& truth, BaroSample& out, std::string& error) override
    {
        if (!clock_.advance(truth.sim_time_ns, error)) {
            return false;
        }
        // AirPressureSensor computes the height as reference altitude + pose z.
        sensor_->SetPose(gz::math::Pose3d(0.0, 0.0, truth.altitude_m, 0.0, 0.0, 0.0));

        const auto now = detail::to_gz_time(truth.sim_time_ns);
        const bool sample_now = truth.sim_time_ns % update_period_ns_ == 0;
        if (sample_now && sensor_->gz::sensors::Sensor::Update(now, true)) {
            double pressure = 0.0;
            if (!wait_for_message(now, pressure, error)) {
                return false;
            }
            last_ = {true, truth.sim_time_ns, pressure, 0.0, 0.0, 0};
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

    BaroAdapterCapabilities capabilities() const override
    {
        BaroAdapterCapabilities caps{};
        // Fixed standard sea level (101325 Pa, 288.15 K); no temperature or
        // pressure altitude in the output.
        caps.noise = true;
        caps.offset = true;  // the sign of bias_mean is flipped with probability 1/2
        // AirPressureSensor applies its noise with a time step of 0: the dynamic bias never moves.
        caps.drift = false;
        caps.resolution = true;
        return caps;
    }

    const std::vector<LogColumn>& log_columns() const override { return columns_; }
    void log_values(std::vector<double>& values) const override { values.clear(); }

private:
    bool create_sensor(std::string& error)
    {
        gz::sensors::SensorFactory factory;
        sensor_ = factory.CreateSensor<gz::sensors::AirPressureSensor>(sdf_);
        if (!sensor_) {
            error = "gz-sensors could not create the air pressure sensor";
            return false;
        }
        sensor_->SetReferenceAltitude(0.0);
        return true;
    }

    void on_message(const gz::msgs::FluidPressure& msg)
    {
        const auto stamp = gz::msgs::Convert(msg.header().stamp());
        std::lock_guard<std::mutex> lock(mutex_);
        received_stamp_ = stamp;
        received_pressure_ = msg.pressure();
        received_ = true;
        cv_.notify_all();
    }

    bool wait_for_message(std::chrono::steady_clock::duration stamp, double& pressure, std::string& error)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool got = cv_.wait_for(lock, kMessageTimeout, [&] { return received_ && received_stamp_ == stamp; });
        if (!got) {
            error = "the air pressure message did not arrive on " + topic_;
            return false;
        }
        pressure = received_pressure_;
        return true;
    }

    BaroAdapterSetup setup_;
    detail::StepClock clock_;
    sdf::Sensor sdf_{};
    std::string topic_;
    gz::transport::Node node_;
    std::unique_ptr<gz::sensors::AirPressureSensor> sensor_;
    std::uint64_t configured_period_ns_{0};
    std::uint64_t update_period_ns_{0};
    BaroSample last_{};
    std::vector<LogColumn> columns_;

    std::mutex mutex_;
    std::condition_variable cv_;
    bool received_{false};
    std::chrono::steady_clock::duration received_stamp_{};
    double received_pressure_{0.0};
};

}  // namespace

std::unique_ptr<IBaroSensorAdapter> create_gz_baro_sensor_adapter(const BaroAdapterSetup& setup, std::string& error)
{
    auto adapter = std::make_unique<GzBaroSensorAdapter>(setup);
    if (!adapter->initialize(error)) {
        return nullptr;
    }
    return adapter;
}

}  // namespace hakoniwa::sensor_adapter::gazebo
