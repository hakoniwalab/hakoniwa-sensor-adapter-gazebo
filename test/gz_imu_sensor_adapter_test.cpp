// Tests of the gz-sensors IMU adapter through the IImuSensorAdapter interface.
#include "hakoniwa/sensor_adapter/gazebo/gz_imu_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

bool near(const sa::Vector3& a, const sa::Vector3& b, double tolerance)
{
    return std::fabs(a.x - b.x) <= tolerance && std::fabs(a.y - b.y) <= tolerance &&
           std::fabs(a.z - b.z) <= tolerance;
}

sa::ImuAdapterSetup setup_for(const char* file)
{
    sa::ImuAdapterSetup setup{};
    setup.config_path = std::string(HAKO_GZ_TEST_CONFIG_DIR) + "/" + file;
    setup.physics_step_ns = 1000000;  // 1 ms
    setup.has_random_seed = true;
    setup.random_seed = 9201;
    return setup;
}

// Rotates v by the unit quaternion q (v' = q v q*).
sa::Vector3 rotate(const sa::Quaternion& q, const sa::Vector3& v)
{
    // t = 2 (q.xyz x v); v' = v + w t + q.xyz x t
    const double tx = 2.0 * (q.y * v.z - q.z * v.y);
    const double ty = 2.0 * (q.z * v.x - q.x * v.z);
    const double tz = 2.0 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

// A tilted, rotating truth in the Hakoniwa frames (world NWU, sensor FLU): a hover
// (no kinematic acceleration) seen by a sensor rolled 30 degrees.
sa::ImuTruth truth_at(int k)
{
    sa::ImuTruth truth{};
    truth.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
    truth.angular_velocity = {0.2, -0.1, 0.05 * k};
    const double h = 15.0 * 3.14159265358979323846 / 180.0;
    truth.orientation_world_to_sensor = {std::cos(h), std::sin(h), 0.0, 0.0};
    truth.gravity_world = {0.0, 0.0, -9.80665};
    // f = a - g with a = 0: +g up in the world, rotated into the sensor axes.
    truth.specific_force = rotate(truth.orientation_world_to_sensor, {0.0, 0.0, 9.80665});
    return truth;
}

std::vector<sa::ImuSample> run(sa::IImuSensorAdapter& imu, int steps)
{
    std::vector<sa::ImuSample> samples;
    std::string error;
    for (int k = 0; k < steps; ++k) {
        sa::ImuSample sample{};
        check(imu.step(truth_at(k), sample, error), "step succeeds");
        samples.push_back(sample);
    }
    return samples;
}

void ideal_sensor_passes_the_truth_at_its_rate()
{
    std::string error;
    auto imu = sa::gazebo::create_gz_imu_sensor_adapter(setup_for("imu-ideal.sdf"), error);
    check(imu != nullptr, "ideal adapter is created");
    if (!imu) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    const auto samples = run(*imu, 9);
    for (int k = 0; k < 9; ++k) {
        const int measured_at = k - k % 4;  // 250 Hz = every 4 ms from time 0
        const sa::ImuTruth truth = truth_at(measured_at);
        check(samples[k].gyro.updated == (k % 4 == 0), "measures at 0, 4, 8 ms");
        check(samples[k].gyro.sample_time_ns == truth.sim_time_ns, "sample time is the measurement time");
        check(near(samples[k].gyro.value, truth.angular_velocity, 1e-12), "gyro equals the truth, held between");
        // f + R g - R g: only rounding remains.
        check(near(samples[k].accel.value, truth.specific_force, 1e-12), "accel equals the specific force");
    }
}

void same_seed_gives_the_same_values_and_reset_repeats_them()
{
    std::string error;
    auto first = sa::gazebo::create_gz_imu_sensor_adapter(setup_for("imu-noise.sdf"), error);
    check(first != nullptr, "noisy adapter is created");
    if (!first) {
        return;
    }
    const auto a = run(*first, 400);
    check(first->reset(error), "reset succeeds");
    const auto b = run(*first, 400);
    auto second = sa::gazebo::create_gz_imu_sensor_adapter(setup_for("imu-noise.sdf"), error);
    const auto c = run(*second, 400);

    bool same_after_reset = true;
    bool same_new_instance = true;
    double sum = 0.0, sum2 = 0.0;
    int n = 0;
    for (std::size_t k = 0; k < a.size(); ++k) {
        same_after_reset &= near(a[k].gyro.value, b[k].gyro.value, 0.0);
        same_new_instance &= near(a[k].gyro.value, c[k].gyro.value, 0.0);
        if (a[k].accel.updated) {
            const double e = a[k].accel.value.x - truth_at(static_cast<int>(k)).specific_force.x;
            sum += e;
            sum2 += e * e;
            ++n;
        }
    }
    check(same_after_reset, "reset repeats the same values with the same seed");
    check(same_new_instance, "a new instance with the same seed gives the same values");
    const double sd = std::sqrt(sum2 / n - (sum / n) * (sum / n));
    std::printf("accel x noise: n=%d mean=%.4f sd=%.4f (configured 0.03)\n", n, sum / n, sd);
    check(sd > 0.02 && sd < 0.04, "accel noise has the configured standard deviation");
}

void a_mount_pose_in_the_sdf_is_rejected()
{
    std::string error;
    auto imu = sa::gazebo::create_gz_imu_sensor_adapter(setup_for("imu-with-pose.sdf"), error);
    check(imu == nullptr && !error.empty(), "non-zero <pose> is rejected");
}

void a_time_jump_is_an_error()
{
    std::string error;
    auto imu = sa::gazebo::create_gz_imu_sensor_adapter(setup_for("imu-ideal.sdf"), error);
    sa::ImuSample sample{};
    sa::ImuTruth truth = truth_at(0);
    truth.sim_time_ns = 5;
    check(imu && !imu->step(truth, sample, error) && !error.empty(), "a time jump is an error");
}

void capabilities_declare_what_gazebo_lacks()
{
    std::string error;
    auto imu = sa::gazebo::create_gz_imu_sensor_adapter(setup_for("imu-ideal.sdf"), error);
    const auto caps = imu->capabilities();
    check(caps.noise && caps.bias && caps.drift && caps.resolution, "noise, bias, drift, resolution");
    check(!caps.delay && !caps.filter && !caps.range_clamp && !caps.separate_gyro_accel_timing,
          "no delay, filter, range clamp, separate timing");
    sa::ImuCalibration calibration{};
    check(!caps.calibration_values && !imu->calibration(calibration), "no calibration values");
    check(imu->log_columns().empty(), "no log columns");
}

}  // namespace

int main()
{
    ideal_sensor_passes_the_truth_at_its_rate();
    same_seed_gives_the_same_values_and_reset_repeats_them();
    a_mount_pose_in_the_sdf_is_rejected();
    a_time_jump_is_an_error();
    capabilities_declare_what_gazebo_lacks();
    if (failures == 0) {
        std::printf("gz_imu_sensor_adapter_test: OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
