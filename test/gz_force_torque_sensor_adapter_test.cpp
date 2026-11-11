// gz-sensors ForceTorqueSensor behind IForceTorqueSensorAdapter: direction, period,
// noise statistics, tare (absent), time rule and reset.
#include "hakoniwa/sensor_adapter/gazebo/gz_force_torque_sensor_adapter.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace sa = hakoniwa::sensor_adapter;
namespace gz = hakoniwa::sensor_adapter::gazebo;

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++failures;
    }
}

std::unique_ptr<sa::IForceTorqueSensorAdapter> make(const char* file)
{
    sa::ForceTorqueAdapterSetup setup;
    setup.config_path = std::string(HAKO_GZ_TEST_CONFIG_DIR) + "/" + file;
    setup.physics_step_ns = 1000000;  // 1 ms
    setup.has_random_seed = true;
    setup.random_seed = 3;
    std::string error;
    auto adapter = gz::create_gz_force_torque_sensor_adapter(setup, error);
    if (!adapter) std::fprintf(stderr, "create: %s\n", error.c_str());
    return adapter;
}

double stddev(const std::vector<double>& v)
{
    double m = 0;
    for (double x : v) m += x;
    m /= static_cast<double>(v.size());
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

}  // namespace

int main()
{
    std::string error;
    // Ideal: the interface's truth comes out unchanged (same direction), on 2 ms boundaries.
    auto ideal = make("ft-ideal.sdf");
    check(ideal != nullptr, "create the ideal sensor");
    if (!ideal) return EXIT_FAILURE;
    sa::ForceTorqueTruth truth;
    truth.wrench.force = {1.0, -2.0, -19.6};  // a hanging tool pulls the sensor down (z up)
    truth.wrench.torque = {0.1, 1.96, -0.3};
    sa::ForceTorqueSample s;
    for (std::uint64_t k = 0; k < 6; ++k) {
        truth.sim_time_ns = k * 1000000;
        check(ideal->step(truth, s, error), "ideal step");
        check(s.updated == (k % 2 == 0), "measures every 2 ms");
        check(std::fabs(s.wrench.force.z + 19.6) < 1e-9, "same direction as the interface truth (Fz)");
        check(std::fabs(s.wrench.torque.y - 1.96) < 1e-9, "same direction as the interface truth (Ty)");
        check(std::fabs(s.wrench.force.x - 1.0) < 1e-9, "Fx unchanged");
    }
    check(!ideal->tare(error), "gz-sensors has no tare");
    check(!ideal->capabilities().tare, "capabilities: no tare");
    truth.sim_time_ns = 9 * 1000000;
    check(!ideal->step(truth, s, error), "skipped time is an error");
    check(ideal->reset(error), "reset");
    truth.sim_time_ns = 0;
    check(ideal->step(truth, s, error) && s.updated, "step from 0 after reset");

    // Noise: per-axis sigma as configured (0.1 N on Fx, 0.02 N m on Tz), 20,000 samples.
    auto noisy = make("ft-noise.sdf");
    check(noisy != nullptr, "create the noisy sensor");
    if (!noisy) return EXIT_FAILURE;
    truth.wrench = {};
    std::vector<double> fx, tz;
    for (std::uint64_t k = 0; k < 40000; ++k) {
        truth.sim_time_ns = k * 1000000;
        if (!noisy->step(truth, s, error)) {
            check(false, "noisy step");
            break;
        }
        if (s.updated) {
            fx.push_back(s.wrench.force.x);
            tz.push_back(s.wrench.torque.z);
        }
    }
    check(fx.size() == 20000, "20,000 samples at 500 Hz over 40 s");
    check(std::fabs(stddev(fx) - 0.1) < 0.003, "Fx noise sigma 0.1 N within 3 %");
    check(std::fabs(stddev(tz) - 0.02) < 0.0006, "Tz noise sigma 0.02 N m within 3 %");

    if (failures == 0) std::printf("gz_force_torque_sensor_adapter: OK\n");
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
