// Tests of the gz-sensors magnetometer, air pressure and NavSat adapters through
// the interfaces. Frames: world NWU, sensor FLU.
#include "hakoniwa/sensor_adapter/gazebo/gz_baro_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_gps_sensor_adapter.hpp"
#include "hakoniwa/sensor_adapter/gazebo/gz_mag_sensor_adapter.hpp"

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

template <typename Setup>
Setup setup_for(const char* file)
{
    Setup setup{};
    setup.config_path = std::string(HAKO_GZ_TEST_CONFIG_DIR) + "/" + file;
    setup.physics_step_ns = 1000000;  // 1 ms
    setup.has_random_seed = true;
    setup.random_seed = 9201;
    return setup;
}

sa::Vector3 rotate(const sa::Quaternion& q, const sa::Vector3& v)
{
    const double tx = 2.0 * (q.y * v.z - q.z * v.y);
    const double ty = 2.0 * (q.z * v.x - q.x * v.z);
    const double tz = 2.0 * (q.x * v.y - q.y * v.x);
    return {v.x + q.w * tx + (q.y * tz - q.z * ty), v.y + q.w * ty + (q.z * tx - q.x * tz),
            v.z + q.w * tz + (q.x * ty - q.y * tx)};
}

double stddev(const std::vector<double>& v)
{
    double s = 0.0, s2 = 0.0;
    for (double x : v) {
        s += x;
        s2 += x * x;
    }
    const double n = static_cast<double>(v.size());
    return std::sqrt(s2 / n - (s / n) * (s / n));
}

// --- magnetometer ---

// Earth field at the default location (intensity 53045 nT, declination 15 deg east,
// inclination 69 deg down), in NWU, seen by a sensor yawed 40 degrees.
sa::MagTruth mag_truth(int k)
{
    const double d2r = 3.14159265358979323846 / 180.0;
    const double h = 53045.1 * std::cos(68.984 * d2r);
    sa::MagTruth t{};
    t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
    t.magnetic_field_world = {h * std::cos(15.011 * d2r), -h * std::sin(15.011 * d2r),
                              -53045.1 * std::sin(68.984 * d2r)};
    const double half = 0.5 * 40.0 * d2r;
    t.orientation_world_to_sensor = {std::cos(half), 0.0, 0.0, std::sin(half)};
    t.magnetic_field = rotate(t.orientation_world_to_sensor, t.magnetic_field_world);
    return t;
}

void magnetometer()
{
    std::string error;
    auto ideal = sa::gazebo::create_gz_mag_sensor_adapter(setup_for<sa::MagAdapterSetup>("mag-ideal.sdf"), error);
    check(ideal != nullptr, "magnetometer is created");
    if (!ideal) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    for (int k = 0; k < 25; ++k) {
        sa::MagSample s{};
        check(ideal->step(mag_truth(k), s, error), "mag step");
        const int measured = k - k % 10;  // 100 Hz
        check(s.updated == (k % 10 == 0), "mag measures every 10 ms");
        const sa::Vector3 expected = mag_truth(measured).magnetic_field;
        check(std::fabs(s.value.x - expected.x) < 1e-6 && std::fabs(s.value.y - expected.y) < 1e-6 &&
                  std::fabs(s.value.z - expected.z) < 1e-6,
              "mag equals the world field rotated into the sensor axes [nT]");
    }

    auto noisy = sa::gazebo::create_gz_mag_sensor_adapter(setup_for<sa::MagAdapterSetup>("mag-noise.sdf"), error);
    std::vector<double> ex;
    for (int k = 0; k < 4000; ++k) {
        sa::MagSample s{};
        noisy->step(mag_truth(k), s, error);
        if (s.updated) ex.push_back(s.value.x - mag_truth(k).magnetic_field.x);
    }
    std::printf("mag x noise: n=%zu sd=%.2f nT (configured 1e-8 T = 10 nT)\n", ex.size(), stddev(ex));
    check(stddev(ex) > 8.0 && stddev(ex) < 12.0, "mag noise in tesla becomes 10 nT");
}

// --- air pressure ---

// Independent expectation: standard troposphere with geopotential height.
double expected_pressure(double altitude_m)
{
    const double r = 6356766.0;
    const double h = r * altitude_m / (r + altitude_m);
    return 101325.0 * std::pow(1.0 - 0.0065 * h / 288.15, 9.80665 * 0.0289644 / (8.31432 * 0.0065));
}

void air_pressure()
{
    std::string error;
    auto ideal = sa::gazebo::create_gz_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("baro-ideal.sdf"), error);
    check(ideal != nullptr, "air pressure sensor is created");
    if (!ideal) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    for (int k = 0; k < 45; ++k) {
        sa::BaroTruth t{};
        t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
        t.altitude_m = 1000.0 + 10.0 * k;
        sa::BaroSample s{};
        check(ideal->step(t, s, error), "baro step");
        const int measured = k - k % 20;  // 50 Hz
        check(s.updated == (k % 20 == 0), "baro measures every 20 ms");
        check(std::fabs(s.pressure_pa - expected_pressure(1000.0 + 10.0 * measured)) < 1.0,
              "baro pressure follows the standard troposphere");
    }

    // Two instances in one process keep their own messages.
    auto low = sa::gazebo::create_gz_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("baro-ideal.sdf"), error);
    auto high = sa::gazebo::create_gz_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("baro-ideal.sdf"), error);
    sa::BaroTruth tl{}, th{};
    tl.altitude_m = 0.0;
    th.altitude_m = 3000.0;
    sa::BaroSample sl{}, sh{};
    check(low->step(tl, sl, error) && high->step(th, sh, error), "two barometers step");
    check(std::fabs(sl.pressure_pa - 101325.0) < 1.0 && std::fabs(sh.pressure_pa - expected_pressure(3000.0)) < 1.0,
          "two barometers do not mix their values");

    auto noisy = sa::gazebo::create_gz_baro_sensor_adapter(setup_for<sa::BaroAdapterSetup>("baro-noise.sdf"), error);
    std::vector<double> e;
    for (int k = 0; k < 4000; ++k) {
        sa::BaroTruth t{};
        t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
        sa::BaroSample s{};
        noisy->step(t, s, error);
        if (s.updated) e.push_back(s.pressure_pa - 101325.0);
    }
    std::printf("baro noise: n=%zu sd=%.2f Pa (configured 10)\n", e.size(), stddev(e));
    check(stddev(e) > 7.5 && stddev(e) < 12.5, "baro noise in pascals");
    check(!noisy->capabilities().sea_level_weather && !noisy->capabilities().temperature,
          "capabilities: no weather, no temperature");
}

// --- navsat ---

sa::GpsTruth gps_truth(int k)
{
    sa::GpsTruth t{};
    t.sim_time_ns = static_cast<std::uint64_t>(k) * 1000000;
    t.position = {47.641468 + 1e-6 * k, -122.140165, 121.321 + 0.01 * k};
    t.velocity_nwu = {2.0, -1.0, 0.5};  // north, east (west -1), up
    return t;
}

void navsat()
{
    std::string error;
    auto ideal = sa::gazebo::create_gz_gps_sensor_adapter(setup_for<sa::GpsAdapterSetup>("gps-ideal.sdf"), error);
    check(ideal != nullptr, "navsat is created");
    if (!ideal) {
        std::fprintf(stderr, "  %s\n", error.c_str());
        return;
    }
    for (int k = 0; k < 250; ++k) {
        sa::GpsSample s{};
        check(ideal->step(gps_truth(k), s, error), "gps step");
        const int measured = k - k % 100;  // 10 Hz
        const sa::GpsTruth t = gps_truth(measured);
        check(s.updated == (k % 100 == 0), "gps measures every 100 ms");
        check(std::fabs(s.position.latitude_deg - t.position.latitude_deg) < 1e-12 &&
                  std::fabs(s.position.longitude_deg - t.position.longitude_deg) < 1e-12 &&
                  std::fabs(s.position.altitude_m - t.position.altitude_m) < 1e-9,
              "gps position equals the truth");
        check(s.velocity_nwu.x == 2.0 && s.velocity_nwu.y == -1.0 && s.velocity_nwu.z == 0.5,
              "gps velocity round trip NWU -> ENU -> NWU");
        check(s.fix_type == 3, "gps reports a 3D fix");
    }

    auto noisy = sa::gazebo::create_gz_gps_sensor_adapter(setup_for<sa::GpsAdapterSetup>("gps-noise.sdf"), error);
    std::vector<double> lat, alt, vn;
    for (int k = 0; k < 40000; ++k) {
        sa::GpsSample s{};
        noisy->step(gps_truth(k), s, error);
        if (s.updated) {
            lat.push_back(s.position.latitude_deg - gps_truth(k).position.latitude_deg);
            alt.push_back(s.position.altitude_m - gps_truth(k).position.altitude_m);
            vn.push_back(s.velocity_nwu.x - 2.0);
        }
    }
    std::printf("gps noise: n=%zu sd lat=%.2e deg alt=%.3f m vel north=%.3f m/s (configured 1e-5 deg, 0.5, 0.1)\n",
                lat.size(), stddev(lat), stddev(alt), stddev(vn));
    check(stddev(lat) > 0.75e-5 && stddev(lat) < 1.25e-5, "horizontal noise is in degrees");
    check(stddev(alt) > 0.375 && stddev(alt) < 0.625, "vertical noise is in metres");
    check(stddev(vn) > 0.075 && stddev(vn) < 0.125, "velocity noise is in m/s");

    check(noisy->reset(error), "gps reset");
    sa::GpsSample first_after_reset{};
    noisy->step(gps_truth(0), first_after_reset, error);
    auto again = sa::gazebo::create_gz_gps_sensor_adapter(setup_for<sa::GpsAdapterSetup>("gps-noise.sdf"), error);
    sa::GpsSample first_new{};
    again->step(gps_truth(0), first_new, error);
    check(first_after_reset.position.latitude_deg == first_new.position.latitude_deg,
          "same seed: reset and a new instance start with the same value");
}

}  // namespace

int main()
{
    magnetometer();
    air_pressure();
    navsat();
    if (failures == 0) {
        std::printf("gz_mag_baro_gps_sensor_adapter_test: OK\n");
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
