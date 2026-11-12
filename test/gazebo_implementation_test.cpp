// Tests of the "gazebo" descriptor, the JSON definition files and the joint adapter.
//
// The JSON form of each example SDF (config/*.json, made with tools/sdf_json.py)
// must give exactly the same measurements as the SDF itself: same seed, same
// truth, same steps. The SDF path is the one the adapter tests already check.
#include "hakoniwa/sensor_adapter/sensor_adapter_implementation.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

HAKO_SENSOR_ADAPTER_DECLARE_IMPLEMENTATION(gazebo);

namespace sa = hakoniwa::sensor_adapter;

namespace {

int failures = 0;
const sa::SensorAdapterImplementation* gazebo = nullptr;
constexpr int kSteps = 300;

void check(bool condition, const std::string& what)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what.c_str());
        ++failures;
    }
}

std::string config(const std::string& file)
{
    return std::string(HAKO_GZ_TEST_CONFIG_DIR) + "/" + file;
}

template <typename Setup>
Setup setup_for(const std::string& path)
{
    Setup setup{};
    setup.config_path = path;
    setup.physics_step_ns = 1000000;  // 1 ms
    setup.has_random_seed = true;
    setup.random_seed = 4242;
    return setup;
}

bool same(const sa::Vector3& a, const sa::Vector3& b)
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

std::uint64_t at(int k)
{
    return static_cast<std::uint64_t>(k) * 1000000;
}

// --- one run per type: the measurements as plain numbers ---

std::vector<double> run_imu(const std::string& path)
{
    std::string error;
    auto imu = gazebo->create_imu(setup_for<sa::ImuAdapterSetup>(path), error);
    check(imu != nullptr, path + ": " + error);
    std::vector<double> values;
    for (int k = 0; imu && k < kSteps; ++k) {
        sa::ImuTruth truth{};
        truth.sim_time_ns = at(k);
        truth.angular_velocity = {0.1, -0.2, 0.01 * k};
        truth.orientation_world_to_sensor = {1.0, 0.0, 0.0, 0.0};
        truth.gravity_world = {0.0, 0.0, -9.80665};
        truth.specific_force = {0.0, 0.0, 9.80665};
        sa::ImuSample out{};
        check(imu->step(truth, out, error), path + ": " + error);
        values.insert(values.end(), {out.gyro.value.x, out.gyro.value.y, out.gyro.value.z, out.accel.value.x,
                                     out.accel.value.y, out.accel.value.z});
    }
    return values;
}

std::vector<double> run_mag(const std::string& path)
{
    std::string error;
    auto mag = gazebo->create_mag(setup_for<sa::MagAdapterSetup>(path), error);
    check(mag != nullptr, path + ": " + error);
    std::vector<double> values;
    for (int k = 0; mag && k < kSteps; ++k) {
        sa::MagTruth truth{};
        truth.sim_time_ns = at(k);
        truth.magnetic_field = {30000.0, -2000.0, -40000.0};
        truth.magnetic_field_world = truth.magnetic_field;
        truth.orientation_world_to_sensor = {1.0, 0.0, 0.0, 0.0};
        sa::MagSample out{};
        check(mag->step(truth, out, error), path + ": " + error);
        values.insert(values.end(), {out.value.x, out.value.y, out.value.z});
    }
    return values;
}

std::vector<double> run_baro(const std::string& path)
{
    std::string error;
    auto baro = gazebo->create_baro(setup_for<sa::BaroAdapterSetup>(path), error);
    check(baro != nullptr, path + ": " + error);
    std::vector<double> values;
    for (int k = 0; baro && k < kSteps; ++k) {
        sa::BaroTruth truth{};
        truth.sim_time_ns = at(k);
        truth.altitude_m = 40.0 + 0.001 * k;
        sa::BaroSample out{};
        check(baro->step(truth, out, error), path + ": " + error);
        values.insert(values.end(), {out.pressure_pa, out.temperature_c, out.pressure_altitude_m});
    }
    return values;
}

std::vector<double> run_gps(const std::string& path)
{
    std::string error;
    auto setup = setup_for<sa::GpsAdapterSetup>(path);
    auto gps = gazebo->create_gps(setup, error);
    check(gps != nullptr, path + ": " + error);
    std::vector<double> values;
    for (int k = 0; gps && k < kSteps; ++k) {
        sa::GpsTruth truth{};
        truth.sim_time_ns = at(k);
        truth.position = {35.681236, 139.767125, 40.0};
        truth.velocity_nwu = {1.0, 0.5, 0.0};
        sa::GpsSample out{};
        check(gps->step(truth, out, error), path + ": " + error);
        values.insert(values.end(), {out.position.latitude_deg, out.position.longitude_deg, out.position.altitude_m,
                                     out.ground_speed_mps});
    }
    return values;
}

std::vector<double> run_force_torque(const std::string& path)
{
    std::string error;
    auto ft = gazebo->create_force_torque(setup_for<sa::ForceTorqueAdapterSetup>(path), error);
    check(ft != nullptr, path + ": " + error);
    std::vector<double> values;
    for (int k = 0; ft && k < kSteps; ++k) {
        sa::ForceTorqueTruth truth{};
        truth.sim_time_ns = at(k);
        truth.wrench.force = {1.0, -2.0, -19.6};
        truth.wrench.torque = {0.1, 1.96, -0.3};
        sa::ForceTorqueSample out{};
        check(ft->step(truth, out, error), path + ": " + error);
        values.insert(values.end(), {out.wrench.force.x, out.wrench.force.y, out.wrench.force.z,
                                     out.wrench.torque.x, out.wrench.torque.y, out.wrench.torque.z});
    }
    return values;
}

// The JSON form gives bit-identical measurements to the SDF.
template <typename Run>
void json_equals_sdf(const char* stem, Run run)
{
    const std::vector<double> from_sdf = run(config(std::string(stem) + ".sdf"));
    const std::vector<double> from_json = run(config(std::string(stem) + ".json"));
    check(!from_sdf.empty(), std::string(stem) + ": measured");
    check(from_sdf.size() == from_json.size()
              && std::memcmp(from_sdf.data(), from_json.data(), from_sdf.size() * sizeof(double)) == 0,
          std::string(stem) + ": the JSON definition measures exactly as the SDF");
}

void descriptor()
{
    check(gazebo->version == sa::kSensorAdapterImplementationVersion, "descriptor version");
    check(std::strcmp(gazebo->name, "gazebo") == 0, "descriptor name");
    const std::vector<std::string> expected {"imu", "mag", "baro", "gps", "force_torque", "joint"};
    check(sa::provided_sensor_types(*gazebo) == expected, "provides imu, mag, baro, gps, force_torque and joint");
}

std::string write_file(const std::string& name, const std::string& text)
{
    const std::string path = std::string(HAKO_GZ_TEST_BINARY_DIR) + "/" + name;
    std::ofstream(path) << text;
    return path;
}

// A bad JSON definition fails at creation and the error names the place.
void json_errors()
{
    struct Case {
        const char* name;
        const char* text;
        const char* expected;
    };
    const Case cases[] = {
        {"bad-type.json", R"({"type": "imu", "sensor": {"update_rate": 10}})", "type must be \"baro\""},
        {"no-sensor.json", R"({"type": "baro"})", "\"sensor\" object"},
        {"boolean.json", R"({"type": "baro", "sensor": {"always_on": true}})", "sensor.always_on"},
        {"null.json", R"({"type": "baro", "sensor": {"update_rate": null}})", "sensor.update_rate"},
        {"bad-name.json", R"({"type": "baro", "sensor": {"bad name": 1}})", "sensor.bad name"},
        {"gz-type.json", R"({"type": "baro", "sensor": {"@type": "imu"}})", "sensor.@type"},
    };
    for (const auto& c : cases) {
        std::string error;
        auto baro = gazebo->create_baro(setup_for<sa::BaroAdapterSetup>(write_file(c.name, c.text)), error);
        check(baro == nullptr && error.find(c.expected) != std::string::npos,
              std::string(c.name) + ": rejected naming the place (got: " + error + ")");
    }
}

sa::JointSensorTruth joint_truth(int k)
{
    sa::JointSensorTruth truth{};
    truth.sim_time_ns = at(k);
    truth.position = 0.5 + 0.001 * k;
    truth.velocity = -0.25 + 0.002 * k;
    truth.actuator_torque = 1.0;
    truth.transmitted_torque = 1.0;
    return truth;
}

// The joint adapter reports the truth, at the period, held between.
void joint_true_values()
{
    std::string error;
    auto every = gazebo->create_joint(
        setup_for<sa::JointSensorAdapterSetup>(write_file("joint-every.json", R"({"type": "joint"})")), error);
    check(every != nullptr, "joint without update_rate: " + error);
    for (int k = 0; every && k < 20; ++k) {
        sa::JointSensorSample out{};
        check(every->step(joint_truth(k), out, error), error);
        check(out.updated && out.sample_time_ns == at(k), "measured every step without update_rate");
        check(out.position == joint_truth(k).position && out.velocity == joint_truth(k).velocity,
              "position and velocity are the truth");
        check(!out.has_effort, "no effort");
    }

    auto rate = gazebo->create_joint(
        setup_for<sa::JointSensorAdapterSetup>(
            write_file("joint-100.json", R"({"type": "joint", "update_rate": 100})")),
        error);
    check(rate != nullptr, "joint at 100 Hz: " + error);
    for (int k = 0; rate && k < 35; ++k) {
        sa::JointSensorSample out{};
        check(rate->step(joint_truth(k), out, error), error);
        const int measured = k - k % 10;  // 100 Hz with 1 ms steps
        check(out.updated == (k % 10 == 0), "100 Hz: measured every 10 steps");
        check(out.sample_time_ns == at(measured) && out.position == joint_truth(measured).position,
              "100 Hz: the last measurement is held");
    }

    std::string bad;
    auto unknown = gazebo->create_joint(
        setup_for<sa::JointSensorAdapterSetup>(write_file("joint-bad.json", R"({"type": "joint", "noise": 1})")),
        bad);
    check(unknown == nullptr && bad.find("noise") != std::string::npos, "an unknown joint key is rejected");
}

}  // namespace

int main()
{
    gazebo = HAKO_SENSOR_ADAPTER_IMPLEMENTATION_ENTRY(gazebo)();
    if (gazebo == nullptr) {
        std::fprintf(stderr, "FAIL: no descriptor\n");
        return 1;
    }
    descriptor();
    json_equals_sdf("imu-ideal", run_imu);
    json_equals_sdf("imu-noise", run_imu);
    json_equals_sdf("mag-ideal", run_mag);
    json_equals_sdf("mag-noise", run_mag);
    json_equals_sdf("baro-ideal", run_baro);
    json_equals_sdf("baro-noise", run_baro);
    json_equals_sdf("gps-ideal", run_gps);
    json_equals_sdf("gps-noise", run_gps);
    json_equals_sdf("ft-ideal", run_force_torque);
    json_equals_sdf("ft-noise", run_force_torque);
    json_errors();
    joint_true_values();
    if (failures != 0) {
        std::fprintf(stderr, "%d failure(s)\n", failures);
        return 1;
    }
    std::printf("gazebo_implementation_test: OK\n");
    return 0;
}
