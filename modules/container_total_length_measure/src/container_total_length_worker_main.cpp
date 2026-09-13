// container-total-length-worker: loads ContainerTotalLength.dll in an isolated process.
// Writes result.txt before process exit so a teardown crash still leaves a usable result.

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Windows.h>

#include "ContainerTotalLengthApi.h"

namespace {

std::string JoinPath(const std::string& dir, const std::string& name)
{
    if (dir.empty()) {
        return name;
    }
    const char last = dir.back();
    if (last == '\\' || last == '/') {
        return dir + name;
    }
    return dir + '\\' + name;
}

std::string Trim(std::string s)
{
    while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) {
        s.pop_back();
    }
    size_t i = 0;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t')) {
        ++i;
    }
    return s.substr(i);
}

bool LoadKeyValues(const std::string& path, std::map<std::string, std::string>* out)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        line = Trim(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        const size_t eq = line.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        (*out)[Trim(line.substr(0, eq))] = Trim(line.substr(eq + 1));
    }
    return true;
}

bool WriteResult(const std::string& path, const std::map<std::string, std::string>& kv)
{
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        return false;
    }
    for (const auto& item : kv) {
        out << item.first << '=' << item.second << '\n';
    }
    out.flush();
    return static_cast<bool>(out);
}

bool LoadBin(const std::string& path, size_t pointCount, std::vector<float>* xyz)
{
    xyz->assign(pointCount * 3, 0.0f);
    if (pointCount == 0) {
        return true;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    const auto bytes = static_cast<std::streamsize>(xyz->size() * sizeof(float));
    in.read(reinterpret_cast<char*>(xyz->data()), bytes);
    return in.gcount() == bytes;
}

[[noreturn]] void Fail(const std::string& resultPath, int status, const std::string& message, int exitCode)
{
    WriteResult(
        resultPath,
        {{"status", std::to_string(status)},
         {"message", message},
         {"valid", "0"},
         {"length_mm", "0"},
         {"left_end_position", "0"},
         {"right_end_position", "0"},
         {"icp_fitness", "0"},
         {"fitted_radius_mm", "0"},
         {"cylinder_point_x", "0"},
         {"cylinder_point_y", "0"},
         {"cylinder_point_z", "0"},
         {"cylinder_axis_x", "0"},
         {"cylinder_axis_y", "0"},
         {"cylinder_axis_z", "0"},
         {"icp_converged", "0"},
         {"input_point_count", "0"}});
    std::fprintf(stderr, "container-total-length-worker FAIL: %s\n", message.c_str());
    ExitProcess(static_cast<UINT>(exitCode));
}

std::string FormatDouble(double value)
{
    char buf[64] = {};
    std::snprintf(buf, sizeof(buf), "%.9g", value);
    return buf;
}

std::string FormatFloat(float value)
{
    char buf[64] = {};
    std::snprintf(buf, sizeof(buf), "%.9g", static_cast<double>(value));
    return buf;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(
            stderr,
            "Usage: container-total-length-worker.exe <job_workdir>\n"
            "Expects request.txt + cloud.bin; writes result.txt.\n");
        return 1;
    }

    const std::string workDir = argv[1];
    const std::string resultPath = JoinPath(workDir, "result.txt");
    std::fprintf(stdout, "container-total-length-worker start workdir=%s\n", workDir.c_str());

    std::map<std::string, std::string> request;
    if (!LoadKeyValues(JoinPath(workDir, "request.txt"), &request)) {
        Fail(resultPath, CTL_ERR_INVALID_ARGUMENT, "Cannot read request.txt", 4);
    }

    const std::string configPath = request["config"];
    if (configPath.empty()) {
        Fail(resultPath, CTL_ERR_CONFIG, "Missing config= in request.txt", 2);
    }

    const std::string cloudName =
        request.count("cloud") && !request["cloud"].empty() ? request["cloud"] : "cloud.bin";
    const size_t pointCount = std::strtoull(request["point_count"].c_str(), nullptr, 10);
    if (pointCount == 0) {
        Fail(resultPath, CTL_ERR_INVALID_ARGUMENT, "point_count must be > 0", 1);
    }

    std::vector<float> xyz;
    if (!LoadBin(JoinPath(workDir, cloudName), pointCount, &xyz)) {
        Fail(resultPath, CTL_ERR_INPUT, "Failed to load cloud.bin", 4);
    }

    std::fprintf(
        stdout,
        "worker config=%s points=%llu\n",
        configPath.c_str(),
        static_cast<unsigned long long>(pointCount));

    ctl_context* ctx = nullptr;
    char message[512] = {};
    const ctl_status createStatus =
        ctl_create_from_ini(configPath.c_str(), &ctx, message, sizeof(message));
    if (createStatus != CTL_OK || ctx == nullptr) {
        Fail(
            resultPath,
            createStatus != CTL_OK ? static_cast<int>(createStatus) : CTL_ERR_CONFIG,
            message[0] != '\0' ? message : "ctl_create_from_ini failed",
            2);
    }

    ctl_result result{};
    message[0] = '\0';
    const ctl_status measureStatus =
        ctl_measure(ctx, xyz.data(), pointCount, &result, message, sizeof(message));

    // Persist result before destroy/teardown so a DLL leak or crash still leaves
    // a readable outcome for the host.
    const bool valid = measureStatus == CTL_OK && result.valid != 0;
    WriteResult(
        resultPath,
        {{"status", std::to_string(static_cast<int>(measureStatus))},
         {"message", message[0] != '\0' ? message : (valid ? "" : "measure invalid")},
         {"valid", valid ? "1" : "0"},
         {"length_mm", FormatDouble(result.length_mm)},
         {"left_end_position", FormatDouble(result.left_end_position)},
         {"right_end_position", FormatDouble(result.right_end_position)},
         {"icp_fitness", FormatDouble(result.icp_fitness)},
         {"fitted_radius_mm", FormatFloat(result.fitted_radius_mm)},
         {"cylinder_point_x", FormatFloat(result.cylinder_point_x)},
         {"cylinder_point_y", FormatFloat(result.cylinder_point_y)},
         {"cylinder_point_z", FormatFloat(result.cylinder_point_z)},
         {"cylinder_axis_x", FormatFloat(result.cylinder_axis_x)},
         {"cylinder_axis_y", FormatFloat(result.cylinder_axis_y)},
         {"cylinder_axis_z", FormatFloat(result.cylinder_axis_z)},
         {"icp_converged", result.icp_converged != 0 ? "1" : "0"},
         {"input_point_count", std::to_string(result.input_point_count)}});

    // Prefer process exit over relying on ctl_destroy: algorithm TEST_LEAK (if
    // present) is reclaimed by the OS when this short-lived worker terminates.
    if (ctx != nullptr) {
        ctl_destroy(ctx);
        ctx = nullptr;
    }

    std::fprintf(
        stdout,
        "result.txt written valid=%d length_mm=%.3f exit=%d\n",
        valid ? 1 : 0,
        result.length_mm,
        valid ? 0 : 3);
    ExitProcess(valid ? 0 : 3);
}
