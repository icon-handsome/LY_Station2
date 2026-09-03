// thickness-measure-v3-worker: loads ThicknessMeasureV3.dll in an isolated process.
// Writes result.txt BEFORE tmv3_destroy so a teardown crash still leaves a usable result.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <Windows.h>

#include "thickness_measure_v3_c_api.h"

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

[[noreturn]] void ExitWithoutDllTeardown(int exitCode)
{
    // The vendor/PCL/ONNX teardown path can corrupt the heap after a valid
    // measurement. The result file is already closed before this is called;
    // terminate the short-lived worker and let the OS reclaim its address
    // space without invoking DLL or C++ destructors.
    ::TerminateProcess(::GetCurrentProcess(), static_cast<UINT>(exitCode));
    // TerminateProcess only returns on failure. Keep a hard exit fallback so
    // the worker cannot continue into tmv3_destroy in that unlikely case.
    std::_Exit(exitCode);
}

bool LoadXyzBin(const std::string& path, size_t pointCount, std::vector<float>* xyz)
{
    xyz->assign(pointCount * 3, 0.0f);
    if (pointCount == 0) {
        return true;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    const auto expected = static_cast<std::streamsize>(xyz->size() * sizeof(float));
    in.read(reinterpret_cast<char*>(xyz->data()), expected);
    return in.gcount() == expected;
}

std::string EscapeMessage(std::string message)
{
    for (char& c : message) {
        if (c == '\n' || c == '\r') {
            c = ' ';
        }
    }
    return message;
}

int Fail(const std::string& resultPath, int status, const std::string& message, int exitCode)
{
    std::map<std::string, std::string> kv;
    kv["status"] = std::to_string(status);
    kv["message"] = EscapeMessage(message);
    kv["thickness_mm"] = "0";
    kv["pair_count"] = "0";
    kv["success_count"] = "0";
    kv["valid"] = "0";
    WriteResult(resultPath, kv);
    std::fprintf(stderr, "thickness-measure-v3-worker FAIL: %s\n", message.c_str());
    return exitCode;
}

}  // namespace

int main(int argc, char* argv[])
{
    if (argc < 2) {
        std::fprintf(stderr,
                     "Usage: thickness-measure-v3-worker.exe <job_workdir>\n"
                     "  Expects request.txt in workdir; writes result.txt before DLL destroy.\n");
        return 1;
    }

    const std::string workDir = argv[1];
    const std::string requestPath = JoinPath(workDir, "request.txt");
    const std::string resultPath = JoinPath(workDir, "result.txt");

    std::fprintf(stdout, "thickness-measure-v3-worker start workdir=%s\n", workDir.c_str());
    std::fflush(stdout);

    std::map<std::string, std::string> req;
    if (!LoadKeyValues(requestPath, &req)) {
        return Fail(resultPath, TMV3_ERR_INVALID_ARG, "Cannot read request.txt", 4);
    }

    const std::string mode = req.count("mode") ? req["mode"] : "pairs_average";
    const std::string config = req.count("config") ? req["config"] : "";
    if (config.empty()) {
        return Fail(resultPath, TMV3_ERR_CONFIG, "Missing config= in request.txt", 2);
    }

    size_t pairCount = 0;
    if (req.count("pair_count")) {
        pairCount = static_cast<size_t>(std::strtoull(req["pair_count"].c_str(), nullptr, 10));
    }
    if (pairCount == 0) {
        return Fail(resultPath, TMV3_ERR_INVALID_ARG, "pair_count must be > 0", 1);
    }

    std::vector<std::vector<float>> innerClouds(pairCount);
    std::vector<std::vector<float>> outerClouds(pairCount);
    std::vector<tmv3_pair_clouds> views(pairCount);
    for (size_t i = 0; i < pairCount; ++i) {
        const std::string is = std::to_string(i);
        const std::string innerKey = "inner_" + is;
        const std::string outerKey = "outer_" + is;
        const std::string innerCountKey = "inner_" + is + "_count";
        const std::string outerCountKey = "outer_" + is + "_count";
        if (!req.count(innerKey) || !req.count(outerKey) || !req.count(innerCountKey) ||
            !req.count(outerCountKey)) {
            return Fail(resultPath, TMV3_ERR_INVALID_ARG, "Missing pair cloud keys for index " + is, 1);
        }
        const size_t innerCount =
            static_cast<size_t>(std::strtoull(req[innerCountKey].c_str(), nullptr, 10));
        const size_t outerCount =
            static_cast<size_t>(std::strtoull(req[outerCountKey].c_str(), nullptr, 10));
        if (!LoadXyzBin(JoinPath(workDir, req[innerKey]), innerCount, &innerClouds[i]) ||
            !LoadXyzBin(JoinPath(workDir, req[outerKey]), outerCount, &outerClouds[i])) {
            return Fail(resultPath, TMV3_ERR_CLOUD, "Failed to load cloud bins for pair " + is, 4);
        }
        views[i].inner.xyz = innerClouds[i].data();
        views[i].inner.point_count = innerCount;
        views[i].outer.xyz = outerClouds[i].data();
        views[i].outer.point_count = outerCount;
        std::fprintf(stdout,
                     "  pair[%zu] inner=%zu outer=%zu\n",
                     i,
                     innerCount,
                     outerCount);
    }
    std::fflush(stdout);

    tmv3_context* ctx = nullptr;
    char message[512] = {0};
    const tmv3_status createStatus =
        tmv3_create_from_ini(config.c_str(), &ctx, message, sizeof(message));
    if (createStatus != TMV3_OK || ctx == nullptr) {
        return Fail(resultPath,
                    createStatus,
                    message[0] != '\0' ? message : tmv3_status_string(createStatus),
                    2);
    }
    std::fprintf(stdout, "tmv3_create_from_ini OK config=%s\n", config.c_str());
    std::fflush(stdout);

    std::map<std::string, std::string> resultKv;
    resultKv["status"] = "0";
    resultKv["message"] = "";

    tmv3_status measureStatus = TMV3_OK;
    if (mode == "pair") {
        tmv3_pair_result pair{};
        measureStatus = tmv3_measure_pair(ctx,
                                          views[0].inner.xyz,
                                          views[0].inner.point_count,
                                          views[0].outer.xyz,
                                          views[0].outer.point_count,
                                          &pair,
                                          message,
                                          sizeof(message));
        resultKv["status"] = std::to_string(static_cast<int>(measureStatus));
        resultKv["message"] = EscapeMessage(message[0] != '\0' ? message : "");
        resultKv["thickness_mm"] = std::to_string(pair.thickness_mm);
        resultKv["pair_count"] = "1";
        resultKv["success_count"] = (measureStatus == TMV3_OK && pair.valid) ? "1" : "0";
        resultKv["valid"] = pair.valid ? "1" : "0";
        resultKv["method"] = pair.method;
        resultKv["section_count"] = std::to_string(pair.section_count);
        resultKv["inner_outer_icp_fitness"] = std::to_string(pair.inner_outer_icp_fitness);
        resultKv["outer_template_icp_fitness"] = std::to_string(pair.outer_template_icp_fitness);
        std::fprintf(stdout,
                     "tmv3_measure_pair status=%d valid=%d thickness=%.4f\n",
                     static_cast<int>(measureStatus),
                     pair.valid,
                     pair.thickness_mm);
    } else {
        tmv3_average_result average{};
        measureStatus = tmv3_measure_pairs_average(
            ctx, views.data(), views.size(), &average, message, sizeof(message));
        resultKv["status"] = std::to_string(static_cast<int>(measureStatus));
        resultKv["message"] = EscapeMessage(message[0] != '\0' ? message : "");
        resultKv["thickness_mm"] = std::to_string(average.thickness_mm);
        resultKv["pair_count"] = std::to_string(average.pair_count);
        resultKv["success_count"] = std::to_string(average.success_count);
        resultKv["valid"] = average.valid ? "1" : "0";
        std::fprintf(stdout,
                     "tmv3_measure_pairs_average status=%d valid=%d thickness=%.4f success=%zu/%zu\n",
                     static_cast<int>(measureStatus),
                     average.valid,
                     average.thickness_mm,
                     average.success_count,
                     average.pair_count);
    }
    std::fflush(stdout);

    // Persist before process termination: field crashes often happen during
    // PCL/ONNX teardown, so do not call tmv3_destroy in this worker.
    if (!WriteResult(resultPath, resultKv)) {
        std::fprintf(stderr, "Failed to write result.txt\n");
        ExitWithoutDllTeardown(4);
    }
    std::fprintf(stdout, "result.txt written, terminating worker without DLL teardown...\n");
    std::fflush(stdout);

    const bool measureOk = measureStatus == TMV3_OK && resultKv["valid"] == "1" &&
        resultKv["success_count"] != "0";
    ExitWithoutDllTeardown(measureOk ? 0 : 3);
}
