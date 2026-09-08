#include <Windows.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "ThicknessMeasurement.h"

namespace {
std::string join(const std::string& d, const std::string& n) { return d + (d.empty() || d.back() == '\\' ? "" : "\\") + n; }
std::string trim(std::string s) { while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back(); return s; }
bool loadKv(const std::string& p, std::map<std::string, std::string>* out) { std::ifstream f(p); if (!f) return false; std::string l; while (std::getline(f, l)) { l = trim(l); const auto e = l.find('='); if (!l.empty() && l[0] != '#' && e != std::string::npos) (*out)[trim(l.substr(0, e))] = trim(l.substr(e + 1)); } return true; }
bool writeKv(const std::string& p, const std::map<std::string, std::string>& values) { std::ofstream f(p); if (!f) return false; for (const auto& x : values) f << x.first << '=' << x.second << '\n'; return true; }
bool loadBin(const std::string& p, size_t count, std::vector<float>* xyz) { xyz->resize(count * 3); std::ifstream f(p, std::ios::binary); if (!f) return false; const auto bytes = static_cast<std::streamsize>(xyz->size() * sizeof(float)); f.read(reinterpret_cast<char*>(xyz->data()), bytes); return f.gcount() == bytes; }
bool writePcd(const std::string& p, const std::vector<float>& xyz) { std::ofstream f(p); if (!f) return false; const size_t n = xyz.size() / 3; f << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH " << n << "\nHEIGHT 1\nPOINTS " << n << "\nDATA ascii\n"; for (size_t i = 0; i < n; ++i) f << xyz[i * 3] << ' ' << xyz[i * 3 + 1] << ' ' << xyz[i * 3 + 2] << '\n'; return true; }
void fail(const std::string& p, int status, const std::string& message, int code) { writeKv(p, {{"status", std::to_string(status)}, {"message", message}, {"thickness_mm", "0"}, {"pair_count", "0"}, {"success_count", "0"}, {"valid", "0"}}); ExitProcess(static_cast<UINT>(code)); }
}

int main(int argc, char** argv)
{
    if (argc < 2) return 1;
    const std::string work = argv[1], resultPath = join(work, "result.txt");
    std::map<std::string, std::string> req;
    if (!loadKv(join(work, "request.txt"), &req)) fail(resultPath, 1, "Cannot read request.txt", 4);
    const size_t innerCount = std::strtoull(req["inner_0_count"].c_str(), nullptr, 10);
    const size_t outerCount = std::strtoull(req["outer_0_count"].c_str(), nullptr, 10);
    std::vector<float> inner, outer;
    if (innerCount == 0 || outerCount == 0 || !loadBin(join(work, req["inner_0"]), innerCount, &inner) || !loadBin(join(work, req["outer_0"]), outerCount, &outer)) fail(resultPath, 4, "Failed to read cloud bins", 4);
    const std::string innerPcd = join(work, "inner.pcd"), outerPcd = join(work, "outer.pcd");
    if (!writePcd(innerPcd, inner) || !writePcd(outerPcd, outer)) fail(resultPath, 4, "Failed to write temporary PCD files", 4);
    ThicknessMeasurement algorithm;
    std::string error;
    if (!algorithm.LoadIni(req["config"], &error)) fail(resultPath, 3, error, 2);
    ThicknessResult measured{};
    const bool ok = algorithm.MeasureOnePair(innerPcd, outerPcd, &measured, &error);
    const bool valid = ok && std::isfinite(measured.thickness);
    writeKv(resultPath, {{"status", valid ? "0" : "6"}, {"message", error}, {"thickness_mm", std::to_string(measured.thickness)}, {"pair_count", "1"}, {"success_count", valid ? "1" : "0"}, {"valid", valid ? "1" : "0"}, {"method", measured.method}, {"section_count", std::to_string(measured.sectionResults.size())}, {"inner_outer_icp_fitness", std::to_string(measured.innerOuterIcpFitnessScore)}, {"outer_template_icp_fitness", std::to_string(measured.outerTemplateIcpFitnessScore)}});
    ExitProcess(valid ? 0 : 3);
}
