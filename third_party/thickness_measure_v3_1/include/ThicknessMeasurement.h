#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "Config.h"

struct SectionThicknessResult 
{
    int sectionIndex;
    double thicknessA;
    double thicknessB;
    Point3d outerA;
    Point3d outerB;
    Point3d innerA;
    Point3d innerB;
};

struct ThicknessResult 
{
    double innerOuterIcpFitnessScore;
    double outerTemplateIcpFitnessScore;
    double thickness;
    std::string method;
    std::vector<SectionThicknessResult> sectionResults;
};

bool MeasureThickness(const ThicknessConfig& config,
                      const std::string& innerPath,
                      const std::string& outerPath,
                      ThicknessResult* result,
                      std::string* error);
bool SaveResult(const std::string& path, const ThicknessResult& result, std::string* error);

// DLL 导出宏。接口使用 STL 类型，调用方需与 DLL 使用相同的 MSVC 工具链。
#ifdef THICKNESS_MEASUREMENT_EXPORTS
#  define THICKNESS_API __declspec(dllexport)
#else
#  define THICKNESS_API __declspec(dllimport)
#endif

class THICKNESS_API ThicknessMeasurement {
public:
    ThicknessMeasurement();
    ~ThicknessMeasurement();
    bool LoadIni(const std::string& iniPath, std::string* error = nullptr);
    bool ReadInputPointClouds(std::string* error = nullptr) const;
    bool ReadTemplatePointClouds(std::string* error = nullptr) const;
    void SetSaveData(bool enabled);
    bool SaveDataEnabled() const;
    // 测量指定的一对内、外表面点云；模板点云取自已加载的 INI 配置第 1 组。
    bool MeasureOnePair(const std::string& innerPath, const std::string& outerPath,
                        ThicknessResult* result, std::string* error = nullptr) const;
    // groupIndex=0：第1组（点点 ICP + 模板1 / DynamicSections）；
    // groupIndex=1：第2组（点面 ICP + 模板2 / DynamicSections2，需 INI 含 [Input2]）。
    bool MeasureOnePairForGroup(int groupIndex,
                                const std::string& innerPath,
                                const std::string& outerPath,
                                ThicknessResult* result,
                                std::string* error = nullptr) const;
    // 测量 INI 中配置的全部组：第 1 组，以及存在时的第 2 组。
    bool MeasureConfiguredGroups(std::vector<ThicknessResult>* results,
                                 std::string* error = nullptr) const;
    // 读取最近一次批量测量的全部结果和总体平均厚度。
    const std::vector<ThicknessResult>& LastResults() const;
    double LastAverageThickness() const;
    // 导出所有截面 A/B 厚度值；顺序为第1组截面1 A、B，第1组截面2 A、B ...。
    void LastSectionThicknessValues(std::vector<double>* values) const;
    const ThicknessConfig& Config() const;
private:
    ThicknessConfig config_;
    bool loaded_;
    mutable std::vector<ThicknessResult> lastResults_;
    mutable double lastAverageThickness_;
};
