#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <QtCore/QString>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

#include "BevelMeasurement.h"

namespace scan_tracking::vision::bevel {

struct BevelInspectionResult {
    bool invoked = false;
    bool ok = false;
    int bevelType = -1;
    float angleDeg = 0.0f;
    float lengthMm = 0.0f;
    float icpFitness = 0.0f;
    int qualityCode = 10000;
    QString message;
};

/// 将 Mech-Eye 点云帧转换为 PCL 点云（供测试与其它模块复用）。
pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame);

/// 解析 config.ini / 环境变量 / exe 旁 bevel 目录，得到 config.txt 绝对路径。
QString resolveBevelConfigPath();

/// 解析模板目录绝对路径（可为空，表示使用 config.txt 内相对路径）。
QString resolveBevelTemplateDir();

/// 由工艺配方与公差构造 Po_Kou 求解选项（供测试校验映射）。
::bevel::BevelSolveOptions buildBevelSolveOptions(
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm);

BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud,
    const scan_tracking::common::BevelRecipe& recipe,
    float angleTolDeg,
    float lengthTolMm);

BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud);

}  // namespace scan_tracking::vision::bevel
