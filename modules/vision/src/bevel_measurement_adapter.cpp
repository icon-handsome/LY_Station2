#include "scan_tracking/vision/bevel_measurement_adapter.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <mutex>

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>

#include "BevelMeasurement.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"

namespace scan_tracking::vision::bevel {

namespace {

QString localPathFromEnv(const char* name)
{
    const char* value = std::getenv(name);
    if (value == nullptr || value[0] == '\0') {
        return {};
    }
    return QString::fromLocal8Bit(value);
}

QString defaultBevelRootDirectory()
{
    return QCoreApplication::applicationDirPath() + QStringLiteral("/bevel");
}

}  // namespace

pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (!frame.pointsXYZ || frame.pointCount <= 0) {
        return cloud;
    }

    const auto& points = *frame.pointsXYZ;
    const int availablePointCount = static_cast<int>(points.size() / 3);
    const int pointCount = std::min(frame.pointCount, availablePointCount);
    if (pointCount <= 0) {
        return cloud;
    }

    cloud->points.reserve(static_cast<std::size_t>(pointCount));
    bool allFinite = true;
    for (int index = 0; index < pointCount; ++index) {
        const auto base = static_cast<std::size_t>(index * 3);
        const float x = points[base];
        const float y = points[base + 1];
        const float z = points[base + 2];
        allFinite = allFinite && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
        cloud->points.emplace_back(x, y, z);
    }

    const int expectedGridCount = frame.width * frame.height;
    if (frame.width > 0 && frame.height > 0 && expectedGridCount == pointCount) {
        cloud->width = static_cast<std::uint32_t>(frame.width);
        cloud->height = static_cast<std::uint32_t>(frame.height);
    } else {
        cloud->width = static_cast<std::uint32_t>(pointCount);
        cloud->height = 1;
    }
    cloud->is_dense = allFinite;

    return cloud;
}

QString resolveBevelConfigPath()
{
    const QString envRoot = localPathFromEnv("SCAN_TRACKING_BEVEL_CONFIG_DIR");
    if (!envRoot.isEmpty()) {
        const QFileInfo envConfig(QDir(envRoot).filePath(QStringLiteral("config.txt")));
        if (envConfig.exists()) {
            return envConfig.absoluteFilePath();
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr) {
        const QString configured = configManager->bevelConfig().configPath.trimmed();
        if (!configured.isEmpty()) {
            QFileInfo configuredInfo(configured);
            if (configuredInfo.isAbsolute() && configuredInfo.exists()) {
                return configuredInfo.absoluteFilePath();
            }
            const QFileInfo relativeToExe(
                QDir(QCoreApplication::applicationDirPath()).filePath(configured));
            if (relativeToExe.exists()) {
                return relativeToExe.absoluteFilePath();
            }
        }
    }

    const QFileInfo defaultConfig(defaultBevelRootDirectory() + QStringLiteral("/config.txt"));
    return defaultConfig.absoluteFilePath();
}

QString resolveBevelTemplateDir()
{
    const QString envRoot = localPathFromEnv("SCAN_TRACKING_BEVEL_CONFIG_DIR");
    if (!envRoot.isEmpty()) {
        const QDir envTemplateDir(QDir(envRoot).filePath(QStringLiteral("data/templates")));
        if (envTemplateDir.exists()) {
            return envTemplateDir.absolutePath();
        }
    }

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager != nullptr) {
        const QString configured = configManager->bevelConfig().templateDir.trimmed();
        if (!configured.isEmpty()) {
            QFileInfo configuredInfo(configured);
            if (configuredInfo.isAbsolute() && configuredInfo.exists()) {
                return configuredInfo.absoluteFilePath();
            }
            const QFileInfo relativeToExe(
                QDir(QCoreApplication::applicationDirPath()).filePath(configured));
            if (relativeToExe.exists()) {
                return relativeToExe.absoluteFilePath();
            }
        }
    }

    const QDir defaultTemplateDir(defaultBevelRootDirectory() + QStringLiteral("/data/templates"));
    return defaultTemplateDir.exists() ? defaultTemplateDir.absolutePath() : QString();
}

BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud)
{
    BevelInspectionResult result;

    auto pclCloud = toPclPointCloud(cloud);
    if (pclCloud->empty()) {
        result.message = QStringLiteral("坡口测量缺少有效输入点云。");
        return result;
    }

    const QString configPath = resolveBevelConfigPath();
    if (!QFileInfo::exists(configPath)) {
        result.message = QStringLiteral("坡口测量配置文件不存在：%1").arg(configPath);
        return result;
    }

    const QString templateDir = resolveBevelTemplateDir();
    result.invoked = true;

    try {
        std::lock_guard<std::mutex> pclGuard(
            scan_tracking::mech_eye::pointCloudAlgorithmMutex());

        const std::string configPathUtf8 = configPath.toLocal8Bit().toStdString();
        const std::string templateDirUtf8 = templateDir.toLocal8Bit().toStdString();

        const ::bevel::BevelMeasurementResult algorithmResult =
            templateDir.isEmpty()
                ? ::bevel::solveBevelFromRawCloud(pclCloud, configPathUtf8)
                : ::bevel::solveBevelFromRawCloud(
                      pclCloud, configPathUtf8, templateDirUtf8);

        result.ok = algorithmResult.ok;
        result.bevelType = algorithmResult.bevelType;
        result.angleDeg = static_cast<float>(algorithmResult.angleDeg);
        result.lengthMm = static_cast<float>(algorithmResult.length);
        result.icpFitness = static_cast<float>(algorithmResult.icpFitness);
        result.qualityCode = algorithmResult.qualityCode;
        result.message = algorithmResult.message.empty()
            ? QString()
            : QString::fromLocal8Bit(algorithmResult.message.c_str());

        if (result.ok && result.message.isEmpty()) {
            result.message = QStringLiteral(
                "坡口测量完成：angle=%1 deg, length=%2 mm, qualityCode=%3")
                                 .arg(result.angleDeg, 0, 'f', 3)
                                 .arg(result.lengthMm, 0, 'f', 3)
                                 .arg(result.qualityCode);
        }
    } catch (const std::exception& ex) {
        result.ok = false;
        result.message = QStringLiteral("坡口测量抛出异常：%1")
                             .arg(QString::fromLocal8Bit(ex.what()));
    } catch (...) {
        result.ok = false;
        result.message = QStringLiteral("坡口测量抛出未知异常。");
    }

    return result;
}

}  // namespace scan_tracking::vision::bevel
