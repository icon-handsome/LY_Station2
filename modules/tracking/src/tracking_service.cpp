/**
 * @file tracking_service.cpp
 * @brief 跟踪服务实现文件
 *
 * 实现跟踪服务类，提供坡口测量综合检测和位姿校验功能。
 */

#include "scan_tracking/tracking/tracking_service.h"

#include <cmath>

#include <QtCore/QJsonObject>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaType>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/lb_pose_check.h"
#include "scan_tracking/vision/bevel_measurement_adapter.h"

namespace scan_tracking::tracking {

Q_LOGGING_CATEGORY(LOG_TRACKING, "tracking")

namespace {

void ensureInspectionMeasurementMetaTypeRegistered()
{
    static const bool registered = []() {
        qRegisterMetaType<InspectionMeasurement>(
            "scan_tracking::tracking::InspectionMeasurement");
        return true;
    }();
    Q_UNUSED(registered);
}

double measurementJsonValue(float value)
{
    return std::isfinite(value) ? static_cast<double>(value) : 0.0;
}

quint16 countMeasuredItems(const InspectionMeasurement& measurement)
{
    quint16 count = 0;
    if (std::isfinite(measurement.headAngleTol)) {
        ++count;
    }
    if (std::isfinite(measurement.bluntHeightTol)) {
        ++count;
    }
    return count;
}

InspectionMeasurement measurementFromBevelResult(
    const scan_tracking::vision::bevel::BevelInspectionResult& detection)
{
    InspectionMeasurement measurement;
    measurement.headAngleTol = detection.angleDeg;
    measurement.bluntHeightTol = detection.lengthMm;
    measurement.bevelType = detection.bevelType;
    measurement.icpFitness = detection.icpFitness;
    measurement.qualityCode = detection.qualityCode;
    return measurement;
}

}  // namespace

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    payload[QStringLiteral("head_angle_tol")] = measurementJsonValue(measurement.headAngleTol);
    payload[QStringLiteral("blunt_height_tol")] = measurementJsonValue(measurement.bluntHeightTol);
    payload[QStringLiteral("bevel_type")] = measurement.bevelType;
    payload[QStringLiteral("icp_fitness")] = measurementJsonValue(measurement.icpFitness);
    payload[QStringLiteral("quality_code")] = measurement.qualityCode;
}

std::string TrackingService::statusText() const
{
    return scan_tracking::common::ApplicationInfo::name() + " core is ready.";
}

void TrackingService::setInspectionResultNotifier(InspectionResultNotifier notifier)
{
    m_inspectionResultNotifier = std::move(notifier);
}

InspectionResult TrackingService::deliverInspectionResult(
    InspectionResult result, bool notifyListener) const
{
    if (notifyListener && m_inspectionResultNotifier) {
        m_inspectionResultNotifier(result);
    }
    return result;
}

InspectionResult TrackingService::inspectPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& pointCloud,
    int sourcePointCount,
    bool notifyListener) const
{
    ensureInspectionMeasurementMetaTypeRegistered();

    InspectionResult result;
    result.sourcePointCount = sourcePointCount > 0 ? sourcePointCount : pointCloud.pointCount;

    if (!pointCloud.isValid() || result.sourcePointCount <= 0) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("综合检测没有可用点云。");
        return deliverInspectionResult(result, notifyListener);
    }

    const auto detection = scan_tracking::vision::bevel::runBevelMeasurement(pointCloud);

    if (!detection.invoked) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("坡口测量适配层未启动。")
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    result.measurement = measurementFromBevelResult(detection);

    if (!detection.ok) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 5);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("坡口测量算法失败。")
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    if (detection.qualityCode != 0) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 6);
        result.message = detection.message.isEmpty()
            ? QStringLiteral("坡口测量超出标准范围，qualityCode=%1。")
                  .arg(detection.qualityCode)
            : detection.message;
        return deliverInspectionResult(result, notifyListener);
    }

    result.resultCode = 1;
    result.measureItemCount = countMeasuredItems(result.measurement);
    result.message = QStringLiteral(
        "坡口测量通过：angle=%1 deg, length=%2 mm, bevelType=%3, icpFitness=%4。")
                         .arg(detection.angleDeg, 0, 'f', 3)
                         .arg(detection.lengthMm, 0, 'f', 3)
                         .arg(detection.bevelType)
                         .arg(detection.icpFitness, 0, 'f', 6);
    return deliverInspectionResult(result, notifyListener);
}

PoseCheckResult TrackingService::checkPose() const
{
    PoseCheckResult result;
    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    if (configManager == nullptr) {
        result.resultCode = 7;
        result.message = QStringLiteral("LB 位姿检查时 ConfigManager 不可用。");
        return result;
    }

    const auto lbResult = runLegacyLbPoseCheck(configManager->lbPoseConfig());
    result.invoked = lbResult.invoked;
    result.success = lbResult.success;
    result.resultCode = lbResult.resultCode;
    result.inputPointCount = lbResult.inputPointCount;
    result.poseDeviationMm = lbResult.poseDeviationMm;
    result.rt = lbResult.rt;
    result.message = lbResult.message;
    if (result.success && result.hasPoseMatrix()) {
        result.message += QStringLiteral(" Pose matrix is ready for downstream use.");
    }
    return result;
}

}  // namespace scan_tracking::tracking
