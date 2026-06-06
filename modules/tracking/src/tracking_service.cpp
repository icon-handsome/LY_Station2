#include "scan_tracking/tracking/tracking_service.h"

#include <cmath>

#include <QtCore/QJsonObject>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMetaType>

#include "scan_tracking/common/application_info.h"
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/tracking/lb_pose_check.h"
#include "scan_tracking/vision/bevel_measurement_adapter.h"
#ifdef SCAN_TRACKING_HAS_HOLE_MEASUREMENT
#include "scan_tracking/vision/hole_measurement_adapter.h"
#endif

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
    if (measurement.algorithm == InspectionAlgorithm::Hole) {
        if (std::isfinite(measurement.innerDiameterMm) && measurement.innerDiameterMm > 0.0f) {
            ++count;
        }
        if (std::isfinite(measurement.roundnessToleranceMm)) {
            ++count;
        }
        if (std::isfinite(measurement.straightSideHeightMm)) {
            ++count;
        }
        if (std::isfinite(measurement.holeOpeningMm)) {
            ++count;
        }
        return count;
    }

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
    measurement.algorithm = InspectionAlgorithm::Bevel;
    measurement.headAngleTol = detection.angleDeg;
    measurement.bluntHeightTol = detection.lengthMm;
    measurement.bevelType = detection.bevelType;
    measurement.icpFitness = detection.icpFitness;
    measurement.qualityCode = detection.qualityCode;
    return measurement;
}

#ifdef SCAN_TRACKING_HAS_HOLE_MEASUREMENT
InspectionMeasurement measurementFromHoleResult(
    const scan_tracking::vision::hole::HoleInspectionResult& detection)
{
    InspectionMeasurement measurement;
    measurement.algorithm = InspectionAlgorithm::Hole;
    measurement.innerDiameterMm = static_cast<float>(detection.measureResult.innerDiameterMm);
    measurement.innerCircumferenceMm =
        static_cast<float>(detection.measureResult.innerCircumferenceMm);
    measurement.roundnessToleranceMm =
        static_cast<float>(detection.measureResult.roundnessToleranceMm);
    measurement.straightSideSlopeDeg =
        static_cast<float>(detection.measureResult.straightSideSlopeDeg);
    measurement.straightSideHeightMm =
        static_cast<float>(detection.measureResult.straightSideHeightMm);
    measurement.holeOpeningMm =
        static_cast<float>(detection.measureResult.opening.centerToInnerWallDistanceMm);
    measurement.jointFitUpAngleDeg =
        static_cast<float>(detection.measureResult.opening.axisToHeadAxisAngleDeg);
    measurement.icpFitness = static_cast<float>(detection.icpRmsMm);
    measurement.qualityCode = detection.ok ? 0 : 1;
    return measurement;
}
#endif

scan_tracking::common::InspectionType resolveInspectionType(
    const scan_tracking::common::ConfigManager* configManager,
    int inspectionPathId)
{
    if (configManager == nullptr) {
        return scan_tracking::common::InspectionType::Bevel;
    }
    if (inspectionPathId > 0) {
        return configManager->inspectionTypeForPath(inspectionPathId);
    }
    return scan_tracking::common::InspectionType::Bevel;
}

}  // namespace

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    payload[QStringLiteral("inspection_algorithm")] =
        measurement.algorithm == InspectionAlgorithm::Hole ? QStringLiteral("hole")
                                                           : QStringLiteral("bevel");
    payload[QStringLiteral("head_angle_tol")] = measurementJsonValue(measurement.headAngleTol);
    payload[QStringLiteral("blunt_height_tol")] = measurementJsonValue(measurement.bluntHeightTol);
    payload[QStringLiteral("bevel_type")] = measurement.bevelType;
    payload[QStringLiteral("icp_fitness")] = measurementJsonValue(measurement.icpFitness);
    payload[QStringLiteral("quality_code")] = measurement.qualityCode;
    payload[QStringLiteral("inner_diameter_mm")] = measurementJsonValue(measurement.innerDiameterMm);
    payload[QStringLiteral("inner_circumference_mm")] =
        measurementJsonValue(measurement.innerCircumferenceMm);
    payload[QStringLiteral("roundness_tolerance_mm")] =
        measurementJsonValue(measurement.roundnessToleranceMm);
    payload[QStringLiteral("straight_side_slope_deg")] =
        measurementJsonValue(measurement.straightSideSlopeDeg);
    payload[QStringLiteral("straight_side_height_mm")] =
        measurementJsonValue(measurement.straightSideHeightMm);
    payload[QStringLiteral("hole_opening_mm")] = measurementJsonValue(measurement.holeOpeningMm);
    payload[QStringLiteral("joint_fit_up_angle_deg")] =
        measurementJsonValue(measurement.jointFitUpAngleDeg);
}

void appendHeadDisplayMetricsFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    QJsonObject headMetrics;
    headMetrics[QStringLiteral("inner_diameter_mm")] =
        measurementJsonValue(measurement.innerDiameterMm);
    headMetrics[QStringLiteral("roundness_tol")] =
        measurementJsonValue(measurement.roundnessToleranceMm);
    headMetrics[QStringLiteral("straight_slope_tol")] =
        measurementJsonValue(measurement.straightSideSlopeDeg);
    headMetrics[QStringLiteral("head_depth_mm")] = 0.0;
    headMetrics[QStringLiteral("straight_height_tol")] =
        measurementJsonValue(measurement.straightSideHeightMm);
    headMetrics[QStringLiteral("bevel_angle_deg")] = measurementJsonValue(measurement.headAngleTol);
    headMetrics[QStringLiteral("blunt_height_mm")] = measurementJsonValue(measurement.bluntHeightTol);
    headMetrics[QStringLiteral("inner_circumference_mm")] =
        measurementJsonValue(measurement.innerCircumferenceMm);
    headMetrics[QStringLiteral("hole_opening_mm")] = measurementJsonValue(measurement.holeOpeningMm);
    headMetrics[QStringLiteral("joint_fit_up_angle_deg")] =
        measurementJsonValue(measurement.jointFitUpAngleDeg);
    headMetrics[QStringLiteral("thickness_mm")] = 0.0;
    headMetrics[QStringLiteral("head_volume_m3")] = 0.0;
    payload[QStringLiteral("headMetrics")] = headMetrics;
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
    int inspectionPathId,
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

    const auto* configManager = scan_tracking::common::ConfigManager::instance();
    const scan_tracking::common::InspectionType inspectionType =
        resolveInspectionType(configManager, inspectionPathId);

#ifdef SCAN_TRACKING_HAS_HOLE_MEASUREMENT
    if (inspectionType == scan_tracking::common::InspectionType::Hole) {
        const auto detection =
            scan_tracking::vision::hole::runHoleMeasurement(pointCloud, inspectionPathId);

        if (!detection.invoked) {
            result.resultCode = 2;
            result.ngReasonWord0 = (1u << 4);
            result.message = detection.message.isEmpty()
                ? QStringLiteral("Hole 测量适配层未启动。")
                : detection.message;
            return deliverInspectionResult(result, notifyListener);
        }

        result.measurement = measurementFromHoleResult(detection);

        if (!detection.ok) {
            result.resultCode = 2;
            result.ngReasonWord0 = (1u << 5);
            result.message = detection.message.isEmpty()
                ? QStringLiteral("Hole 测量算法失败。")
                : detection.message;
            return deliverInspectionResult(result, notifyListener);
        }

        result.resultCode = 1;
        result.measureItemCount = countMeasuredItems(result.measurement);
        result.message = detection.message;
        return deliverInspectionResult(result, notifyListener);
    }
#else
    if (inspectionType == scan_tracking::common::InspectionType::Hole) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("Hole 测量未编译（SCAN_TRACKING_ENABLE_HOLE_MEASUREMENT=OFF）。");
        return deliverInspectionResult(result, notifyListener);
    }
#endif

    if (configManager == nullptr || !configManager->hasActiveBevelRecipe()) {
        result.resultCode = 2;
        result.ngReasonWord0 = (1u << 4);
        result.message = QStringLiteral("请先通过 HMI 设置坡口配方（cmd.set_bevel_recipe）。");
        return deliverInspectionResult(result, notifyListener);
    }

    const scan_tracking::common::BevelRecipe recipe = configManager->bevelRecipe();
    const scan_tracking::common::BevelConfig& bevelConfig = configManager->bevelConfig();
    const auto detection = scan_tracking::vision::bevel::runBevelMeasurement(
        pointCloud, recipe, bevelConfig.angleTolDeg, bevelConfig.lengthTolMm);

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
