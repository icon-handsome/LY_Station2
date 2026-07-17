#include "scan_tracking/flow_control/inspection_types.h"

namespace scan_tracking {
namespace flow_control {

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    QJsonObject headMetrics;
    headMetrics[QStringLiteral("qualityCode")] = measurement.qualityCode;
    headMetrics[QStringLiteral("mismatchMm")] = measurement.mismatchMm;
    headMetrics[QStringLiteral("reinforcementMm")] = measurement.reinforcementMm;
    headMetrics[QStringLiteral("angularityMm")] = measurement.angularityMm;
    headMetrics[QStringLiteral("includedAngleDeg")] = measurement.includedAngleDeg;
    headMetrics[QStringLiteral("leftUndercutMm")] = measurement.leftUndercutMm;
    headMetrics[QStringLiteral("rightUndercutMm")] = measurement.rightUndercutMm;
    headMetrics[QStringLiteral("maxUndercutMm")] = measurement.maxUndercutMm;
    headMetrics[QStringLiteral("measuredSegmentCount")] = measurement.measuredSegmentCount;
    headMetrics[QStringLiteral("thicknessMm")] = measurement.thicknessMm;
    headMetrics[QStringLiteral("thickness_mm")] = measurement.thicknessMm;
    headMetrics[QStringLiteral("thicknessPairCount")] = measurement.thicknessPairCount;
    headMetrics[QStringLiteral("thicknessSuccessCount")] = measurement.thicknessSuccessCount;
    headMetrics[QStringLiteral("innerDiameterMm")] = measurement.innerDiameterMm;
    headMetrics[QStringLiteral("inner_diameter_mm")] = measurement.innerDiameterMm;
    headMetrics[QStringLiteral("innerCircumferenceMm")] = measurement.innerCircumferenceMm;
    headMetrics[QStringLiteral("inner_circumference_mm")] = measurement.innerCircumferenceMm;
    headMetrics[QStringLiteral("innerRoundness")] = measurement.innerRoundness;
    headMetrics[QStringLiteral("roundness_tol")] = measurement.innerRoundness;
    headMetrics[QStringLiteral("innerSurfacePairCount")] = measurement.innerSurfacePairCount;
    headMetrics[QStringLiteral("innerSurfaceSuccessCount")] = measurement.innerSurfaceSuccessCount;
    if (!measurement.codeValue.isEmpty()) {
        headMetrics[QStringLiteral("codeValue")] = measurement.codeValue;
    }
    payload[QStringLiteral("headMetrics")] = headMetrics;
}

}  // namespace flow_control
}  // namespace scan_tracking
