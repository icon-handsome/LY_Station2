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
    payload[QStringLiteral("headMetrics")] = headMetrics;
}

}  // namespace flow_control
}  // namespace scan_tracking
