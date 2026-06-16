#include "scan_tracking/flow_control/inspection_types.h"

namespace scan_tracking {
namespace flow_control {

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement)
{
    QJsonObject headMetrics;
    headMetrics[QStringLiteral("qualityCode")] = measurement.qualityCode;
    payload[QStringLiteral("headMetrics")] = headMetrics;
}

}  // namespace flow_control
}  // namespace scan_tracking
