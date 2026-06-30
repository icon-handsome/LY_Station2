#pragma once

#include <QtCore/QJsonObject>
#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

namespace scan_tracking {
namespace flow_control {

struct InspectionMeasurement {
    int qualityCode = 0;
};

struct InspectionResult {
    quint16 resultCode = 0;
    quint16 ngReasonWord0 = 0;
    quint16 ngReasonWord1 = 0;
    quint16 measureItemCount = 0;
    int sourcePointCount = 0;
    InspectionMeasurement measurement;
    QString message;
};

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement);

}  // namespace flow_control
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::flow_control::InspectionMeasurement)
Q_DECLARE_METATYPE(scan_tracking::flow_control::InspectionResult)
