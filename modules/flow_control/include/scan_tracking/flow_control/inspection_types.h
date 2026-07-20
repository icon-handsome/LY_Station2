#pragma once

#include <QtCore/QJsonObject>
#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QtGlobal>

namespace scan_tracking {
namespace flow_control {

struct InspectionMeasurement {
    int qualityCode = 0;
    // weld_section
    double mismatchMm = 0.0;
    double reinforcementMm = 0.0;
    double angularityMm = 0.0;
    double includedAngleDeg = 0.0;
    double leftUndercutMm = 0.0;
    double rightUndercutMm = 0.0;
    double maxUndercutMm = 0.0;
    int measuredSegmentCount = 0;
    // thickness_inner_surface
    double thicknessMm = 0.0;
    int thicknessPairCount = 0;
    int thicknessSuccessCount = 0;
    double innerDiameterMm = 0.0;
    double innerCircumferenceMm = 0.0;
    double innerRoundness = 0.0;
    int innerSurfacePairCount = 0;
    int innerSurfaceSuccessCount = 0;
    // length_volume
    double lengthMm = 0.0;
    double volumeLiters = 0.0;
    double volumeRadiusMm = 0.0;
    double fittedOuterRadiusMm = 0.0;
    QString codeValue;
};

struct InspectionResult {
    quint16 resultCode = 0;
    quint16 ngReasonWord0 = 0;
    quint16 ngReasonWord1 = 0;
    quint16 measureItemCount = 0;
    int sourcePointCount = 0;
    int pathId = 0;
    QString pathName;
    QString algorithm;
    InspectionMeasurement measurement;
    QString message;
};

void appendInspectionMeasurementFields(QJsonObject& payload, const InspectionMeasurement& measurement);

}  // namespace flow_control
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::flow_control::InspectionMeasurement)
Q_DECLARE_METATYPE(scan_tracking::flow_control::InspectionResult)
