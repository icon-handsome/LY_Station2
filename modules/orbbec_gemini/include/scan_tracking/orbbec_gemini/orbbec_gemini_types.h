#pragma once

#include <QtCore/QMetaType>
#include <QtCore/QString>
#include <QtCore/QVector>

namespace scan_tracking {
namespace orbbec_gemini {

enum class OrbbecGeminiRuntimeState {
    Idle = 0,
    Enumerating,
    Opening,
    Ready,
    Failed,
    Stopped,
};

struct OrbbecGeminiOpenConfig {
    QString serial;
    int deviceIndex = 0;
};

struct OrbbecGeminiDeviceSummary {
    int index = -1;
    QString name;
    QString serialNumber;
    QString firmwareVersion;
    QString connectionType;
    quint16 pid = 0;
    quint16 vid = 0;
    QString uid;
};

}  // namespace orbbec_gemini
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig)
Q_DECLARE_METATYPE(scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary)
Q_DECLARE_METATYPE(QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary>)
