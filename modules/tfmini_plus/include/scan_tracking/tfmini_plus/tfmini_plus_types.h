#pragma once

#include <QtCore/QMetaType>
#include <QtCore/QString>

namespace scan_tracking {
namespace tfmini_plus {

enum class TfminiPlusRuntimeState {
    Idle = 0,
    Opening,
    Ready,
    Failed,
    Stopped,
};

struct TfminiPlusOpenConfig {
    QString portName;
    int baudRate = 115200;
};

}  // namespace tfmini_plus
}  // namespace scan_tracking

Q_DECLARE_METATYPE(scan_tracking::tfmini_plus::TfminiPlusRuntimeState)
Q_DECLARE_METATYPE(scan_tracking::tfmini_plus::TfminiPlusOpenConfig)
