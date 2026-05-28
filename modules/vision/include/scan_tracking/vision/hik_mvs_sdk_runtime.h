#pragma once

#include <QtCore/QString>

namespace scan_tracking {
namespace vision {

/// 全局 MVS SDK 引用计数（MvCameraControl Initialize/Finalize）
bool acquireHikMvsSdk(QString* errorMessage = nullptr);
void releaseHikMvsSdk();

}  // namespace vision
}  // namespace scan_tracking
