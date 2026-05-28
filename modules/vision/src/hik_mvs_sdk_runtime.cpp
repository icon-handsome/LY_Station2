#include "scan_tracking/vision/hik_mvs_sdk_runtime.h"

#include <QtCore/QMutex>
#include <QtCore/QMutexLocker>

#include "MvCameraControl.h"

namespace scan_tracking {
namespace vision {

namespace {

QMutex g_sdkMutex;
int g_sdkRefCount = 0;

}  // namespace

bool acquireHikMvsSdk(QString* errorMessage)
{
    QMutexLocker locker(&g_sdkMutex);
    if (g_sdkRefCount > 0) {
        ++g_sdkRefCount;
        return true;
    }

    const int result = MV_CC_Initialize();
    if (result != MV_OK) {
        if (errorMessage != nullptr) {
            *errorMessage = QStringLiteral("MVS SDK 初始化失败，错误码=0x%1")
                                .arg(static_cast<quint32>(result), 8, 16, QLatin1Char('0'));
        }
        return false;
    }

    ++g_sdkRefCount;
    return true;
}

void releaseHikMvsSdk()
{
    QMutexLocker locker(&g_sdkMutex);
    if (g_sdkRefCount <= 0) {
        return;
    }
    --g_sdkRefCount;
    if (g_sdkRefCount == 0) {
        MV_CC_Finalize();
    }
}

}  // namespace vision
}  // namespace scan_tracking
