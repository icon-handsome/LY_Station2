#pragma once

#include <QMap>
#include <QString>
#include <QVector>

#include <QtCore/QtGlobal>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking::flow_control {

struct ScanSegmentCacheKey {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int localIndex = 0;

    bool operator<(const ScanSegmentCacheKey& other) const
    {
        if (device != other.device) {
            return static_cast<int>(device) < static_cast<int>(other.device);
        }
        return localIndex < other.localIndex;
    }

    bool operator==(const ScanSegmentCacheKey& other) const
    {
        return device == other.device && localIndex == other.localIndex;
    }
};

struct ScanSegmentCacheEntry {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int segmentIndex = 0;  ///< 设备内本地段号
    quint32 taskId = 0;
    QString runCaptureRoot;
    QString captureTimestamp;
    vision::MultiCameraCaptureBundle bundle;
};

class ScanSegmentCache {
public:
    void reset();

    /// 为新 taskId 准备 run 目录；taskId 变化时会先 reset。
    bool ensureRunRoot(quint32 taskId, QString* runRootOut = nullptr, QString* timestampOut = nullptr);

    void storeSegment(
        common::ScanDeviceKind device,
        int localIndex,
        quint32 taskId,
        vision::MultiCameraCaptureBundle bundle);

    bool persistSegment(
        common::ScanDeviceKind device,
        int localIndex,
        QString* errorMessage = nullptr);

    const ScanSegmentCacheEntry* entry(common::ScanDeviceKind device, int localIndex) const;
    QVector<ScanSegmentCacheKey> cachedKeys() const;
    bool allCachedBundlesSuccessful() const;
    int cachedSegmentCount() const;
    int cachedCountForDevice(common::ScanDeviceKind device) const;
    bool meetsDeviceQuotas(int expectedArmCount, int expectedTelescopicCount) const;
    quint32 runTaskId() const { return m_runTaskId; }
    const QString& runCaptureRoot() const { return m_runCaptureRoot; }

private:
    QMap<ScanSegmentCacheKey, ScanSegmentCacheEntry> m_entries;
    quint32 m_runTaskId = 0;
    QString m_runCaptureRoot;
    QString m_runTimestamp;
};

bool persistScanSegmentBundle(
    const QString& runRoot,
    common::ScanDeviceKind device,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp,
    const vision::MultiCameraCaptureBundle& bundle,
    QString* errorMessage = nullptr);

}  // namespace scan_tracking::flow_control
