#pragma once

#include <QMap>
#include <QString>

#include <QtCore/QtGlobal>

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking::flow_control {

struct ScanSegmentCacheEntry {
    int segmentIndex = 0;
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

    void storeSegment(int segmentIndex, quint32 taskId, vision::MultiCameraCaptureBundle bundle);

    bool persistSegment(int segmentIndex, QString* errorMessage = nullptr);

    const ScanSegmentCacheEntry* entry(int segmentIndex) const;
    QVector<int> cachedSegmentIndices() const;
    bool allCachedBundlesSuccessful() const;
    int cachedSegmentCount() const;
    quint32 runTaskId() const { return m_runTaskId; }
    const QString& runCaptureRoot() const { return m_runCaptureRoot; }

private:
    QMap<int, ScanSegmentCacheEntry> m_entries;
    quint32 m_runTaskId = 0;
    QString m_runCaptureRoot;
    QString m_runTimestamp;
};

bool persistScanSegmentBundle(
    const QString& runRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp,
    const vision::MultiCameraCaptureBundle& bundle,
    QString* errorMessage = nullptr);

}  // namespace scan_tracking::flow_control
