#pragma once

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"

#include <QtCore/QVector>

#include <vector>

namespace scan_tracking::flow_control {

struct InspectionQuota {
    int expectedArmCount = 0;
    int expectedTelescopicCount = 0;
    int pathId = 0;
    QString pathName;
    QString algorithm;

    int total() const { return expectedArmCount + expectedTelescopicCount; }
};

/// 检测用轻量段云：仅有限值 XYZ + 元数据，不含完整 PointCloudFrame / 纹理 / CXP 图。
struct InspectionSegmentCloud {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int localIndex = 0;
    bool captureOk = false;
    /// length_volume：CXP 参与段必须已 LB 变换。
    bool cxpParticipated = false;
    bool lbPoseOk = false;
    std::vector<float> xyz;  ///< interleaved finite x,y,z
    int finiteCount = 0;
};

/// 后台解算 / 评估共用的点云快照（与 ScanSegmentCache 解耦，避免双持有全量 bundle）。
struct InspectionCloudSnapshot {
    quint32 runTaskId = 0;
    /// 原扫描 run 落盘根；物化临时缓存时必须复用，禁止再 mkdir 新的 run_*。
    QString runCaptureRoot;
    QVector<InspectionSegmentCloud> segments;

    void clear();
    int segmentCount() const { return segments.size(); }
    int countForDevice(common::ScanDeviceKind device) const;
    bool meetsDeviceQuotas(int expectedArmCount, int expectedTelescopicCount) const;
    const InspectionSegmentCloud* find(common::ScanDeviceKind device, int localIndex) const;
};

/// 从段缓存提取有限 XYZ；调用后主缓存可安全清空/剥离，快照独立持有数据。
InspectionCloudSnapshot buildInspectionCloudSnapshot(const ScanSegmentCache& cache);

/// 基于轻量快照执行第二工位综合检测。
/// 内部持有全局评估锁；后台工作线程可阻塞等待。GUI/主线程请用 tryEvaluateStation2Inspection。
InspectionResult evaluateStation2Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota);

/// 兼容入口：内部先 buildInspectionCloudSnapshot 再评估（HMI 同步复测等）。
InspectionResult evaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota);

/// 非阻塞评估：拿不到锁时返回 false，并写入 resultCode=3 的繁忙说明（不阻塞 GUI）。
bool tryEvaluateStation2Inspection(
    const InspectionCloudSnapshot& snapshot,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out);

bool tryEvaluateStation2Inspection(
    const ScanSegmentCache& cache,
    quint32 taskId,
    const InspectionQuota& quota,
    InspectionResult* out);

}  // namespace scan_tracking::flow_control
