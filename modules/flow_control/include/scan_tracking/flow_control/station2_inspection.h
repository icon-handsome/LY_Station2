#pragma once

#include "scan_tracking/flow_control/inspection_types.h"
#include "scan_tracking/flow_control/scan_segment_cache.h"

#include <QtCore/QVector>

#include <memory>
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

/// 检测用轻量段云：共享采集点云的 XYZ 缓冲 + 元数据，不含纹理 / 法向 / CXP 图。
/// 后续算法逐帧提取有限值，避免路径齐套时为全部段深拷贝近 GB 级点云。
struct InspectionSegmentCloud {
    common::ScanDeviceKind device = common::ScanDeviceKind::Arm;
    int localIndex = 0;
    bool captureOk = false;
    /// length_volume：CXP 参与段必须已 LB 变换。
    bool cxpParticipated = false;
    bool lbPoseOk = false;
    std::shared_ptr<std::vector<float>> xyz;  ///< shared interleaved x,y,z
    int pointCount = 0;
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

/// 启动期预热当前路径算法上下文；当前仅对 weld_section 生效。
/// 应在累计大点云前调用，避免路径齐套后才创建 ONNX 会话。
bool prewarmActiveStation2InspectionAlgorithm(QString* errorMessage = nullptr);

/// 从段缓存共享 XYZ 缓冲；调用后主缓存可安全清空，快照通过 shared_ptr 保持数据存活。
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
