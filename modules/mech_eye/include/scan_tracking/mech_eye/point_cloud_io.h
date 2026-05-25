#pragma once

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache
QString defaultScanCacheDirectory();

/// 生成分段 PLY 绝对路径（pointcloud 子目录）：segment_{N}_task{T}_{timestamp}.ply
/// @param configuredRoot config.ini scanCacheDirectory，空则默认根目录
/// @param timestamp 与同段海康图共用；空则自动生成
QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& timestamp = QString());

/// 将 PointCloudFrame 保存为 ASCII PLY（x,y,z,nx,ny,nz）
bool savePointCloudFrameToPly(const PointCloudFrame& frame, const QString& absolutePath);

/// 从 ASCII PLY 加载点云；支持 x,y,z 与 x,y,z,nx,ny,nz
bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame);

/// 释放 PointCloudFrame 中的大数组，保留 pointCount/width/height 等元数据
void releasePointCloudFrameBuffers(PointCloudFrame* frame);

}  // namespace scan_tracking::mech_eye
