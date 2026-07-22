#pragma once

#include <QtCore/QString>

#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

/// 默认采集缓存根目录：<applicationDir>/ScanTracking_CaptureCache
QString defaultScanCacheDirectory();

/// 生成分段 PLY 绝对路径：
/// <runRoot>/path_{pathId}/{arm|telescopic}/{segmentIndex}/cloud.ply
QString buildSegmentPlyPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// 将 PointCloudFrame 保存为 binary_little_endian PLY（仅 x,y,z；保留全部点含 NaN）
bool savePointCloudFrameToPly(const PointCloudFrame& frame, const QString& absolutePath);

/// 从 PLY 加载点云；写入仅 xyz，读取兼容带法向的旧文件
bool loadPointCloudFrameFromPly(const QString& absolutePath, PointCloudFrame* outFrame);

/// 释放 PointCloudFrame 中的大数组，保留 pointCount/width/height 等元数据
void releasePointCloudFrameBuffers(PointCloudFrame* frame);

/// 生成分段 Mech 纹理 PNG：
/// <runRoot>/path_{pathId}/{arm|telescopic}/{segmentIndex}/texture.png
QString buildSegmentMechTexturePngPath(
    const QString& configuredRoot,
    int pathId,
    const QString& deviceTag,
    int segmentIndex);

/// 将 GrayTextureFrame 保存为 8 位灰度 PNG
bool saveGrayTextureFrameToPng(const GrayTextureFrame& frame, const QString& absolutePath);

}  // namespace scan_tracking::mech_eye
