#pragma once

#include <QtCore/QString>

#include "scan_tracking/vision/vision_types.h"

namespace scan_tracking::vision {

/// 生成分段海康 Mono PGM 路径（hik_mono 子目录）
QString buildSegmentHikMonoPath(
    const QString& configuredRoot,
    int segmentIndex,
    quint32 taskId,
    const QString& cameraTag,
    const QString& timestamp);

/// 将 Mono8 帧保存为 P5 PGM
bool saveHikMonoFrameToPgm(const HikMonoFrame& frame, const QString& absolutePath);

/// 释放像素缓冲，保留 width/height 等元数据
void releaseHikMonoFrameBuffers(HikMonoFrame* frame);

}  // namespace scan_tracking::vision
