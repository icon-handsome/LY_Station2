#pragma once

#include <QtCore/QString>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::mech_eye {

struct PointCloudProcessReport {
    int inputPointCount = 0;
    int outputPointCount = 0;
    QString message;
};

/// 对 Mech-Eye 点云执行可配置后处理；enabled=false 时深拷贝直通。
bool processPointCloudFrame(
    const PointCloudFrame& input,
    const common::PointCloudProcessingConfig& config,
    PointCloudFrame* output,
    PointCloudProcessReport* report = nullptr);

}  // namespace scan_tracking::mech_eye
