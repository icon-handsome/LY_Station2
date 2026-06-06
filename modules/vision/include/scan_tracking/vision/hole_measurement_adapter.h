#pragma once

// HeadMeasure 柱面/开孔测量算法适配层。
//
// 将 Mech-Eye 点云帧转换为 PCL 格式，调用 third_party/Hole MeasurePipeline，
// 输出内径、圆度、直边、开孔等尺寸，供 TrackingService / StateMachine 写寄存器与 HMI 上报。

#include <QtCore/QString>

#include "HeadMeasure/Types.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"

namespace scan_tracking::vision::hole {

struct HoleInspectionResult {
    bool invoked = false;
    bool ok = false;
    QString message;
    hm::MeasureResult measureResult;
    double icpRmsMm = 0.0;
    double cylinderRmsMm = 0.0;
};

QString resolveHoleConfigPath(int inspectionPathId = 0);

HoleInspectionResult runHoleMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud,
    int inspectionPathId = 0);

}  // namespace scan_tracking::vision::hole
