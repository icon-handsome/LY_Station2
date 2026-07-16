#pragma once

#include <cstddef>

#include <QtCore/QString>

#include "scan_tracking/weld_measure/weld_measure_types.h"

namespace scan_tracking::weld_measure {

/// Thin façade over WeldMeasure.dll (C API). Calls are serialized per instance.
class WeldMeasureService {
public:
    WeldMeasureService();
    ~WeldMeasureService();

    WeldMeasureService(const WeldMeasureService&) = delete;
    WeldMeasureService& operator=(const WeldMeasureService&) = delete;

    /// Default model: <applicationDir>/models/weld_measure/pointnet_weld_seam_V7.3_good.onnx
    static QString defaultModelPath();

    bool isReady() const;
    QString modelPath() const;

    bool initialize(const QString& modelPath = QString(), WeldMeasureError* error = nullptr);
    void shutdown();

    /// @param xyz Interleaved x,y,z; non-finite points are skipped inside the DLL.
    bool measureSection(
        const float* xyz,
        size_t pointCount,
        WeldSectionMeasurement* out,
        WeldMeasureError* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::weld_measure
