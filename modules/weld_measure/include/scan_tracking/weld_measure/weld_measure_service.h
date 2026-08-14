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

    /// Default: <applicationDir>/config/weld_measure/weld_measurement.ini
    static QString defaultConfigPath();
    /// Arm straight-weld ini (11 frames).
    static QString defaultArmConfigPath();
    /// Telescopic straight-weld ini (7 frames).
    static QString defaultTelescopicConfigPath();
    /// Arm ring-weld ini (18 frames).
    static QString defaultRingArmConfigPath();
    /// Telescopic ring-weld ini (18 frames).
    static QString defaultRingTelescopicConfigPath();
    /// Default model: <applicationDir>/models/weld_measure/pointnet_weld_seam_V7.3_good.onnx
    static QString defaultModelPath();

    bool isReady() const;
    QString configPath() const;
    QString modelPath() const;

    /// Preferred: load ONNX + Frame/ICP/Undercut via wm_create_from_ini.
    bool initializeFromIni(const QString& configPath = QString(), WeldMeasureError* error = nullptr);

    /// Legacy/smoke: create context from ONNX model path only.
    bool initialize(const QString& modelPath = QString(), WeldMeasureError* error = nullptr);

    void shutdown();

    /// Formal V2.2 frame pipeline: optional ROI -> downsample -> ICP -> multi-section -> average.
    /// @param frameIndex1Based matches Frame1..N in the loaded ini.
    bool measureFrame(
        int frameIndex1Based,
        const float* xyz,
        size_t pointCount,
        WeldFrameMeasurement* out,
        WeldMeasureError* error = nullptr);

    /// Legacy: measure one already-cut section cloud.
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
