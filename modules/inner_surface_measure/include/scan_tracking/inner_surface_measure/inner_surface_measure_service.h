#pragma once

#include <cstddef>

#include <QtCore/QString>

#include "scan_tracking/inner_surface_measure/inner_surface_measure_types.h"

namespace scan_tracking::inner_surface_measure {

/// Thin façade over InnerSurfaceMeasure.dll (C API). Calls are serialized per instance.
class InnerSurfaceMeasureService {
public:
    InnerSurfaceMeasureService();
    ~InnerSurfaceMeasureService();

    InnerSurfaceMeasureService(const InnerSurfaceMeasureService&) = delete;
    InnerSurfaceMeasureService& operator=(const InnerSurfaceMeasureService&) = delete;

    /// Default config: <applicationDir>/config/inner_surface_measure/config.ini
    static QString defaultConfigPath();

    bool isReady() const;
    QString configPath() const;

    /// Load template + params from config.ini (InputFrames ignored; scans come from memory APIs).
    bool initializeFromIni(const QString& configPath = QString(), InnerSurfaceMeasureError* error = nullptr);

    /// Create context from in-memory template (interleaved xyz floats).
    bool initialize(
        const InnerSurfaceConfig& config,
        const float* templateXyz,
        size_t templateCount,
        InnerSurfaceMeasureError* error = nullptr);

    void shutdown();

    bool measureFrame(
        const float* scanXyz,
        size_t scanCount,
        InnerSurfaceFrameMeasurement* out,
        InnerSurfaceMeasureError* error = nullptr);

    bool measureTwoFramesAverage(
        const float* frame1Xyz,
        size_t frame1Count,
        const float* frame2Xyz,
        size_t frame2Count,
        InnerSurfaceAverageMeasurement* outAverage,
        InnerSurfaceFrameMeasurement* outFrame1 = nullptr,
        InnerSurfaceFrameMeasurement* outFrame2 = nullptr,
        InnerSurfaceMeasureError* error = nullptr);

    /// Uses measuredLengthMm for volume when it is positive; otherwise config fallback is used.
    bool measureTwoFramesAverageWithLength(
        const float* frame1Xyz,
        size_t frame1Count,
        const float* frame2Xyz,
        size_t frame2Count,
        double measuredLengthMm,
        InnerSurfaceAverageMeasurement* outAverage,
        InnerSurfaceFrameMeasurement* outFrame1 = nullptr,
        InnerSurfaceFrameMeasurement* outFrame2 = nullptr,
        InnerSurfaceMeasureError* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::inner_surface_measure
