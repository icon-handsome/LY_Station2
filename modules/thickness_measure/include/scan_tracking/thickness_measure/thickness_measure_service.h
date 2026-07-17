#pragma once

#include <cstddef>

#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/thickness_measure/thickness_measure_types.h"

namespace scan_tracking::thickness_measure {

/// Thin façade over ThicknessMeasure.dll (C API). Calls are serialized per instance.
class ThicknessMeasureService {
public:
    ThicknessMeasureService();
    ~ThicknessMeasureService();

    ThicknessMeasureService(const ThicknessMeasureService&) = delete;
    ThicknessMeasureService& operator=(const ThicknessMeasureService&) = delete;

    /// Default config: <applicationDir>/config/thickness_measure/thickness_config.json
    static QString defaultConfigPath();

    bool isReady() const;
    QString configPath() const;

    /// Load templates + params from thickness_config.json (scan paths in JSON ignored).
    bool initializeFromJson(const QString& configPath = QString(), ThicknessMeasureError* error = nullptr);

    /// Create context from in-memory templates (interleaved xyz floats).
    bool initialize(
        const ThicknessConfig& config,
        const float* innerTemplateXyz,
        size_t innerTemplateCount,
        const float* outerTemplateXyz,
        size_t outerTemplateCount,
        ThicknessMeasureError* error = nullptr);

    void shutdown();

    bool measurePair(
        const float* innerScanXyz,
        size_t innerScanCount,
        const float* outerScanXyz,
        size_t outerScanCount,
        ThicknessPairMeasurement* out,
        ThicknessMeasureError* error = nullptr);

    bool measurePairsAverage(
        const QVector<ThicknessPairClouds>& pairs,
        ThicknessAverageMeasurement* out,
        ThicknessMeasureError* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::thickness_measure
