#pragma once

#include <cstddef>

#include <QtCore/QString>

#include "scan_tracking/length_volume_measure/length_volume_measure_types.h"

namespace scan_tracking::length_volume_measure {

/// Thin façade over LengthVolumeMeasure.dll (C API). Calls are serialized per instance.
class LengthVolumeMeasureService {
public:
    LengthVolumeMeasureService();
    ~LengthVolumeMeasureService();

    LengthVolumeMeasureService(const LengthVolumeMeasureService&) = delete;
    LengthVolumeMeasureService& operator=(const LengthVolumeMeasureService&) = delete;

    /// Default config: <applicationDir>/config/length_volume_measure/config.ini
    static QString defaultConfigPath();

    bool isReady() const;
    QString configPath() const;

    /// Load template + params from config.ini (inputCloud ignored; scans come from memory APIs).
    bool initializeFromIni(const QString& configPath = QString(), LengthVolumeMeasureError* error = nullptr);

    /// Create context from in-memory template (interleaved xyz floats).
    bool initialize(
        const LengthVolumeConfig& config,
        const float* templateXyz,
        size_t templateCount,
        LengthVolumeMeasureError* error = nullptr);

    void shutdown();

    /// volumeRadiusMm is typically half of inner-surface diameter (mm).
    bool measure(
        const float* scanXyz,
        size_t scanCount,
        double volumeRadiusMm,
        LengthVolumeMeasurement* out,
        LengthVolumeMeasureError* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::length_volume_measure
