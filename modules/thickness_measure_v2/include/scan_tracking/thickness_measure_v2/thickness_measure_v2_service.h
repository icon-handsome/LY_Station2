#pragma once

#include <cstddef>

#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/thickness_measure_v2/thickness_measure_v2_types.h"

namespace scan_tracking::thickness_measure_v2 {

/// Thin façade over ThicknessMeasureV3 via an out-of-process worker.
/// Host never loads ThicknessMeasureV3.dll; create/measure/destroy run in
/// thickness-measure-v3-worker.exe. Calls are serialized per instance.
class ThicknessMeasureV2Service {
public:
    ThicknessMeasureV2Service();
    ~ThicknessMeasureV2Service();

    ThicknessMeasureV2Service(const ThicknessMeasureV2Service&) = delete;
    ThicknessMeasureV2Service& operator=(const ThicknessMeasureV2Service&) = delete;

    /// Default: <applicationDir>/config/thickness_measure_v3/thickness_measurement.ini
    static QString defaultConfigPath();

    bool isReady() const;
    QString configPath() const;

    bool initializeFromIni(const QString& configPath = QString(), ThicknessV2Error* error = nullptr);
    void shutdown();

    bool measurePair(
        const float* innerScanXyz,
        size_t innerScanCount,
        const float* outerScanXyz,
        size_t outerScanCount,
        ThicknessV2PairMeasurement* out,
        ThicknessV2Error* error = nullptr);

    bool measurePairsAverage(
        const QVector<ThicknessV2PairClouds>& pairs,
        ThicknessV2AverageMeasurement* out,
        ThicknessV2Error* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::thickness_measure_v2
