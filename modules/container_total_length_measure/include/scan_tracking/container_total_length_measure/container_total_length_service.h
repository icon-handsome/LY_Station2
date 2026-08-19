#pragma once

#include <cstddef>

#include <QtCore/QString>

#include "scan_tracking/container_total_length_measure/container_total_length_types.h"

namespace scan_tracking::container_total_length_measure {

/// Thin façade over ContainerTotalLength.dll (C API). Calls are serialized per instance.
class ContainerTotalLengthService {
public:
    ContainerTotalLengthService();
    ~ContainerTotalLengthService();

    ContainerTotalLengthService(const ContainerTotalLengthService&) = delete;
    ContainerTotalLengthService& operator=(const ContainerTotalLengthService&) = delete;

    /// Default config: <applicationDir>/config/container_total_length/config.ini
    static QString defaultConfigPath();

    bool isReady() const;
    QString configPath() const;

    /// Load template + params from config.ini (inputCloud ignored; scans come from memory APIs).
    bool initializeFromIni(const QString& configPath = QString(), ContainerTotalLengthError* error = nullptr);

    void shutdown();

    bool measure(
        const float* scanXyz,
        size_t scanCount,
        ContainerTotalLengthMeasurement* out,
        ContainerTotalLengthError* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::container_total_length_measure
