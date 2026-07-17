#pragma once

#include <cstddef>

#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/undercut_length_measure/undercut_length_measure_types.h"

namespace scan_tracking::undercut_length_measure {

/// Thin façade over UndercutLengthMeasure.dll (C API). Calls are serialized per instance.
class UndercutLengthMeasureService {
public:
    UndercutLengthMeasureService();
    ~UndercutLengthMeasureService();

    UndercutLengthMeasureService(const UndercutLengthMeasureService&) = delete;
    UndercutLengthMeasureService& operator=(const UndercutLengthMeasureService&) = delete;

    /// Default config: <applicationDir>/config/undercut_length_measure/undercut_length.ini
    static QString defaultConfigPath();

    bool isReady() const;
    QString configPath() const;

    /// Create context with in-memory options (preferred for production).
    bool initialize(
        const UndercutLengthOptions& options = UndercutLengthOptions(),
        UndercutLengthMeasureError* error = nullptr);

    /// Load [Undercut] options from undercut_length.ini ([Input] sections_path ignored).
    bool initializeFromIni(const QString& configPath = QString(), UndercutLengthMeasureError* error = nullptr);

    bool setOptions(const UndercutLengthOptions& options, UndercutLengthMeasureError* error = nullptr);
    bool getOptions(UndercutLengthOptions* out, UndercutLengthMeasureError* error = nullptr) const;

    void shutdown();

    /// Measure from in-memory section sequence (upstream weld measure outputs).
    bool measure(
        const QVector<UndercutSectionInput>& sections,
        UndercutLengthMeasurement* out,
        UndercutLengthMeasureError* error = nullptr);

    /// Load sections txt then measure (tooling / offline).
    bool measureFromFile(
        const QString& sectionsPath,
        UndercutLengthMeasurement* out,
        UndercutLengthMeasureError* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::undercut_length_measure
