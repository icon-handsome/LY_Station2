#pragma once

#include <cstddef>

#include <QtCore/QString>

#include "scan_tracking/collision_monitor/collision_monitor_types.h"

namespace scan_tracking::collision_monitor {

/// Thin façade over CollisionDetection.dll (C API). Calls are serialized per instance.
class CollisionMonitorService {
public:
    CollisionMonitorService();
    ~CollisionMonitorService();

    CollisionMonitorService(const CollisionMonitorService&) = delete;
    CollisionMonitorService& operator=(const CollisionMonitorService&) = delete;

    /// Default config: <applicationDir>/config/collision_detection/collision.ini
    static QString defaultConfigPath();

    bool isReady() const;
    bool hasBackground() const;
    QString configPath() const;

    bool initializeFromIni(const QString& configPath = QString(), CollisionMonitorError* error = nullptr);

    /// Static background cloud with no hoisted object; replaces prior background and resets state.
    bool setBackground(
        const float* xyz,
        size_t pointCount,
        CollisionMonitorError* error = nullptr);

    bool setBackground(
        const CollisionCloudView& cloud,
        CollisionMonitorError* error = nullptr);

    /// Detect one live lidar frame; consecutive-frame state is kept inside the DLL context.
    bool detect(
        const float* xyz,
        size_t pointCount,
        CollisionDetectResult* out,
        CollisionMonitorError* error = nullptr);

    bool detect(
        const CollisionCloudView& cloud,
        CollisionDetectResult* out,
        CollisionMonitorError* error = nullptr);

    void resetState();
    void shutdown();

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::collision_monitor
