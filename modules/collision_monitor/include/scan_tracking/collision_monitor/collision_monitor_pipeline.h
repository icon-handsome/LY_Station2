#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/collision_monitor/collision_monitor_service.h"

namespace scan_tracking::collision_monitor {

/// Feeds Livox point clouds into CollisionMonitorService and logs confirmed alerts.
class CollisionMonitorPipeline : public QObject {
    Q_OBJECT

public:
    explicit CollisionMonitorPipeline(QObject* parent = nullptr);
    ~CollisionMonitorPipeline() override;

    bool isReady() const { return m_ready; }

public slots:
    bool start(const QString& configPath = QString());
    void stop();
    void onPointCloudFrame(QVector<float> xyz);

private:
    static QString alertLevelName(CollisionAlertLevel level);

    CollisionMonitorService m_service;
    bool m_ready = false;
    bool m_backgroundSet = false;
};

}  // namespace scan_tracking::collision_monitor
