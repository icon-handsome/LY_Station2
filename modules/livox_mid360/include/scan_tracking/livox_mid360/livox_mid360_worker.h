#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtCore/QVector>

#include <mutex>
#include <atomic>
#include <vector>

#include "scan_tracking/livox_mid360/livox_mid360_types.h"

namespace scan_tracking {
namespace livox_mid360 {

class LivoxMid360Worker : public QObject {
    Q_OBJECT

public:
    explicit LivoxMid360Worker(QObject* parent = nullptr);
    ~LivoxMid360Worker() override;

    void appendPointCloudPoints(std::vector<float> points);
    quint32 selectedHandleForPointCloud() const { return m_selectedHandle; }
    bool pointCloudStreamActive() const { return m_pointCloudStreamActive.load(std::memory_order_acquire); }

public slots:
    void startWorker(const scan_tracking::livox_mid360::LivoxMid360OpenConfig& config);
    void stopWorker();
    void onDeviceDiscovered(
        quint32 handle,
        QString serialNumber,
        QString lidarIp,
        int deviceType);
    void onDiscoveryTimeout();

signals:
    void enumerateFinished(QVector<scan_tracking::livox_mid360::LivoxMid360DeviceSummary> devices);
    void openFinished(
        bool success,
        scan_tracking::livox_mid360::LivoxMid360DeviceSummary deviceInfo,
        QString errorMessage);
    void stateChanged(
        scan_tracking::livox_mid360::LivoxMid360RuntimeState newState,
        QString description);
    void logMessage(QString message);
    void discoveryWindowFinished();
    void pointCloudFrameReady(QVector<float> xyz);

private slots:
    void flushPointCloudBuffer();

private:
    void finishDiscovery();
    void startPointCloudStream(quint32 handle);
    void stopPointCloudStream();

    static int pointCloudFeedIntervalMs(const LivoxMid360OpenConfig& config);

    bool m_sdkInitialized = false;
    bool m_discoveryActive = false;
    int m_nextDeviceIndex = 0;
    QVector<LivoxMid360DeviceSummary> m_discoveredDevices;
    LivoxMid360OpenConfig m_openConfig;
    QString m_tempConfigPath;

    std::atomic<quint32> m_selectedHandle{0};
    std::atomic_bool m_pointCloudStreamActive{false};
    QTimer* m_pointCloudFlushTimer = nullptr;
    std::mutex m_pointCloudMutex;
    std::vector<float> m_pointCloudBuffer;
    void teardownSdk();
    void cleanupTempConfigFile();
    QString resolveConfigPathForSdk(const QString& configPath, QString* warningMessage);
};

}  // namespace livox_mid360
}  // namespace scan_tracking
