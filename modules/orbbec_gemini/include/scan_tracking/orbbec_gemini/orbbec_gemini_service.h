#pragma once

#include <QtCore/QObject>

#include "scan_tracking/orbbec_gemini/orbbec_gemini_types.h"

QT_BEGIN_NAMESPACE
class QThread;
QT_END_NAMESPACE

namespace scan_tracking {
namespace orbbec_gemini {

class OrbbecGeminiWorker;

class OrbbecGeminiService : public QObject {
    Q_OBJECT

public:
    explicit OrbbecGeminiService(QObject* parent = nullptr);
    ~OrbbecGeminiService() override;

    void start();
    void stop();

    OrbbecGeminiRuntimeState state() const { return m_currentState; }

signals:
    void enumerateFinished(QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary> devices);
    void openFinished(
        bool success,
        scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary deviceInfo,
        QString errorMessage);
    void stateChanged(
        scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState newState,
        QString description);
    void logMessage(QString message);

private slots:
    void onWorkerEnumerateFinished(
        QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary> devices);
    void onWorkerOpenFinished(
        bool success,
        scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary deviceInfo,
        QString errorMessage);
    void onWorkerStateChanged(
        scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState newState,
        QString description);
    void onWorkerLogMessage(QString message);

signals:
    void sig_startWorker(scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig config);
    void sig_stopWorker();

private:
    static void registerMetaTypes();

    QThread* m_workerThread = nullptr;
    OrbbecGeminiWorker* m_worker = nullptr;
    OrbbecGeminiOpenConfig m_openConfig;
    OrbbecGeminiRuntimeState m_currentState = OrbbecGeminiRuntimeState::Idle;
    bool m_started = false;
    bool m_stopping = false;
};

}  // namespace orbbec_gemini
}  // namespace scan_tracking
