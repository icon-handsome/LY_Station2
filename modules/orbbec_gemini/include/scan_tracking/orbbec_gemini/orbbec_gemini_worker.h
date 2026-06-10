#pragma once

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QVector>

#include "scan_tracking/orbbec_gemini/orbbec_gemini_types.h"

namespace scan_tracking {
namespace orbbec_gemini {

class OrbbecGeminiWorker : public QObject {
    Q_OBJECT

public:
    explicit OrbbecGeminiWorker(QObject* parent = nullptr);
    ~OrbbecGeminiWorker() override;

public slots:
    void startWorker(const scan_tracking::orbbec_gemini::OrbbecGeminiOpenConfig& config);
    void stopWorker();
    void performCapture(const scan_tracking::orbbec_gemini::OrbbecCaptureRequest& request);

signals:
    void enumerateFinished(QVector<scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary> devices);
    void openFinished(
        bool success,
        scan_tracking::orbbec_gemini::OrbbecGeminiDeviceSummary deviceInfo,
        QString errorMessage);
    void stateChanged(
        scan_tracking::orbbec_gemini::OrbbecGeminiRuntimeState newState,
        QString description);
    void captureFinished(scan_tracking::orbbec_gemini::OrbbecCaptureResult result);
    void logMessage(QString message);

private:
    class Impl;
    Impl* m_impl = nullptr;
    OrbbecGeminiOpenConfig m_config;
    OrbbecGeminiDeviceSummary m_openedDevice;
    bool m_stopping = false;
};

}  // namespace orbbec_gemini
}  // namespace scan_tracking
