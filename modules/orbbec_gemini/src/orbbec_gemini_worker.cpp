#include "scan_tracking/orbbec_gemini/orbbec_gemini_worker.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QString>

#include <iomanip>
#include <memory>
#include <sstream>

#include <libobsensor/ObSensor.hpp>

Q_LOGGING_CATEGORY(LOG_ORBBEC_GEMINI, "orbbec.gemini")

namespace scan_tracking {
namespace orbbec_gemini {

namespace {

QString logPrefix()
{
    return QStringLiteral("[OrbbecGemini]");
}

OrbbecGeminiDeviceSummary summaryFromDeviceInfo(
    int index,
    const std::shared_ptr<ob::DeviceInfo>& info)
{
    OrbbecGeminiDeviceSummary summary;
    summary.index = index;
    if (info == nullptr) {
        return summary;
    }

    summary.name = QString::fromStdString(info->getName());
    summary.serialNumber = QString::fromStdString(info->getSerialNumber());
    summary.firmwareVersion = QString::fromStdString(info->getFirmwareVersion());
    summary.connectionType = QString::fromStdString(info->getConnectionType());
    summary.pid = static_cast<quint16>(info->getPid());
    summary.vid = static_cast<quint16>(info->getVid());
    summary.uid = QString::fromStdString(info->getUid());
    return summary;
}

QString formatDeviceSummaryLine(const OrbbecGeminiDeviceSummary& summary)
{
    std::ostringstream stream;
    stream << "  [" << summary.index << "] name=" << summary.name.toStdString()
           << " SN=" << summary.serialNumber.toStdString()
           << " firmware=" << summary.firmwareVersion.toStdString()
           << " connection=" << summary.connectionType.toStdString();
    return QString::fromStdString(stream.str());
}

QString formatOpenedDeviceLine(const OrbbecGeminiDeviceSummary& summary)
{
    std::ostringstream stream;
    stream << "Opened device: name=" << summary.name.toStdString()
           << " SN=" << summary.serialNumber.toStdString()
           << " firmware=" << summary.firmwareVersion.toStdString()
           << " pid=0x" << std::hex << std::setw(4) << std::setfill('0') << summary.pid
           << " vid=0x" << std::setw(4) << summary.vid << std::dec
           << " uid=" << summary.uid.toStdString()
           << " connection=" << summary.connectionType.toStdString();
    return QString::fromStdString(stream.str());
}

}  // namespace

class OrbbecGeminiWorker::Impl {
public:
    std::unique_ptr<ob::Context> context;
    std::shared_ptr<ob::Device> device;
};

OrbbecGeminiWorker::OrbbecGeminiWorker(QObject* parent)
    : QObject(parent)
    , m_impl(new Impl())
{
}

OrbbecGeminiWorker::~OrbbecGeminiWorker()
{
    stopWorker();
    delete m_impl;
    m_impl = nullptr;
}

void OrbbecGeminiWorker::startWorker(const OrbbecGeminiOpenConfig& config)
{
    stopWorker();

    emit logMessage(QStringLiteral("%1 Starting worker...").arg(logPrefix()));
    emit stateChanged(OrbbecGeminiRuntimeState::Enumerating, QStringLiteral("Enumerating devices"));

    QVector<OrbbecGeminiDeviceSummary> summaries;

    try {
        m_impl->context = std::make_unique<ob::Context>();
        const std::shared_ptr<ob::DeviceList> deviceList = m_impl->context->queryDeviceList();
        const uint32_t count = deviceList->getCount();

        if (count == 0) {
            emit logMessage(QStringLiteral("%1 No device found").arg(logPrefix()));
            emit enumerateFinished(summaries);
            emit openFinished(false, {}, QStringLiteral("No Orbbec device connected"));
            emit stateChanged(OrbbecGeminiRuntimeState::Failed, QStringLiteral("No device found"));
            return;
        }

        emit logMessage(
            QStringLiteral("%1 Enumerated %2 device(s)").arg(logPrefix()).arg(count));

        for (uint32_t index = 0; index < count; ++index) {
            try {
                const std::shared_ptr<ob::Device> probeDevice = deviceList->getDevice(index);
                const std::shared_ptr<ob::DeviceInfo> info = probeDevice->getDeviceInfo();
                const OrbbecGeminiDeviceSummary summary = summaryFromDeviceInfo(static_cast<int>(index), info);
                summaries.push_back(summary);
                emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), formatDeviceSummaryLine(summary)));
            } catch (const ob::Error& error) {
                emit logMessage(
                    QStringLiteral("%1 Failed to probe device index=%2: %3")
                        .arg(logPrefix())
                        .arg(index)
                        .arg(QString::fromStdString(error.what())));
            }
        }

        emit enumerateFinished(summaries);
        emit stateChanged(OrbbecGeminiRuntimeState::Opening, QStringLiteral("Opening device"));

        const QString trimmedSerial = config.serial.trimmed();
        if (!trimmedSerial.isEmpty()) {
            emit logMessage(
                QStringLiteral("%1 Opening device by serial=%2")
                    .arg(logPrefix(), trimmedSerial));
            const QByteArray serialBytes = trimmedSerial.toUtf8();
            m_impl->device = deviceList->getDeviceBySN(serialBytes.constData());
        } else {
            emit logMessage(
                QStringLiteral("%1 Opening device by index=%2")
                    .arg(logPrefix())
                    .arg(config.deviceIndex));
            if (config.deviceIndex < 0
                || static_cast<uint32_t>(config.deviceIndex) >= deviceList->getCount()) {
                throw std::runtime_error(
                    "Device index out of range: "
                    + std::to_string(config.deviceIndex));
            }
            m_impl->device = deviceList->getDevice(static_cast<uint32_t>(config.deviceIndex));
        }

        const OrbbecGeminiDeviceSummary openedSummary =
            summaryFromDeviceInfo(config.deviceIndex, m_impl->device->getDeviceInfo());
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), formatOpenedDeviceLine(openedSummary)));
        emit logMessage(QStringLiteral("%1 Ready (no streams started)").arg(logPrefix()));
        emit openFinished(true, openedSummary, {});
        emit stateChanged(OrbbecGeminiRuntimeState::Ready, QStringLiteral("Device opened"));
    } catch (const ob::Error& error) {
        const QString message = QString::fromStdString(error.what());
        emit logMessage(QStringLiteral("%1 Open failed: %2").arg(logPrefix(), message));
        emit openFinished(false, {}, message);
        emit stateChanged(OrbbecGeminiRuntimeState::Failed, message);
        m_impl->device.reset();
    } catch (const std::exception& error) {
        const QString message = QString::fromUtf8(error.what());
        emit logMessage(QStringLiteral("%1 Open failed: %2").arg(logPrefix(), message));
        emit openFinished(false, {}, message);
        emit stateChanged(OrbbecGeminiRuntimeState::Failed, message);
        m_impl->device.reset();
    }
}

void OrbbecGeminiWorker::stopWorker()
{
    m_impl->device.reset();
    m_impl->context.reset();
    emit stateChanged(OrbbecGeminiRuntimeState::Stopped, QStringLiteral("Worker stopped"));
}

}  // namespace orbbec_gemini
}  // namespace scan_tracking
