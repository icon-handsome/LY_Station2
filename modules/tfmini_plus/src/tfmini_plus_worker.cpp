#include "scan_tracking/tfmini_plus/tfmini_plus_worker.h"

#include <QtCore/QLoggingCategory>
#include <QtSerialPort/QSerialPort>

Q_LOGGING_CATEGORY(LOG_TFMINI_PLUS, "tfmini.plus")

namespace scan_tracking {
namespace tfmini_plus {

namespace {

QString logPrefix()
{
    return QStringLiteral("[TfminiPlus]");
}

QString bytesToHexLine(const QByteArray& data)
{
    return QString::fromLatin1(data.toHex(' ')).toUpper();
}

int byteAt(const QByteArray& data, int index)
{
    return static_cast<unsigned char>(data.at(index));
}

bool hasValidChecksum(const QByteArray& frame)
{
    int checksum = 0;
    for (int i = 0; i < 8; ++i) {
        checksum = (checksum + byteAt(frame, i)) & 0xFF;
    }
    return checksum == byteAt(frame, 8);
}

}  // namespace

TfminiPlusWorker::TfminiPlusWorker(QObject* parent)
    : QObject(parent)
{
}

TfminiPlusWorker::~TfminiPlusWorker()
{
    stopWorker();
}

void TfminiPlusWorker::startWorker(const TfminiPlusOpenConfig& config)
{
    stopWorker();

    m_openConfig = config;
    m_buffer.clear();

    const QString portName = config.portName.trimmed();
    if (portName.isEmpty()) {
        const QString message = QStringLiteral("Serial port is empty; set tfminiPlusPort in [TfminiPlus]");
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        emit openFinished(false, message);
        emit stateChanged(TfminiPlusRuntimeState::Failed, message);
        return;
    }

    emit stateChanged(TfminiPlusRuntimeState::Opening, QStringLiteral("Opening serial port"));

    m_serialPort = new QSerialPort(this);
    m_serialPort->setPortName(portName);
    m_serialPort->setBaudRate(config.baudRate > 0 ? config.baudRate : 115200);
    m_serialPort->setDataBits(QSerialPort::Data8);
    m_serialPort->setParity(QSerialPort::NoParity);
    m_serialPort->setStopBits(QSerialPort::OneStop);
    m_serialPort->setFlowControl(QSerialPort::NoFlowControl);

    connect(m_serialPort, &QSerialPort::readyRead, this, &TfminiPlusWorker::onReadyRead);
    connect(
        m_serialPort,
        QOverload<QSerialPort::SerialPortError>::of(&QSerialPort::errorOccurred),
        this,
        &TfminiPlusWorker::onSerialError);

    if (!m_serialPort->open(QIODevice::ReadOnly)) {
        const QString message = QStringLiteral("Open serial port failed: %1 (%2)")
                                    .arg(portName, m_serialPort->errorString());
        emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
        emit openFinished(false, message);
        emit stateChanged(TfminiPlusRuntimeState::Failed, message);
        teardownSerial();
        return;
    }

    const QString openedMessage = QStringLiteral("Opened %1 baud=%2 8N1")
                                      .arg(portName)
                                      .arg(m_serialPort->baudRate());
    emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), openedMessage));
    emit openFinished(true, {});
    emit stateChanged(TfminiPlusRuntimeState::Ready, openedMessage);
}

void TfminiPlusWorker::stopWorker()
{
    const bool wasOpen = m_serialPort != nullptr;
    teardownSerial();
    m_buffer.clear();
    if (wasOpen) {
        emit stateChanged(TfminiPlusRuntimeState::Stopped, QStringLiteral("Worker stopped"));
    }
}

void TfminiPlusWorker::onReadyRead()
{
    if (m_serialPort == nullptr) {
        return;
    }

    const QByteArray data = m_serialPort->readAll();
    if (data.isEmpty()) {
        return;
    }

    if (m_openConfig.printRawData) {
        emit logMessage(QStringLiteral("%1 raw=%2").arg(logPrefix(), bytesToHexLine(data)));
    }

    m_buffer.append(data);
    parseBuffer();
}

void TfminiPlusWorker::onSerialError()
{
    if (m_serialPort == nullptr) {
        return;
    }

    const auto error = m_serialPort->error();
    if (error == QSerialPort::NoError || error == QSerialPort::TimeoutError) {
        return;
    }

    const QString message = QStringLiteral("Serial port error: %1").arg(m_serialPort->errorString());
    emit logMessage(QStringLiteral("%1 %2").arg(logPrefix(), message));
    emit stateChanged(TfminiPlusRuntimeState::Failed, message);
}

void TfminiPlusWorker::parseBuffer()
{
    constexpr int kFrameSize = 9;
    constexpr char kFrameHeader = '\x59';

    while (m_buffer.size() >= 2) {
        int headerIndex = -1;
        for (int i = 0; i + 1 < m_buffer.size(); ++i) {
            if (m_buffer.at(i) == kFrameHeader && m_buffer.at(i + 1) == kFrameHeader) {
                headerIndex = i;
                break;
            }
        }

        if (headerIndex < 0) {
            m_buffer.clear();
            return;
        }
        if (headerIndex > 0) {
            m_buffer.remove(0, headerIndex);
        }
        if (m_buffer.size() < kFrameSize) {
            return;
        }

        const QByteArray frame = m_buffer.left(kFrameSize);
        if (!hasValidChecksum(frame)) {
            emit logMessage(QStringLiteral("%1 checksum failed frame=%2")
                                .arg(logPrefix(), bytesToHexLine(frame)));
            m_buffer.remove(0, 1);
            continue;
        }

        const int distanceCm = byteAt(frame, 2) | (byteAt(frame, 3) << 8);
        const int strength = byteAt(frame, 4) | (byteAt(frame, 5) << 8);
        emit distanceUpdated(distanceCm, strength);
        m_buffer.remove(0, kFrameSize);
    }
}

void TfminiPlusWorker::teardownSerial()
{
    if (m_serialPort == nullptr) {
        return;
    }

    if (m_serialPort->isOpen()) {
        m_serialPort->close();
    }
    delete m_serialPort;
    m_serialPort = nullptr;
}

}  // namespace tfmini_plus
}  // namespace scan_tracking
