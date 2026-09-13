#include "scan_tracking/container_total_length_measure/container_total_length_service.h"

#include "scan_tracking/container_total_length_measure/container_total_length_worker_protocol.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>
#include <QtCore/QMap>
#include <QtCore/QProcess>
#include <QtCore/QTemporaryDir>
#include <QtCore/QTextStream>

#include <mutex>

Q_LOGGING_CATEGORY(LOG_CONTAINER_TOTAL_LENGTH, "container_total_length_measure.service")

namespace scan_tracking::container_total_length_measure {

namespace {

using worker_protocol::kCloudFileName;
using worker_protocol::kDefaultTimeoutMs;
using worker_protocol::kRequestFileName;
using worker_protocol::kResultFileName;
using worker_protocol::kWorkerRelPath;

constexpr int kStatusOk = 0;
constexpr int kStatusInvalidArg = 1;
constexpr int kStatusConfig = 2;
constexpr int kStatusMeasure = 5;
constexpr int kStatusNotInitialized = 6;
constexpr int kStatusInternal = 7;
constexpr int kStatusWorkerCrash = 100;
constexpr int kStatusWorkerTimeout = 101;
constexpr int kStatusWorkerMissing = 102;

void FillError(ContainerTotalLengthError* error, int status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = status;
    error->message = message;
}

QString WorkerExecutablePath()
{
    const QDir appDir(QCoreApplication::applicationDirPath());
    const QByteArray overridePath = qgetenv("SCAN_TRACKING_CTL_WORKER");
    if (!overridePath.isEmpty()) {
        return appDir.filePath(QString::fromLocal8Bit(overridePath));
    }
    return appDir.filePath(QString::fromLatin1(kWorkerRelPath));
}

int ResolveTimeoutMs()
{
    const QByteArray env = qgetenv("SCAN_TRACKING_CTL_WORKER_TIMEOUT_MS");
    if (!env.isEmpty()) {
        bool ok = false;
        const int value = env.toInt(&ok);
        if (ok && value > 0) {
            return value;
        }
    }
    return kDefaultTimeoutMs;
}

bool WriteXyzBin(const QString& path, const float* xyz, size_t pointCount, QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error != nullptr) {
            *error = QStringLiteral("Cannot write %1: %2").arg(path, file.errorString());
        }
        return false;
    }
    if (pointCount == 0 || xyz == nullptr) {
        return true;
    }
    const qint64 bytes = static_cast<qint64>(pointCount * 3 * sizeof(float));
    const qint64 written = file.write(reinterpret_cast<const char*>(xyz), bytes);
    if (written != bytes) {
        if (error != nullptr) {
            *error = QStringLiteral("Short write %1 (%2/%3)").arg(path).arg(written).arg(bytes);
        }
        return false;
    }
    return true;
}

bool WriteRequestFile(
    const QString& path,
    const QString& configPath,
    size_t pointCount,
    QString* error)
{
    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        if (error != nullptr) {
            *error = QStringLiteral("Cannot write request: %1").arg(file.errorString());
        }
        return false;
    }

    QTextStream out(&file);
    out.setCodec("UTF-8");
    out << "config=" << QDir::toNativeSeparators(configPath) << '\n';
    out << "cloud=" << QString::fromLatin1(kCloudFileName) << '\n';
    out << "point_count=" << static_cast<qulonglong>(pointCount) << '\n';
    out.flush();
    return true;
}

QMap<QString, QString> LoadKeyValues(const QString& path)
{
    QMap<QString, QString> map;
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        return map;
    }
    QTextStream in(&file);
    in.setCodec("UTF-8");
    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        if (line.isEmpty() || line.startsWith(QLatin1Char('#'))) {
            continue;
        }
        const int eq = line.indexOf(QLatin1Char('='));
        if (eq <= 0) {
            continue;
        }
        map.insert(line.left(eq).trimmed(), line.mid(eq + 1).trimmed());
    }
    return map;
}

bool ParseMeasureResult(const QMap<QString, QString>& kv, ContainerTotalLengthMeasurement* out)
{
    if (out == nullptr) {
        return false;
    }
    out->lengthMm = kv.value(QStringLiteral("length_mm")).toDouble();
    out->leftEndPosition = kv.value(QStringLiteral("left_end_position")).toDouble();
    out->rightEndPosition = kv.value(QStringLiteral("right_end_position")).toDouble();
    out->icpFitness = kv.value(QStringLiteral("icp_fitness")).toDouble();
    out->fittedRadiusMm = kv.value(QStringLiteral("fitted_radius_mm")).toFloat();
    out->cylinderPointX = kv.value(QStringLiteral("cylinder_point_x")).toFloat();
    out->cylinderPointY = kv.value(QStringLiteral("cylinder_point_y")).toFloat();
    out->cylinderPointZ = kv.value(QStringLiteral("cylinder_point_z")).toFloat();
    out->cylinderAxisX = kv.value(QStringLiteral("cylinder_axis_x")).toFloat();
    out->cylinderAxisY = kv.value(QStringLiteral("cylinder_axis_y")).toFloat();
    out->cylinderAxisZ = kv.value(QStringLiteral("cylinder_axis_z")).toFloat();
    out->icpConverged = kv.value(QStringLiteral("icp_converged")).toInt() != 0;
    out->inputPointCount = kv.value(QStringLiteral("input_point_count")).toInt();
    out->valid = kv.value(QStringLiteral("valid")).toInt() != 0;
    return out->valid;
}

struct WorkerRunOutcome {
    bool ok = false;
    int statusCode = kStatusInternal;
    QString message;
    QMap<QString, QString> resultKv;
    int exitCode = -1;
    bool timedOut = false;
    bool crashed = false;
};

WorkerRunOutcome RunContainerTotalLengthWorker(
    const QString& configPath,
    const float* scanXyz,
    size_t scanCount)
{
    WorkerRunOutcome outcome;

    const QString workerPath = WorkerExecutablePath();
    if (!QFileInfo::exists(workerPath)) {
        outcome.statusCode = kStatusWorkerMissing;
        outcome.message = QStringLiteral("Worker not found: %1").arg(workerPath);
        return outcome;
    }

    QTemporaryDir tempDir;
    tempDir.setAutoRemove(true);
    if (!tempDir.isValid()) {
        outcome.statusCode = kStatusInternal;
        outcome.message = QStringLiteral("Cannot create temp job directory");
        return outcome;
    }

    const QString workDir = tempDir.path();
    const QString requestPath = QDir(workDir).filePath(QString::fromLatin1(kRequestFileName));
    const QString resultPath = QDir(workDir).filePath(QString::fromLatin1(kResultFileName));
    const QString cloudPath = QDir(workDir).filePath(QString::fromLatin1(kCloudFileName));

    QString ioError;
    if (!WriteXyzBin(cloudPath, scanXyz, scanCount, &ioError) ||
        !WriteRequestFile(requestPath, configPath, scanCount, &ioError)) {
        outcome.statusCode = kStatusInternal;
        outcome.message = ioError;
        return outcome;
    }

    const int timeoutMs = ResolveTimeoutMs();
    qInfo(LOG_CONTAINER_TOTAL_LENGTH)
        << "Starting container-total-length worker" << workerPath
        << "points=" << static_cast<qulonglong>(scanCount)
        << "timeoutMs=" << timeoutMs
        << "workdir=" << workDir;

    QProcess process;
    process.setProgram(workerPath);
    process.setArguments(QStringList() << QDir::toNativeSeparators(workDir));
    // DLL search starts from the worker exe directory; keep cwd there too.
    process.setWorkingDirectory(QFileInfo(workerPath).absolutePath());
    process.setProcessChannelMode(QProcess::MergedChannels);
    process.start();
    if (!process.waitForStarted(15000)) {
        outcome.statusCode = kStatusInternal;
        outcome.message = QStringLiteral("Failed to start worker: %1").arg(process.errorString());
        return outcome;
    }

    if (!process.waitForFinished(timeoutMs)) {
        outcome.timedOut = true;
        process.kill();
        process.waitForFinished(5000);
        outcome.statusCode = kStatusWorkerTimeout;
        outcome.message = QStringLiteral("Worker timed out after %1 ms").arg(timeoutMs);
        qWarning(LOG_CONTAINER_TOTAL_LENGTH) << outcome.message
                                             << "stdout:" << QString::fromLocal8Bit(process.readAll());
    } else {
        outcome.exitCode = process.exitCode();
        outcome.crashed = (process.exitStatus() == QProcess::CrashExit);
        const QString workerLog = QString::fromLocal8Bit(process.readAll());
        if (!workerLog.isEmpty()) {
            qInfo(LOG_CONTAINER_TOTAL_LENGTH).noquote() << "worker log:\n" << workerLog;
        }
        qInfo(LOG_CONTAINER_TOTAL_LENGTH)
            << "Worker finished exitCode=" << outcome.exitCode
            << "crashed=" << outcome.crashed;
    }

    if (QFileInfo::exists(resultPath)) {
        outcome.resultKv = LoadKeyValues(resultPath);
    }

    if (outcome.resultKv.isEmpty()) {
        if (outcome.timedOut) {
            return outcome;
        }
        if (outcome.crashed) {
            outcome.statusCode = kStatusWorkerCrash;
            outcome.message = QStringLiteral("Worker crashed without result.txt (exitCode=%1)")
                                  .arg(outcome.exitCode);
            return outcome;
        }
        outcome.statusCode = kStatusInternal;
        outcome.message = QStringLiteral("Worker produced no result.txt (exitCode=%1)")
                              .arg(outcome.exitCode);
        return outcome;
    }

    const int status = outcome.resultKv.value(QStringLiteral("status")).toInt();
    outcome.statusCode = status;
    outcome.message = outcome.resultKv.value(QStringLiteral("message"));
    const bool valid = outcome.resultKv.value(QStringLiteral("valid")).toInt() != 0;
    outcome.ok = (status == kStatusOk) && valid;
    if (!outcome.ok && outcome.message.isEmpty()) {
        outcome.message = QStringLiteral("Worker measure failed (status=%1)").arg(status);
    }
    if (outcome.crashed && !outcome.ok) {
        outcome.statusCode = kStatusWorkerCrash;
        outcome.message += QStringLiteral(" (worker crashed, exitCode=%1)").arg(outcome.exitCode);
    }
    return outcome;
}

}  // namespace

struct ContainerTotalLengthService::Impl {
    mutable std::mutex mutex;
    bool ready = false;
    QString configPath;
};

ContainerTotalLengthService::ContainerTotalLengthService()
    : m_impl(new Impl)
{
}

ContainerTotalLengthService::~ContainerTotalLengthService()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString ContainerTotalLengthService::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/container_total_length/config.ini"));
}

bool ContainerTotalLengthService::isReady() const
{
    if (m_impl == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->ready;
}

QString ContainerTotalLengthService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool ContainerTotalLengthService::initializeFromIni(const QString& configPath, ContainerTotalLengthError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    m_impl->ready = false;
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, kStatusConfig, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_CONTAINER_TOTAL_LENGTH) << "Config missing:" << resolved;
        return false;
    }

    const QString workerPath = WorkerExecutablePath();
    if (!QFileInfo::exists(workerPath)) {
        FillError(
            error,
            kStatusWorkerMissing,
            QStringLiteral("ContainerTotalLength worker not found: %1 "
                           "(deploy under workers/container_total_length/)")
                .arg(workerPath));
        qWarning(LOG_CONTAINER_TOTAL_LENGTH) << "Worker missing:" << workerPath;
        return false;
    }

    // Host no longer loads ContainerTotalLength.dll; create_from_ini runs inside the worker.
    m_impl->configPath = resolved;
    m_impl->ready = true;
    qInfo(LOG_CONTAINER_TOTAL_LENGTH)
        << "ContainerTotalLength ready (out-of-process worker),"
        << "config:" << resolved
        << "worker:" << workerPath;
    return true;
}

void ContainerTotalLengthService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    m_impl->ready = false;
    m_impl->configPath.clear();
}

bool ContainerTotalLengthService::measure(
    const float* scanXyz,
    size_t scanCount,
    ContainerTotalLengthMeasurement* out,
    ContainerTotalLengthError* error)
{
    if (out == nullptr) {
        FillError(error, kStatusInvalidArg, QStringLiteral("out is null"));
        return false;
    }
    *out = ContainerTotalLengthMeasurement{};

    if (scanXyz == nullptr || scanCount == 0) {
        FillError(error, kStatusInvalidArg, QStringLiteral("scan cloud is empty"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (!m_impl->ready || m_impl->configPath.isEmpty()) {
        FillError(error, kStatusNotInitialized, QStringLiteral("ContainerTotalLengthService not initialized"));
        return false;
    }

    const WorkerRunOutcome outcome =
        RunContainerTotalLengthWorker(m_impl->configPath, scanXyz, scanCount);
    if (!outcome.ok) {
        FillError(
            error,
            outcome.statusCode != 0 ? outcome.statusCode : kStatusMeasure,
            outcome.message.isEmpty() ? QStringLiteral("measure failed") : outcome.message);
        qWarning(LOG_CONTAINER_TOTAL_LENGTH) << "measure via worker failed:" << outcome.message;
        return false;
    }

    if (!ParseMeasureResult(outcome.resultKv, out)) {
        FillError(error, kStatusMeasure, QStringLiteral("Worker result invalid"));
        return false;
    }
    return true;
}

}  // namespace scan_tracking::container_total_length_measure
