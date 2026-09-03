#include "scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h"

#include "scan_tracking/thickness_measure_v2/thickness_measure_v3_worker_protocol.h"

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

Q_LOGGING_CATEGORY(LOG_THICKNESS_MEASURE_V2, "thickness_measure_v2.service")

namespace scan_tracking::thickness_measure_v2 {

namespace {

using worker_protocol::kDefaultTimeoutMs;
using worker_protocol::kRequestFileName;
using worker_protocol::kResultFileName;
using worker_protocol::kWorkerExeName;

enum class WorkerMode {
    Pair,
    PairsAverage,
};

constexpr int kStatusOk = 0;
constexpr int kStatusInvalidArg = 1;
constexpr int kStatusNotInitialized = 2;
constexpr int kStatusConfig = 3;
constexpr int kStatusMeasure = 6;
constexpr int kStatusInternal = 7;
constexpr int kStatusWorkerCrash = 100;
constexpr int kStatusWorkerTimeout = 101;
constexpr int kStatusWorkerMissing = 102;

void FillError(ThicknessV2Error* error, int status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = status;
    error->message = message;
}

QString WorkerExecutablePath()
{
    return QDir(QCoreApplication::applicationDirPath()).filePath(QString::fromLatin1(kWorkerExeName));
}

int ResolveTimeoutMs()
{
    const QByteArray env = qgetenv("SCAN_TRACKING_THICKNESS_WORKER_TIMEOUT_MS");
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
    WorkerMode mode,
    const QString& configPath,
    const QVector<ThicknessV2PairClouds>& pairs,
    const QString& workDir,
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
    out << "mode=" << (mode == WorkerMode::Pair ? "pair" : "pairs_average") << '\n';
    out << "config=" << QDir::toNativeSeparators(configPath) << '\n';
    out << "pair_count=" << pairs.size() << '\n';

    for (int i = 0; i < pairs.size(); ++i) {
        const QString innerName = QStringLiteral("pair_%1_inner.bin").arg(i);
        const QString outerName = QStringLiteral("pair_%1_outer.bin").arg(i);
        const QString innerPath = QDir(workDir).filePath(innerName);
        const QString outerPath = QDir(workDir).filePath(outerName);
        if (!WriteXyzBin(innerPath, pairs[i].inner.xyz, pairs[i].inner.pointCount, error) ||
            !WriteXyzBin(outerPath, pairs[i].outer.xyz, pairs[i].outer.pointCount, error)) {
            return false;
        }
        out << "inner_" << i << '=' << innerName << '\n';
        out << "inner_" << i << "_count=" << static_cast<qulonglong>(pairs[i].inner.pointCount) << '\n';
        out << "outer_" << i << '=' << outerName << '\n';
        out << "outer_" << i << "_count=" << static_cast<qulonglong>(pairs[i].outer.pointCount) << '\n';
    }
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

bool ParseAverageResult(const QMap<QString, QString>& kv, ThicknessV2AverageMeasurement* out)
{
    if (out == nullptr) {
        return false;
    }
    out->thicknessMm = kv.value(QStringLiteral("thickness_mm")).toDouble();
    out->pairCount = static_cast<size_t>(kv.value(QStringLiteral("pair_count")).toULongLong());
    out->successCount = static_cast<size_t>(kv.value(QStringLiteral("success_count")).toULongLong());
    out->valid = kv.value(QStringLiteral("valid")).toInt() != 0;
    return out->valid && out->successCount > 0;
}

bool ParsePairResult(const QMap<QString, QString>& kv, ThicknessV2PairMeasurement* out)
{
    if (out == nullptr) {
        return false;
    }
    out->innerOuterIcpFitness = kv.value(QStringLiteral("inner_outer_icp_fitness")).toDouble();
    out->outerTemplateIcpFitness = kv.value(QStringLiteral("outer_template_icp_fitness")).toDouble();
    out->thicknessMm = kv.value(QStringLiteral("thickness_mm")).toDouble();
    out->method = kv.value(QStringLiteral("method"));
    out->sectionCount = kv.value(QStringLiteral("section_count")).toInt();
    out->valid = kv.value(QStringLiteral("valid")).toInt() != 0;
    return out->valid;
}

struct WorkerRunOutcome {
    bool ok = false;
    int statusCode = kStatusInternal;
    QString message;
    QMap<QString, QString> resultKv;
    int exitCode = -1;
    QProcess::ExitStatus exitStatus = QProcess::NormalExit;
    bool timedOut = false;
    bool crashed = false;
};

WorkerRunOutcome RunThicknessWorker(
    WorkerMode mode,
    const QString& configPath,
    const QVector<ThicknessV2PairClouds>& pairs)
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

    QString ioError;
    if (!WriteRequestFile(requestPath, mode, configPath, pairs, workDir, &ioError)) {
        outcome.statusCode = kStatusInternal;
        outcome.message = ioError;
        return outcome;
    }

    const int timeoutMs = ResolveTimeoutMs();
    qInfo(LOG_THICKNESS_MEASURE_V2)
        << "Starting thickness worker" << workerPath
        << "pairs=" << pairs.size()
        << "timeoutMs=" << timeoutMs
        << "workdir=" << workDir;

    QProcess process;
    process.setProgram(workerPath);
    process.setArguments(QStringList() << QDir::toNativeSeparators(workDir));
    process.setWorkingDirectory(QCoreApplication::applicationDirPath());
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
        qWarning(LOG_THICKNESS_MEASURE_V2) << outcome.message
                                           << "stdout:" << QString::fromLocal8Bit(process.readAll());
        // Still try to read partial result if worker wrote before hang.
    } else {
        outcome.exitCode = process.exitCode();
        outcome.exitStatus = process.exitStatus();
        outcome.crashed = (process.exitStatus() == QProcess::CrashExit);
        const QString workerLog = QString::fromLocal8Bit(process.readAll());
        if (!workerLog.isEmpty()) {
            qInfo(LOG_THICKNESS_MEASURE_V2).noquote() << "worker log:\n" << workerLog;
        }
        qInfo(LOG_THICKNESS_MEASURE_V2)
            << "Worker finished exitCode=" << outcome.exitCode
            << "crashed=" << outcome.crashed;
    }

    if (QFileInfo::exists(resultPath)) {
        outcome.resultKv = LoadKeyValues(resultPath);
    }

    if (!outcome.resultKv.isEmpty()) {
        const int status = outcome.resultKv.value(QStringLiteral("status")).toInt();
        const QString message = outcome.resultKv.value(QStringLiteral("message"));
        const bool valid = outcome.resultKv.value(QStringLiteral("valid")).toInt() != 0;
        const quint64 successCount =
            outcome.resultKv.value(QStringLiteral("success_count")).toULongLong();
        if (valid && successCount > 0 && status == kStatusOk) {
            outcome.ok = true;
            outcome.statusCode = kStatusOk;
            outcome.message = message;
            if (outcome.crashed || outcome.timedOut) {
                qWarning(LOG_THICKNESS_MEASURE_V2)
                    << "Worker exited abnormally after writing a valid result"
                    << "(teardown crash is isolated from scan-tracking)."
                    << "exitCode=" << outcome.exitCode
                    << "timedOut=" << outcome.timedOut;
            }
            return outcome;
        }
        outcome.ok = false;
        outcome.statusCode = status == kStatusOk ? kStatusMeasure : status;
        outcome.message = message.isEmpty()
            ? QStringLiteral("Worker reported invalid/failed measurement")
            : message;
        if (outcome.crashed && outcome.message.indexOf(QLatin1String("crash"), 0, Qt::CaseInsensitive) < 0) {
            outcome.message += QStringLiteral(" (worker crashed, exitCode=%1)").arg(outcome.exitCode);
            if (outcome.statusCode == kStatusMeasure) {
                outcome.statusCode = kStatusWorkerCrash;
            }
        }
        return outcome;
    }

    if (outcome.timedOut) {
        return outcome;
    }
    if (outcome.crashed) {
        outcome.statusCode = kStatusWorkerCrash;
        outcome.message = QStringLiteral("Worker crashed without result (exitCode=%1)")
                              .arg(outcome.exitCode);
        return outcome;
    }
    outcome.statusCode = kStatusMeasure;
    outcome.message = QStringLiteral("Worker exitCode=%1 without result.txt").arg(outcome.exitCode);
    return outcome;
}

}  // namespace

struct ThicknessMeasureV2Service::Impl {
    mutable std::mutex mutex;
    bool ready = false;
    QString configPath;
};

ThicknessMeasureV2Service::ThicknessMeasureV2Service()
    : m_impl(new Impl)
{
}

ThicknessMeasureV2Service::~ThicknessMeasureV2Service()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString ThicknessMeasureV2Service::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/thickness_measure_v3/thickness_measurement.ini"));
}

bool ThicknessMeasureV2Service::isReady() const
{
    if (m_impl == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->ready;
}

QString ThicknessMeasureV2Service::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool ThicknessMeasureV2Service::initializeFromIni(const QString& configPath, ThicknessV2Error* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    m_impl->ready = false;
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, kStatusConfig, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_THICKNESS_MEASURE_V2) << "Config missing:" << resolved;
        return false;
    }

    const QString workerPath = WorkerExecutablePath();
    if (!QFileInfo::exists(workerPath)) {
        FillError(
            error,
            kStatusWorkerMissing,
            QStringLiteral("Thickness worker not found: %1 (deploy beside scan-tracking.exe)")
                .arg(workerPath));
        qWarning(LOG_THICKNESS_MEASURE_V2) << "Worker missing:" << workerPath;
        return false;
    }

    // Host no longer loads ThicknessMeasureV3.dll; create_from_ini runs inside the worker.
    m_impl->configPath = resolved;
    m_impl->ready = true;
    qInfo(LOG_THICKNESS_MEASURE_V2)
        << "ThicknessMeasureV2 ready (out-of-process worker),"
        << "config:" << resolved
        << "worker:" << workerPath;
    return true;
}

void ThicknessMeasureV2Service::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    m_impl->ready = false;
    m_impl->configPath.clear();
}

bool ThicknessMeasureV2Service::measurePair(
    const float* innerScanXyz,
    size_t innerScanCount,
    const float* outerScanXyz,
    size_t outerScanCount,
    ThicknessV2PairMeasurement* out,
    ThicknessV2Error* error)
{
    if (out == nullptr) {
        FillError(error, kStatusInvalidArg, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (!m_impl->ready) {
        FillError(error, kStatusNotInitialized, QStringLiteral("ThicknessMeasureV2Service not initialized"));
        return false;
    }

    ThicknessV2PairClouds pair;
    pair.inner.xyz = innerScanXyz;
    pair.inner.pointCount = innerScanCount;
    pair.outer.xyz = outerScanXyz;
    pair.outer.pointCount = outerScanCount;

    const WorkerRunOutcome outcome =
        RunThicknessWorker(WorkerMode::Pair, m_impl->configPath, QVector<ThicknessV2PairClouds>{pair});
    if (!outcome.ok) {
        FillError(error, outcome.statusCode, outcome.message);
        qWarning(LOG_THICKNESS_MEASURE_V2) << "measurePair via worker failed:" << outcome.message;
        return false;
    }

    if (!ParsePairResult(outcome.resultKv, out)) {
        FillError(error, kStatusMeasure, QStringLiteral("Worker result missing valid pair fields"));
        return false;
    }
    return true;
}

bool ThicknessMeasureV2Service::measurePairsAverage(
    const QVector<ThicknessV2PairClouds>& pairs,
    ThicknessV2AverageMeasurement* out,
    ThicknessV2Error* error)
{
    if (out == nullptr) {
        FillError(error, kStatusInvalidArg, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (!m_impl->ready) {
        FillError(error, kStatusNotInitialized, QStringLiteral("ThicknessMeasureV2Service not initialized"));
        return false;
    }
    if (pairs.isEmpty()) {
        FillError(error, kStatusInvalidArg, QStringLiteral("pairs is empty"));
        return false;
    }

    const WorkerRunOutcome outcome =
        RunThicknessWorker(WorkerMode::PairsAverage, m_impl->configPath, pairs);
    if (!outcome.ok) {
        FillError(error, outcome.statusCode, outcome.message);
        // Best-effort fill counts for callers that log success/pair ratios.
        if (!outcome.resultKv.isEmpty()) {
            out->thicknessMm = outcome.resultKv.value(QStringLiteral("thickness_mm")).toDouble();
            out->pairCount =
                static_cast<size_t>(outcome.resultKv.value(QStringLiteral("pair_count")).toULongLong());
            out->successCount =
                static_cast<size_t>(outcome.resultKv.value(QStringLiteral("success_count")).toULongLong());
            out->valid = false;
        } else {
            out->pairCount = static_cast<size_t>(pairs.size());
            out->successCount = 0;
            out->valid = false;
        }
        qWarning(LOG_THICKNESS_MEASURE_V2) << "measurePairsAverage via worker failed:" << outcome.message;
        return false;
    }

    if (!ParseAverageResult(outcome.resultKv, out)) {
        FillError(error, kStatusMeasure, QStringLiteral("Worker result missing valid average fields"));
        return false;
    }
    return true;
}

}  // namespace scan_tracking::thickness_measure_v2
