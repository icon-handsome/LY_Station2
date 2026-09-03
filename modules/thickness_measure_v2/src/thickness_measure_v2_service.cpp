#include "scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <cstddef>
#include <cstring>
#include <mutex>
#include <vector>

#include "thickness_measure_v2_c_api.h"

Q_LOGGING_CATEGORY(LOG_THICKNESS_MEASURE_V2, "thickness_measure_v2.service")

namespace scan_tracking::thickness_measure_v2 {

namespace {

constexpr size_t kMaxInputPointsPerCloud = 600000;

std::vector<float> boundedCloud(const float* xyz, size_t pointCount)
{
    if (xyz == nullptr || pointCount == 0) {
        return {};
    }
    if (pointCount <= kMaxInputPointsPerCloud) {
        return std::vector<float>(xyz, xyz + pointCount * 3u);
    }

    // Uniformly retain the cloud envelope while bounding the V3 DLL's raw
    // input/ICP working set. The DLL performs its own voxel preprocessing;
    // this cap prevents it from first materializing multi-million-point copies.
    std::vector<float> sampled;
    sampled.reserve(kMaxInputPointsPerCloud * 3u);
    for (size_t i = 0; i < kMaxInputPointsPerCloud; ++i) {
        const size_t source = (i * pointCount) / kMaxInputPointsPerCloud;
        sampled.push_back(xyz[source * 3u + 0u]);
        sampled.push_back(xyz[source * 3u + 1u]);
        sampled.push_back(xyz[source * 3u + 2u]);
    }
    return sampled;
}

void FillError(ThicknessV2Error* error, tmv2_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(tmv2_status_string(status))
        : message;
}

void DestroyContext(tmv2_context*& ctx)
{
    if (ctx != nullptr) {
        tmv2_destroy(ctx);
        ctx = nullptr;
    }
}

void FillPairResult(const tmv2_pair_result& src, ThicknessV2PairMeasurement* dst)
{
    if (dst == nullptr) {
        return;
    }
    dst->innerOuterIcpFitness = src.inner_outer_icp_fitness;
    dst->outerTemplateIcpFitness = src.outer_template_icp_fitness;
    dst->thicknessMm = src.thickness_mm;
    // The V3 DLL does not guarantee that its fixed-size method buffer is
    // NUL-terminated. Never pass it to a C-string API without a bound: the
    // previous unbounded read occurred immediately after the DLL printed its
    // completion line and could terminate the IPC process.
    size_t methodLength = 0;
    while (methodLength < sizeof(src.method) && src.method[methodLength] != '\0') {
        ++methodLength;
    }
    dst->method = QString::fromUtf8(src.method, static_cast<int>(methodLength));
    dst->sectionCount = src.section_count;
    dst->valid = src.valid != 0;
}

}  // namespace

struct ThicknessMeasureV2Service::Impl {
    mutable std::mutex mutex;
    tmv2_context* ctx = nullptr;
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
    return m_impl->ctx != nullptr;
}

QString ThicknessMeasureV2Service::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool ThicknessMeasureV2Service::initializeFromIni(const QString& configPath, ThicknessV2Error* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, TMV2_ERR_CONFIG, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_THICKNESS_MEASURE_V2) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    tmv2_context* ctx = nullptr;
    char message[512] = {0};
    const tmv2_status status = tmv2_create_from_ini(
        pathUtf8.constData(), &ctx, message, sizeof(message));
    if (status != TMV2_OK || ctx == nullptr) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message)
                               : QString::fromUtf8(tmv2_status_string(status)));
        qWarning(LOG_THICKNESS_MEASURE_V2) << "tmv2_create_from_ini failed:" << message;
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    qInfo(LOG_THICKNESS_MEASURE_V2) << "ThicknessMeasureV2 ready, config:" << resolved;
    return true;
}

void ThicknessMeasureV2Service::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
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
        FillError(error, TMV2_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, TMV2_ERR_NOT_INITIALIZED, QStringLiteral("ThicknessMeasureV2Service not initialized"));
        return false;
    }

    tmv2_pair_result result{};
    char message[512] = {0};
    const tmv2_status status = tmv2_measure_pair(
        m_impl->ctx,
        innerScanXyz,
        innerScanCount,
        outerScanXyz,
        outerScanCount,
        &result,
        message,
        sizeof(message));
    if (status != TMV2_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message)
                               : QString::fromUtf8(tmv2_status_string(status)));
        qWarning(LOG_THICKNESS_MEASURE_V2) << "tmv2_measure_pair failed:" << message;
        return false;
    }

    FillPairResult(result, out);
    return true;
}

bool ThicknessMeasureV2Service::measurePairsAverage(
    const QVector<ThicknessV2PairClouds>& pairs,
    ThicknessV2AverageMeasurement* out,
    ThicknessV2Error* error)
{
    if (out == nullptr) {
        FillError(error, TMV2_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    // The V3 runtime can retain substantial per-pair ICP/ONNX scratch memory.
    // Calling its batch entry point with both path4 pairs has caused the field
    // process to terminate while entering the second pair.  Process pairs one
    // at a time so each call can release its temporary buffers, then aggregate
    // the valid results here.  This also gives the caller deterministic
    // success/pair counts when one pair is invalid.
    *out = ThicknessV2AverageMeasurement{};
    if (pairs.isEmpty()) {
        FillError(error, TMV2_ERR_INVALID_ARG, QStringLiteral("pairs is empty"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, TMV2_ERR_NOT_INITIALIZED, QStringLiteral("ThicknessMeasureV2Service not initialized"));
        return false;
    }

    double thicknessSum = 0.0;
    QString firstError;
    for (const ThicknessV2PairClouds& pair : pairs) {
        if (pair.inner.xyz == nullptr || pair.outer.xyz == nullptr ||
            pair.inner.pointCount == 0 || pair.outer.pointCount == 0) {
            ++out->pairCount;
            if (firstError.isEmpty()) {
                firstError = QStringLiteral("pair contains an empty cloud");
            }
            continue;
        }

        const std::vector<float> innerBounded = boundedCloud(
            pair.inner.xyz, pair.inner.pointCount);
        const std::vector<float> outerBounded = boundedCloud(
            pair.outer.xyz, pair.outer.pointCount);
        const float* innerXyz = innerBounded.empty() ? pair.inner.xyz : innerBounded.data();
        const float* outerXyz = outerBounded.empty() ? pair.outer.xyz : outerBounded.data();
        const size_t innerCount = innerBounded.empty()
            ? pair.inner.pointCount : innerBounded.size() / 3u;
        const size_t outerCount = outerBounded.empty()
            ? pair.outer.pointCount : outerBounded.size() / 3u;

        tmv2_pair_result result{};
        char message[512] = {0};
        const tmv2_status status = tmv2_measure_pair(
            m_impl->ctx,
            innerXyz,
            innerCount,
            outerXyz,
            outerCount,
            &result,
            message,
            sizeof(message));
        ++out->pairCount;
        if (status != TMV2_OK || !result.valid) {
            if (firstError.isEmpty()) {
                firstError = message[0] != '\0'
                    ? QString::fromUtf8(message)
                    : QString::fromUtf8(tmv2_status_string(
                          status == TMV2_OK ? TMV2_ERR_MEASURE : status));
            }
            qWarning(LOG_THICKNESS_MEASURE_V2)
                << "tmv2_measure_pair failed:" << message
                << "status=" << static_cast<int>(status);
            continue;
        }
        thicknessSum += result.thickness_mm;
        ++out->successCount;
    }

    out->valid = out->successCount > 0;
    out->thicknessMm = out->valid
        ? thicknessSum / static_cast<double>(out->successCount)
        : 0.0;
    if (!out->valid) {
        FillError(error, TMV2_ERR_MEASURE,
                  firstError.isEmpty() ? QStringLiteral("no valid thickness pair")
                                       : firstError);
        return false;
    }
    return true;
}

}  // namespace scan_tracking::thickness_measure_v2
