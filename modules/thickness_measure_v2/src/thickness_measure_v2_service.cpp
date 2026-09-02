#include "scan_tracking/thickness_measure_v2/thickness_measure_v2_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>
#include <vector>

#include "thickness_measure_v2_c_api.h"

Q_LOGGING_CATEGORY(LOG_THICKNESS_MEASURE_V2, "thickness_measure_v2.service")

namespace scan_tracking::thickness_measure_v2 {

namespace {

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
    dst->method = QString::fromUtf8(src.method);
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

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, TMV2_ERR_NOT_INITIALIZED, QStringLiteral("ThicknessMeasureV2Service not initialized"));
        return false;
    }

    std::vector<tmv2_pair_clouds> cPairs;
    cPairs.reserve(static_cast<size_t>(pairs.size()));
    for (const ThicknessV2PairClouds& pair : pairs) {
        tmv2_pair_clouds view{};
        view.inner.xyz = pair.inner.xyz;
        view.inner.point_count = pair.inner.pointCount;
        view.outer.xyz = pair.outer.xyz;
        view.outer.point_count = pair.outer.pointCount;
        cPairs.push_back(view);
    }

    tmv2_average_result average{};
    char message[512] = {0};
    const tmv2_status status = tmv2_measure_pairs_average(
        m_impl->ctx,
        cPairs.data(),
        cPairs.size(),
        &average,
        message,
        sizeof(message));
    if (status != TMV2_OK || !average.valid || average.success_count == 0) {
        FillError(
            error,
            status == TMV2_OK ? TMV2_ERR_MEASURE : status,
            message[0] != '\0' ? QString::fromUtf8(message)
                               : QString::fromUtf8(tmv2_status_string(status)));
        qWarning(LOG_THICKNESS_MEASURE_V2) << "tmv2_measure_pairs_average failed:" << message;
        return false;
    }

    out->thicknessMm = average.thickness_mm;
    out->pairCount = average.pair_count;
    out->successCount = average.success_count;
    out->valid = average.valid != 0;
    return true;
}

}  // namespace scan_tracking::thickness_measure_v2
