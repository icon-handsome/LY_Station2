#include "scan_tracking/thickness_measure/thickness_measure_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>
#include <vector>

#include "thickness_measure_c_api.h"

Q_LOGGING_CATEGORY(LOG_THICKNESS_MEASURE, "thickness_measure.service")

namespace scan_tracking::thickness_measure {

namespace {

void FillError(ThicknessMeasureError* error, tm_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(tm_status_string(status))
        : message;
}

void DestroyContext(tm_context*& ctx)
{
    if (ctx != nullptr) {
        tm_destroy(ctx);
        ctx = nullptr;
    }
}

tm_config ToCApiConfig(const ThicknessConfig& src)
{
    tm_config dst;
    tm_config_default(&dst);

    dst.preprocess.enable_outlier_removal = src.preprocess.enableOutlierRemoval ? 1 : 0;
    dst.preprocess.mean_k = src.preprocess.meanK;
    dst.preprocess.stddev_mul_thresh = src.preprocess.stddevMulThresh;
    dst.preprocess.enable_voxel_downsample = src.preprocess.enableVoxelDownsample ? 1 : 0;
    dst.preprocess.leaf_size = src.preprocess.leafSize;

    dst.icp.max_iterations = src.icp.maxIterations;
    dst.icp.max_correspondence_distance = src.icp.maxCorrespondenceDistance;
    dst.icp.transformation_epsilon = src.icp.transformationEpsilon;
    dst.icp.euclidean_fitness_epsilon = src.icp.euclideanFitnessEpsilon;

    dst.thickness_method = (src.thicknessMethod == ThicknessMethod::TangentPlaneProjection)
        ? TM_METHOD_TANGENT_PLANE_PROJECTION
        : TM_METHOD_NEAREST_BETWEEN_SURFACES;

    dst.axis_point.x = src.axisPoint.x;
    dst.axis_point.y = src.axisPoint.y;
    dst.axis_point.z = src.axisPoint.z;
    dst.axis_direction.x = src.axisDirection.x;
    dst.axis_direction.y = src.axisDirection.y;
    dst.axis_direction.z = src.axisDirection.z;

    for (int i = 0; i < 2; ++i) {
        dst.template_feature_points[i].x = src.templateFeaturePoints[i].x;
        dst.template_feature_points[i].y = src.templateFeaturePoints[i].y;
        dst.template_feature_points[i].z = src.templateFeaturePoints[i].z;
    }
    return dst;
}

void FillPairResult(const tm_pair_result& src, ThicknessPairMeasurement* dst)
{
    dst->innerIcpFitnessScore = src.inner_icp_fitness_score;
    dst->outerIcpFitnessScore = src.outer_icp_fitness_score;
    dst->thicknessMm = src.thickness_mm;
    dst->thicknessMethod = (src.thickness_method == TM_METHOD_TANGENT_PLANE_PROJECTION)
        ? ThicknessMethod::TangentPlaneProjection
        : ThicknessMethod::NearestBetweenSurfaces;
    for (int i = 0; i < 2; ++i) {
        dst->templateFeaturePoints[i].x = src.template_feature_points[i].x;
        dst->templateFeaturePoints[i].y = src.template_feature_points[i].y;
        dst->templateFeaturePoints[i].z = src.template_feature_points[i].z;
        dst->nearestScanPoints[i].x = src.nearest_scan_points[i].x;
        dst->nearestScanPoints[i].y = src.nearest_scan_points[i].y;
        dst->nearestScanPoints[i].z = src.nearest_scan_points[i].z;
        dst->projectedPoints[i].x = src.projected_points[i].x;
        dst->projectedPoints[i].y = src.projected_points[i].y;
        dst->projectedPoints[i].z = src.projected_points[i].z;
    }
    dst->valid = src.valid != 0;
}

}  // namespace

struct ThicknessMeasureService::Impl {
    mutable std::mutex mutex;
    tm_context* ctx = nullptr;
    QString configPath;
};

ThicknessMeasureService::ThicknessMeasureService()
    : m_impl(new Impl)
{
}

ThicknessMeasureService::~ThicknessMeasureService()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString ThicknessMeasureService::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/thickness_measure/thickness_config.json"));
}

bool ThicknessMeasureService::isReady() const
{
    if (m_impl == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->ctx != nullptr;
}

QString ThicknessMeasureService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool ThicknessMeasureService::initializeFromJson(const QString& configPath, ThicknessMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, TM_ERR_CONFIG, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_THICKNESS_MEASURE) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    tm_context* ctx = nullptr;
    const tm_status status = tm_create_from_json(pathUtf8.constData(), &ctx);
    if (status != TM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(tm_status_string(status)));
        qWarning(LOG_THICKNESS_MEASURE) << "tm_create_from_json failed:" << tm_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    qInfo(LOG_THICKNESS_MEASURE) << "ThicknessMeasure ready, config:" << resolved;
    return true;
}

bool ThicknessMeasureService::initialize(
    const ThicknessConfig& config,
    const float* innerTemplateXyz,
    size_t innerTemplateCount,
    const float* outerTemplateXyz,
    size_t outerTemplateCount,
    ThicknessMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    if (innerTemplateXyz == nullptr || outerTemplateXyz == nullptr ||
        innerTemplateCount == 0 || outerTemplateCount == 0) {
        FillError(error, TM_ERR_INVALID_ARG, QStringLiteral("Template clouds are empty"));
        return false;
    }

    const tm_config apiConfig = ToCApiConfig(config);
    tm_context* ctx = nullptr;
    const tm_status status = tm_create(
        &apiConfig,
        innerTemplateXyz,
        innerTemplateCount,
        outerTemplateXyz,
        outerTemplateCount,
        &ctx);
    if (status != TM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(tm_status_string(status)));
        qWarning(LOG_THICKNESS_MEASURE) << "tm_create failed:" << tm_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    qInfo(LOG_THICKNESS_MEASURE) << "ThicknessMeasure ready from memory templates";
    return true;
}

void ThicknessMeasureService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
}

bool ThicknessMeasureService::measurePair(
    const float* innerScanXyz,
    size_t innerScanCount,
    const float* outerScanXyz,
    size_t outerScanCount,
    ThicknessPairMeasurement* out,
    ThicknessMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, TM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, TM_ERR_NOT_INITIALIZED, QStringLiteral("ThicknessMeasureService not initialized"));
        return false;
    }

    tm_pair_result result{};
    char message[512] = {0};
    const tm_status status = tm_measure_pair(
        m_impl->ctx,
        innerScanXyz,
        innerScanCount,
        outerScanXyz,
        outerScanCount,
        &result,
        message,
        sizeof(message));

    if (status != TM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(tm_status_string(status)));
        qWarning(LOG_THICKNESS_MEASURE) << "tm_measure_pair failed:" << message;
        return false;
    }

    FillPairResult(result, out);
    return true;
}

bool ThicknessMeasureService::measurePairsAverage(
    const QVector<ThicknessPairClouds>& pairs,
    ThicknessAverageMeasurement* out,
    ThicknessMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, TM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }
    if (pairs.isEmpty()) {
        FillError(error, TM_ERR_INVALID_ARG, QStringLiteral("pairs is empty"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, TM_ERR_NOT_INITIALIZED, QStringLiteral("ThicknessMeasureService not initialized"));
        return false;
    }

    std::vector<tm_pair_clouds> apiPairs(static_cast<size_t>(pairs.size()));
    for (int i = 0; i < pairs.size(); ++i) {
        apiPairs[static_cast<size_t>(i)].inner.xyz = pairs[i].inner.xyz;
        apiPairs[static_cast<size_t>(i)].inner.point_count = pairs[i].inner.pointCount;
        apiPairs[static_cast<size_t>(i)].outer.xyz = pairs[i].outer.xyz;
        apiPairs[static_cast<size_t>(i)].outer.point_count = pairs[i].outer.pointCount;
    }

    tm_average_result result{};
    char message[512] = {0};
    const tm_status status = tm_measure_pairs_average(
        m_impl->ctx,
        apiPairs.data(),
        apiPairs.size(),
        &result,
        message,
        sizeof(message));

    if (status != TM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(tm_status_string(status)));
        qWarning(LOG_THICKNESS_MEASURE) << "tm_measure_pairs_average failed:" << message;
        return false;
    }

    out->thicknessMm = result.thickness_mm;
    out->pairCount = result.pair_count;
    out->successCount = result.success_count;
    out->valid = result.valid != 0;
    return true;
}

}  // namespace scan_tracking::thickness_measure
