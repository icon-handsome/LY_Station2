#include "scan_tracking/inner_surface_measure/inner_surface_measure_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>

#include "inner_surface_measure_c_api.h"

Q_LOGGING_CATEGORY(LOG_INNER_SURFACE_MEASURE, "inner_surface_measure.service")

namespace scan_tracking::inner_surface_measure {

namespace {

void FillError(InnerSurfaceMeasureError* error, ism_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(ism_status_string(status))
        : message;
}

void DestroyContext(ism_context*& ctx)
{
    if (ctx != nullptr) {
        ism_destroy(ctx);
        ctx = nullptr;
    }
}

ism_config ToCApiConfig(const InnerSurfaceConfig& src)
{
    ism_config dst;
    ism_config_default(&dst);

    dst.voxel_size = src.voxelSize;
    dst.outlier_k = src.outlierK;
    dst.outlier_std = src.outlierStd;
    dst.fit_iterations = src.fitIterations;
    dst.cylinder_inlier_band = src.cylinderInlierBand;
    dst.section_half_width = src.sectionHalfWidth;
    dst.icp_max_iterations = src.icpMaxIterations;
    dst.icp_max_correspondence_distance = src.icpMaxCorrespondenceDistance;
    dst.icp_transformation_epsilon = src.icpTransformationEpsilon;
    dst.icp_euclidean_fitness_epsilon = src.icpEuclideanFitnessEpsilon;
    dst.cylinder_point_x = src.cylinderPointX;
    dst.cylinder_point_y = src.cylinderPointY;
    dst.cylinder_point_z = src.cylinderPointZ;
    dst.cylinder_axis_x = src.cylinderAxisX;
    dst.cylinder_axis_y = src.cylinderAxisY;
    dst.cylinder_axis_z = src.cylinderAxisZ;
    dst.cylinder_radius = src.cylinderRadius;
    dst.container_length_mm = src.containerLengthMm;
    return dst;
}

void FillFrameResult(const ism_frame_result& src, InnerSurfaceFrameMeasurement* dst)
{
    dst->diameterMm = src.diameter_mm;
    dst->circumferenceMm = src.circumference_mm;
    dst->sectionRoundness[0] = src.section_roundness[0];
    dst->sectionRoundness[1] = src.section_roundness[1];
    dst->sectionRoundness[2] = src.section_roundness[2];
    dst->averageRoundness = src.average_roundness;
    dst->icpFitnessScore = src.icp_fitness_score;
    dst->icpConverged = src.icp_converged != 0;
    dst->usedPointCount = src.used_point_count;
    dst->valid = src.valid != 0;
}

}  // namespace

struct InnerSurfaceMeasureService::Impl {
    mutable std::mutex mutex;
    ism_context* ctx = nullptr;
    QString configPath;
};

InnerSurfaceMeasureService::InnerSurfaceMeasureService()
    : m_impl(new Impl)
{
}

InnerSurfaceMeasureService::~InnerSurfaceMeasureService()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString InnerSurfaceMeasureService::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/inner_surface_measure/config.ini"));
}

bool InnerSurfaceMeasureService::isReady() const
{
    if (m_impl == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->ctx != nullptr;
}

QString InnerSurfaceMeasureService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool InnerSurfaceMeasureService::initializeFromIni(const QString& configPath, InnerSurfaceMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, ISM_ERR_CONFIG, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_INNER_SURFACE_MEASURE) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    ism_context* ctx = nullptr;
    const ism_status status = ism_create_from_ini(pathUtf8.constData(), &ctx);
    if (status != ISM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(ism_status_string(status)));
        qWarning(LOG_INNER_SURFACE_MEASURE) << "ism_create_from_ini failed:" << ism_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    qInfo(LOG_INNER_SURFACE_MEASURE) << "InnerSurfaceMeasure ready, config:" << resolved;
    return true;
}

bool InnerSurfaceMeasureService::initialize(
    const InnerSurfaceConfig& config,
    const float* templateXyz,
    size_t templateCount,
    InnerSurfaceMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    if (templateXyz == nullptr || templateCount == 0) {
        FillError(error, ISM_ERR_INVALID_ARG, QStringLiteral("Template cloud is empty"));
        return false;
    }

    const ism_config apiConfig = ToCApiConfig(config);
    ism_context* ctx = nullptr;
    const ism_status status = ism_create(&apiConfig, templateXyz, templateCount, &ctx);
    if (status != ISM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(ism_status_string(status)));
        qWarning(LOG_INNER_SURFACE_MEASURE) << "ism_create failed:" << ism_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    qInfo(LOG_INNER_SURFACE_MEASURE) << "InnerSurfaceMeasure ready from memory template";
    return true;
}

void InnerSurfaceMeasureService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
}

bool InnerSurfaceMeasureService::measureFrame(
    const float* scanXyz,
    size_t scanCount,
    InnerSurfaceFrameMeasurement* out,
    InnerSurfaceMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, ISM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, ISM_ERR_NOT_INITIALIZED, QStringLiteral("InnerSurfaceMeasureService not initialized"));
        return false;
    }

    ism_frame_result result{};
    char message[512] = {0};
    const ism_status status = ism_measure_frame(
        m_impl->ctx,
        scanXyz,
        scanCount,
        &result,
        message,
        sizeof(message));

    if (status != ISM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(ism_status_string(status)));
        qWarning(LOG_INNER_SURFACE_MEASURE) << "ism_measure_frame failed:" << message;
        return false;
    }

    FillFrameResult(result, out);
    return true;
}

bool InnerSurfaceMeasureService::measureTwoFramesAverage(
    const float* frame1Xyz,
    size_t frame1Count,
    const float* frame2Xyz,
    size_t frame2Count,
    InnerSurfaceAverageMeasurement* outAverage,
    InnerSurfaceFrameMeasurement* outFrame1,
    InnerSurfaceFrameMeasurement* outFrame2,
    InnerSurfaceMeasureError* error)
{
    if (outAverage == nullptr) {
        FillError(error, ISM_ERR_INVALID_ARG, QStringLiteral("outAverage is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, ISM_ERR_NOT_INITIALIZED, QStringLiteral("InnerSurfaceMeasureService not initialized"));
        return false;
    }

    ism_average_result average{};
    ism_frame_result frame1{};
    ism_frame_result frame2{};
    char message[512] = {0};
    const ism_status status = ism_measure_two_frames_average(
        m_impl->ctx,
        frame1Xyz,
        frame1Count,
        frame2Xyz,
        frame2Count,
        &average,
        &frame1,
        &frame2,
        message,
        sizeof(message));

    if (status != ISM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(ism_status_string(status)));
        qWarning(LOG_INNER_SURFACE_MEASURE) << "ism_measure_two_frames_average failed:" << message;
        return false;
    }

    outAverage->diameterMm = average.diameter_mm;
    outAverage->circumferenceMm = average.circumference_mm;
    outAverage->roundness = average.roundness;
    outAverage->volumeLiters = average.volume_liters;
    outAverage->containerLengthMm = average.container_length_mm;
    outAverage->valid = average.valid != 0;

    if (outFrame1 != nullptr) {
        FillFrameResult(frame1, outFrame1);
    }
    if (outFrame2 != nullptr) {
        FillFrameResult(frame2, outFrame2);
    }
    return true;
}

}  // namespace scan_tracking::inner_surface_measure
