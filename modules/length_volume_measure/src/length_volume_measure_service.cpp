#include "scan_tracking/length_volume_measure/length_volume_measure_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <algorithm>
#include <cstring>
#include <mutex>

#include "length_volume_measure_c_api.h"

Q_LOGGING_CATEGORY(LOG_LENGTH_VOLUME_MEASURE, "length_volume_measure.service")

namespace scan_tracking::length_volume_measure {

namespace {

void FillError(LengthVolumeMeasureError* error, lvm_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(lvm_status_string(status))
        : message;
}

void DestroyContext(lvm_context*& ctx)
{
    if (ctx != nullptr) {
        lvm_destroy(ctx);
        ctx = nullptr;
    }
}

lvm_config ToCApiConfig(const LengthVolumeConfig& src)
{
    lvm_config dst;
    lvm_config_default(&dst);

    dst.crop_input_cloud = src.cropInputCloud ? 1 : 0;
    dst.crop_min_x = src.cropMinX;
    dst.crop_min_y = src.cropMinY;
    dst.crop_min_z = src.cropMinZ;
    dst.crop_max_x = src.cropMaxX;
    dst.crop_max_y = src.cropMaxY;
    dst.crop_max_z = src.cropMaxZ;

    dst.axis_direction_x = src.axisDirectionX;
    dst.axis_direction_y = src.axisDirectionY;
    dst.axis_direction_z = src.axisDirectionZ;
    dst.end_point1_x = src.endPoint1X;
    dst.end_point1_y = src.endPoint1Y;
    dst.end_point1_z = src.endPoint1Z;
    dst.end_point2_x = src.endPoint2X;
    dst.end_point2_y = src.endPoint2Y;
    dst.end_point2_z = src.endPoint2Z;

    dst.voxel_size = src.voxelSize;
    dst.outlier_k = src.outlierK;
    dst.outlier_std = src.outlierStd;

    dst.icp_max_iterations = src.icpMaxIterations;
    dst.icp_max_correspondence_distance = src.icpMaxCorrespondenceDistance;
    dst.icp_transformation_epsilon = src.icpTransformationEpsilon;
    dst.icp_euclidean_fitness_epsilon = src.icpEuclideanFitnessEpsilon;

    dst.cylinder_fit_iterations = src.cylinderFitIterations;
    dst.cylinder_inlier_band = src.cylinderInlierBand;
    dst.update_cylinder_axis = src.updateCylinderAxis ? 1 : 0;

    dst.normal_k = src.normalK;
    dst.end_normal_min_abs_dot = src.endNormalMinAbsDot;

    const QByteArray methodUtf8 = src.endpointDetectionMethod.toUtf8();
    std::memset(dst.endpoint_detection_method, 0, sizeof(dst.endpoint_detection_method));
    const size_t copyLen = std::min(
        static_cast<size_t>(methodUtf8.size()),
        sizeof(dst.endpoint_detection_method) - 1);
    if (copyLen > 0) {
        std::memcpy(dst.endpoint_detection_method, methodUtf8.constData(), copyLen);
    }

    dst.axial_bin_width = src.axialBinWidth;
    dst.end_search_half_width = src.endSearchHalfWidth;
    dst.outside_scan_peak_search_width = src.outsideScanPeakSearchWidth;
    dst.min_points_per_bin = src.minPointsPerBin;
    dst.min_consecutive_inside_bins = src.minConsecutiveInsideBins;
    dst.outside_check_bins = src.outsideCheckBins;
    dst.refine_half_width = src.refineHalfWidth;
    dst.refine_edge_percentile = src.refineEdgePercentile;
    dst.endpoint_max_radius = src.endpointMaxRadius;
    return dst;
}

void FillResult(const lvm_result& src, LengthVolumeMeasurement* dst)
{
    dst->lengthMm = src.length_mm;
    dst->volumeLiters = src.volume_liters;
    dst->volumeRadiusMm = src.volume_radius_mm;
    dst->fittedOuterRadiusMm = src.fitted_outer_radius_mm;

    dst->cylinderPointX = src.cylinder_point_x;
    dst->cylinderPointY = src.cylinder_point_y;
    dst->cylinderPointZ = src.cylinder_point_z;
    dst->cylinderAxisX = src.cylinder_axis_x;
    dst->cylinderAxisY = src.cylinder_axis_y;
    dst->cylinderAxisZ = src.cylinder_axis_z;

    dst->end1.templatePos = src.end1_template_pos;
    dst->end1.coarsePos = src.end1_coarse_pos;
    dst->end1.refinedPos = src.end1_refined_pos;
    dst->end1.usedEndNormals = src.end1_used_end_normals != 0;
    dst->end1.coarsePointCount = src.end1_coarse_point_count;
    dst->end1.refinedPointCount = src.end1_refined_point_count;

    dst->end2.templatePos = src.end2_template_pos;
    dst->end2.coarsePos = src.end2_coarse_pos;
    dst->end2.refinedPos = src.end2_refined_pos;
    dst->end2.usedEndNormals = src.end2_used_end_normals != 0;
    dst->end2.coarsePointCount = src.end2_coarse_point_count;
    dst->end2.refinedPointCount = src.end2_refined_point_count;

    dst->icpFitnessScore = src.icp_fitness_score;
    dst->icpConverged = src.icp_converged != 0;
    dst->valid = src.valid != 0;
}

}  // namespace

struct LengthVolumeMeasureService::Impl {
    std::mutex mutex;
    lvm_context* ctx = nullptr;
    QString configPath;
};

LengthVolumeMeasureService::LengthVolumeMeasureService()
    : m_impl(new Impl)
{
}

LengthVolumeMeasureService::~LengthVolumeMeasureService()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString LengthVolumeMeasureService::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/length_volume_measure/config.ini"));
}

bool LengthVolumeMeasureService::isReady() const
{
    return m_impl != nullptr && m_impl->ctx != nullptr;
}

QString LengthVolumeMeasureService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool LengthVolumeMeasureService::initializeFromIni(const QString& configPath, LengthVolumeMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, LVM_ERR_CONFIG, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_LENGTH_VOLUME_MEASURE) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    lvm_context* ctx = nullptr;
    const lvm_status status = lvm_create_from_ini(pathUtf8.constData(), &ctx);
    if (status != LVM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(lvm_status_string(status)));
        qWarning(LOG_LENGTH_VOLUME_MEASURE) << "lvm_create_from_ini failed:" << lvm_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    qInfo(LOG_LENGTH_VOLUME_MEASURE) << "LengthVolumeMeasure ready, config:" << resolved;
    return true;
}

bool LengthVolumeMeasureService::initialize(
    const LengthVolumeConfig& config,
    const float* templateXyz,
    size_t templateCount,
    LengthVolumeMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    if (templateXyz == nullptr || templateCount == 0) {
        FillError(error, LVM_ERR_INVALID_ARG, QStringLiteral("Template cloud is empty"));
        return false;
    }

    const lvm_config apiConfig = ToCApiConfig(config);
    lvm_context* ctx = nullptr;
    const lvm_status status = lvm_create(&apiConfig, templateXyz, templateCount, &ctx);
    if (status != LVM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(lvm_status_string(status)));
        qWarning(LOG_LENGTH_VOLUME_MEASURE) << "lvm_create failed:" << lvm_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    qInfo(LOG_LENGTH_VOLUME_MEASURE) << "LengthVolumeMeasure ready from memory template";
    return true;
}

void LengthVolumeMeasureService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
}

bool LengthVolumeMeasureService::measure(
    const float* scanXyz,
    size_t scanCount,
    double volumeRadiusMm,
    LengthVolumeMeasurement* out,
    LengthVolumeMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, LVM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, LVM_ERR_NOT_INITIALIZED, QStringLiteral("LengthVolumeMeasureService not initialized"));
        return false;
    }

    lvm_result result{};
    char message[512] = {0};
    const lvm_status status = lvm_measure(
        m_impl->ctx,
        scanXyz,
        scanCount,
        volumeRadiusMm,
        &result,
        message,
        sizeof(message));

    if (status != LVM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(lvm_status_string(status)));
        qWarning(LOG_LENGTH_VOLUME_MEASURE) << "lvm_measure failed:" << message;
        return false;
    }

    FillResult(result, out);
    return true;
}

}  // namespace scan_tracking::length_volume_measure
