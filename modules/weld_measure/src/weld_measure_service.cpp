#include "scan_tracking/weld_measure/weld_measure_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>

#include "weld_measure_c_api.h"

Q_LOGGING_CATEGORY(LOG_WELD_MEASURE, "weld_measure.service")

namespace scan_tracking::weld_measure {

namespace {

void FillError(WeldMeasureError* error, wm_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(wm_status_string(status))
        : message;
}

void DestroyContext(wm_context*& ctx)
{
    if (ctx != nullptr) {
        wm_destroy(ctx);
        ctx = nullptr;
    }
}

void FillSection(const wm_section_result& src, WeldSectionMeasurement* dst)
{
    dst->mismatchMm = src.mismatch_mm;
    dst->reinforcementMm = src.reinforcement_mm;
    dst->angularityMm = src.angularity_mm;
    dst->includedAngleRad = src.included_angle_rad;
    dst->toeCenterX = src.toe_center_x;
    dst->leftToeX = src.left_toe_x;
    dst->leftToeY = src.left_toe_y;
    dst->leftToeZ = src.left_toe_z;
    dst->rightToeX = src.right_toe_x;
    dst->rightToeY = src.right_toe_y;
    dst->rightToeZ = src.right_toe_z;
    dst->leftUndercutMm = src.left_undercut_mm;
    dst->rightUndercutMm = src.right_undercut_mm;
    dst->maxUndercutMm = src.max_undercut_mm;
    dst->undercutRawFlag = src.undercut_raw_flag;
    dst->valid = src.valid != 0;
}

}  // namespace

struct WeldMeasureService::Impl {
    mutable std::mutex mutex;
    wm_context* ctx = nullptr;
    QString configPath;
    QString modelPath;
};

WeldMeasureService::WeldMeasureService()
    : m_impl(new Impl)
{
}

WeldMeasureService::~WeldMeasureService()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString WeldMeasureService::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/weld_measure/weld_measurement.ini"));
}

QString WeldMeasureService::defaultArmConfigPath()
{
    const QDir dir(QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/weld_measure")));
    const QString ascii = dir.filePath(QStringLiteral("weld_measurement-arm.ini"));
    if (QFileInfo::exists(ascii)) {
        return ascii;
    }
    return dir.filePath(QStringLiteral("weld_measurement-机械臂直焊缝.ini"));
}

QString WeldMeasureService::defaultTelescopicConfigPath()
{
    const QDir dir(QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("config/weld_measure")));
    const QString ascii = dir.filePath(QStringLiteral("weld_measurement-telescopic.ini"));
    if (QFileInfo::exists(ascii)) {
        return ascii;
    }
    return dir.filePath(QStringLiteral("weld_measurement-伸缩杆直焊缝.ini"));
}

QString WeldMeasureService::defaultModelPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("models/weld_measure/pointnet_weld_seam_V7.3_good.onnx"));
}

bool WeldMeasureService::isReady() const
{
    if (m_impl == nullptr) {
        return false;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    return m_impl->ctx != nullptr;
}

QString WeldMeasureService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

QString WeldMeasureService::modelPath() const
{
    return m_impl != nullptr ? m_impl->modelPath : QString();
}

bool WeldMeasureService::initializeFromIni(const QString& configPath, WeldMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
    m_impl->modelPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, WM_ERR_MODEL, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_WELD_MEASURE) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    wm_context* ctx = nullptr;
    const wm_status status = wm_create_from_ini(pathUtf8.constData(), &ctx);
    if (status != WM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(wm_status_string(status)));
        qWarning(LOG_WELD_MEASURE) << "wm_create_from_ini failed:" << wm_status_string(status)
                                   << "ini=" << resolved;
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    m_impl->modelPath = defaultModelPath();
    qInfo(LOG_WELD_MEASURE) << "WeldMeasure ready from ini:" << resolved;
    return true;
}

bool WeldMeasureService::initialize(const QString& modelPath, WeldMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
    m_impl->modelPath.clear();

    const QString resolved = modelPath.isEmpty() ? defaultModelPath() : modelPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, WM_ERR_MODEL, QStringLiteral("ONNX model not found: %1").arg(resolved));
        qWarning(LOG_WELD_MEASURE) << "Model missing:" << resolved;
        return false;
    }

    const QByteArray modelUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    wm_onnx_config onnx;
    wm_onnx_config_default(&onnx);
    onnx.model_path = modelUtf8.constData();

    wm_context* ctx = nullptr;
    const wm_status status = wm_create(&onnx, &ctx);
    if (status != WM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(wm_status_string(status)));
        qWarning(LOG_WELD_MEASURE) << "wm_create failed:" << wm_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->modelPath = resolved;
    qInfo(LOG_WELD_MEASURE) << "WeldMeasure ready, model:" << resolved;
    return true;
}

void WeldMeasureService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
    m_impl->modelPath.clear();
}

bool WeldMeasureService::measureFrame(
    int frameIndex1Based,
    const float* xyz,
    size_t pointCount,
    WeldFrameMeasurement* out,
    WeldMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, WM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, WM_ERR_NOT_INITIALIZED, QStringLiteral("WeldMeasureService not initialized"));
        return false;
    }

    wm_frame_result result{};
    char message[512] = {0};
    const wm_status status = wm_measure_frame(
        m_impl->ctx,
        frameIndex1Based,
        xyz,
        pointCount,
        nullptr,
        &result,
        message,
        sizeof(message));

    if (status != WM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(wm_status_string(status)));
        qWarning(LOG_WELD_MEASURE) << "wm_measure_frame failed:" << message
                                   << "frame=" << frameIndex1Based;
        return false;
    }

    FillSection(result.average, &out->average);
    out->validSections = result.valid_sections;
    out->totalSections = result.total_sections;
    out->leftUndercutLengthMm = result.left_undercut_length_mm;
    out->rightUndercutLengthMm = result.right_undercut_length_mm;
    out->leftMaxUndercutDepthMm = result.left_max_undercut_depth_mm;
    out->rightMaxUndercutDepthMm = result.right_max_undercut_depth_mm;
    return true;
}

bool WeldMeasureService::measureSection(
    const float* xyz,
    size_t pointCount,
    WeldSectionMeasurement* out,
    WeldMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, WM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, WM_ERR_NOT_INITIALIZED, QStringLiteral("WeldMeasureService not initialized"));
        return false;
    }

    wm_options options;
    wm_options_default(&options);

    wm_section_result result{};
    char message[512] = {0};
    const wm_status status = wm_measure_section(
        m_impl->ctx,
        xyz,
        pointCount,
        &options,
        &result,
        message,
        sizeof(message));

    if (status != WM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(wm_status_string(status)));
        qWarning(LOG_WELD_MEASURE) << "wm_measure_section failed:" << message;
        return false;
    }

    FillSection(result, out);
    return true;
}

}  // namespace scan_tracking::weld_measure
