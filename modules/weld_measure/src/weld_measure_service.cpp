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

}  // namespace

struct WeldMeasureService::Impl {
    std::mutex mutex;
    wm_context* ctx = nullptr;
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

QString WeldMeasureService::defaultModelPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("models/weld_measure/pointnet_weld_seam_V7.3_good.onnx"));
}

bool WeldMeasureService::isReady() const
{
    return m_impl != nullptr && m_impl->ctx != nullptr;
}

QString WeldMeasureService::modelPath() const
{
    return m_impl != nullptr ? m_impl->modelPath : QString();
}

bool WeldMeasureService::initialize(const QString& modelPath, WeldMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
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
    m_impl->modelPath.clear();
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

    out->mismatchMm = result.mismatch_mm;
    out->reinforcementMm = result.reinforcement_mm;
    out->angularityMm = result.angularity_mm;
    out->includedAngleRad = result.included_angle_rad;
    out->toeCenterX = result.toe_center_x;
    out->leftToeX = result.left_toe_x;
    out->leftToeY = result.left_toe_y;
    out->leftToeZ = result.left_toe_z;
    out->rightToeX = result.right_toe_x;
    out->rightToeY = result.right_toe_y;
    out->rightToeZ = result.right_toe_z;
    out->leftUndercutMm = result.left_undercut_mm;
    out->rightUndercutMm = result.right_undercut_mm;
    out->maxUndercutMm = result.max_undercut_mm;
    out->undercutRawFlag = result.undercut_raw_flag;
    out->valid = result.valid != 0;
    return true;
}

}  // namespace scan_tracking::weld_measure
