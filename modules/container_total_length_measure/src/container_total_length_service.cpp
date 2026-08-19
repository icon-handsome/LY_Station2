#include "scan_tracking/container_total_length_measure/container_total_length_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>

#include "ContainerTotalLengthApi.h"

Q_LOGGING_CATEGORY(LOG_CONTAINER_TOTAL_LENGTH, "container_total_length_measure.service")

namespace scan_tracking::container_total_length_measure {

namespace {

void FillError(ContainerTotalLengthError* error, ctl_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(ctl_status_string(status))
        : message;
}

void DestroyContext(ctl_context*& ctx)
{
    if (ctx != nullptr) {
        ctl_destroy(ctx);
        ctx = nullptr;
    }
}

void FillResult(const ctl_result& src, ContainerTotalLengthMeasurement* dst)
{
    if (dst == nullptr) {
        return;
    }
    dst->lengthMm = src.length_mm;
    dst->leftEndPosition = src.left_end_position;
    dst->rightEndPosition = src.right_end_position;
    dst->icpFitness = src.icp_fitness;
    dst->fittedRadiusMm = src.fitted_radius_mm;
    dst->cylinderPointX = src.cylinder_point_x;
    dst->cylinderPointY = src.cylinder_point_y;
    dst->cylinderPointZ = src.cylinder_point_z;
    dst->cylinderAxisX = src.cylinder_axis_x;
    dst->cylinderAxisY = src.cylinder_axis_y;
    dst->cylinderAxisZ = src.cylinder_axis_z;
    dst->icpConverged = src.icp_converged != 0;
    dst->inputPointCount = src.input_point_count;
    dst->valid = src.valid != 0;
}

}  // namespace

struct ContainerTotalLengthService::Impl {
    mutable std::mutex mutex;
    ctl_context* ctx = nullptr;
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
    return m_impl->ctx != nullptr;
}

QString ContainerTotalLengthService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool ContainerTotalLengthService::initializeFromIni(const QString& configPath, ContainerTotalLengthError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, CTL_ERR_CONFIG, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_CONTAINER_TOTAL_LENGTH) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    ctl_context* ctx = nullptr;
    char message[512] = {0};
    const ctl_status status = ctl_create_from_ini(
        pathUtf8.constData(), &ctx, message, sizeof(message));
    if (status != CTL_OK || ctx == nullptr) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(ctl_status_string(status)));
        qWarning(LOG_CONTAINER_TOTAL_LENGTH) << "ctl_create_from_ini failed:" << message;
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    qInfo(LOG_CONTAINER_TOTAL_LENGTH) << "ContainerTotalLength ready, config:" << resolved;
    return true;
}

void ContainerTotalLengthService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
}

bool ContainerTotalLengthService::measure(
    const float* scanXyz,
    size_t scanCount,
    ContainerTotalLengthMeasurement* out,
    ContainerTotalLengthError* error)
{
    if (out == nullptr) {
        FillError(error, CTL_ERR_INVALID_ARGUMENT, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, CTL_ERR_NOT_INITIALIZED, QStringLiteral("ContainerTotalLengthService not initialized"));
        return false;
    }

    ctl_result result{};
    char message[512] = {0};
    const ctl_status status = ctl_measure(
        m_impl->ctx,
        scanXyz,
        scanCount,
        &result,
        message,
        sizeof(message));

    if (status != CTL_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(ctl_status_string(status)));
        qWarning(LOG_CONTAINER_TOTAL_LENGTH) << "ctl_measure failed:" << message;
        return false;
    }

    FillResult(result, out);
    return true;
}

}  // namespace scan_tracking::container_total_length_measure
