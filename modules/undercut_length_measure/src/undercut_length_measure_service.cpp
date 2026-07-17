#include "scan_tracking/undercut_length_measure/undercut_length_measure_service.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>
#include <vector>

#include "undercut_length_measure_c_api.h"

Q_LOGGING_CATEGORY(LOG_UNDERCUT_LENGTH_MEASURE, "undercut_length_measure.service")

namespace scan_tracking::undercut_length_measure {

namespace {

void FillError(UndercutLengthMeasureError* error, ulm_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    error->message = message.isEmpty()
        ? QString::fromUtf8(ulm_status_string(status))
        : message;
}

void DestroyContext(ulm_context*& ctx)
{
    if (ctx != nullptr) {
        ulm_destroy(ctx);
        ctx = nullptr;
    }
}

ulm_options ToCApiOptions(const UndercutLengthOptions& src)
{
    ulm_options dst;
    ulm_options_default(&dst);
    dst.section_interval_mm = src.sectionIntervalMm;
    dst.depth_threshold_mm = src.depthThresholdMm;
    dst.median_filter_window = src.medianFilterWindow;
    return dst;
}

void FillOptions(const ulm_options& src, UndercutLengthOptions* dst)
{
    dst->sectionIntervalMm = src.section_interval_mm;
    dst->depthThresholdMm = src.depth_threshold_mm;
    dst->medianFilterWindow = src.median_filter_window;
}

void FillResult(const ulm_result& src, UndercutLengthMeasurement* dst)
{
    dst->left.maxDepthMm = src.left.max_depth_mm;
    dst->left.lengthMm = src.left.length_mm;
    dst->left.maxContinuousSections = src.left.max_continuous_sections;
    dst->right.maxDepthMm = src.right.max_depth_mm;
    dst->right.lengthMm = src.right.length_mm;
    dst->right.maxContinuousSections = src.right.max_continuous_sections;
    dst->valid = src.valid != 0;
}

std::vector<ulm_section> ToCApiSections(const QVector<UndercutSectionInput>& sections)
{
    std::vector<ulm_section> output(static_cast<size_t>(sections.size()));
    for (int i = 0; i < sections.size(); ++i) {
        ulm_section& dst = output[static_cast<size_t>(i)];
        const UndercutSectionInput& src = sections[i];
        dst.position = src.position;
        dst.left_toe.x = src.leftToe.x;
        dst.left_toe.y = src.leftToe.y;
        dst.left_toe.z = src.leftToe.z;
        dst.left_depth_mm = src.leftDepthMm;
        dst.right_toe.x = src.rightToe.x;
        dst.right_toe.y = src.rightToe.y;
        dst.right_toe.z = src.rightToe.z;
        dst.right_depth_mm = src.rightDepthMm;
    }
    return output;
}

}  // namespace

struct UndercutLengthMeasureService::Impl {
    std::mutex mutex;
    ulm_context* ctx = nullptr;
    QString configPath;
};

UndercutLengthMeasureService::UndercutLengthMeasureService()
    : m_impl(new Impl)
{
}

UndercutLengthMeasureService::~UndercutLengthMeasureService()
{
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString UndercutLengthMeasureService::defaultConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/undercut_length_measure/undercut_length.ini"));
}

bool UndercutLengthMeasureService::isReady() const
{
    return m_impl != nullptr && m_impl->ctx != nullptr;
}

QString UndercutLengthMeasureService::configPath() const
{
    return m_impl != nullptr ? m_impl->configPath : QString();
}

bool UndercutLengthMeasureService::initialize(
    const UndercutLengthOptions& options,
    UndercutLengthMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    const ulm_options apiOptions = ToCApiOptions(options);
    ulm_context* ctx = nullptr;
    const ulm_status status = ulm_create(&apiOptions, &ctx);
    if (status != ULM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(ulm_status_string(status)));
        qWarning(LOG_UNDERCUT_LENGTH_MEASURE) << "ulm_create failed:" << ulm_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    qInfo(LOG_UNDERCUT_LENGTH_MEASURE) << "UndercutLengthMeasure ready from options";
    return true;
}

bool UndercutLengthMeasureService::initializeFromIni(
    const QString& configPath,
    UndercutLengthMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, ULM_ERR_CONFIG, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_UNDERCUT_LENGTH_MEASURE) << "Config missing:" << resolved;
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    ulm_context* ctx = nullptr;
    const ulm_status status = ulm_create_from_ini(pathUtf8.constData(), &ctx);
    if (status != ULM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(ulm_status_string(status)));
        qWarning(LOG_UNDERCUT_LENGTH_MEASURE) << "ulm_create_from_ini failed:" << ulm_status_string(status);
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    qInfo(LOG_UNDERCUT_LENGTH_MEASURE) << "UndercutLengthMeasure ready, config:" << resolved;
    return true;
}

bool UndercutLengthMeasureService::setOptions(
    const UndercutLengthOptions& options,
    UndercutLengthMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, ULM_ERR_NOT_INITIALIZED, QStringLiteral("UndercutLengthMeasureService not initialized"));
        return false;
    }

    const ulm_options apiOptions = ToCApiOptions(options);
    const ulm_status status = ulm_set_options(m_impl->ctx, &apiOptions);
    if (status != ULM_OK) {
        FillError(error, status, QString::fromUtf8(ulm_status_string(status)));
        return false;
    }
    return true;
}

bool UndercutLengthMeasureService::getOptions(
    UndercutLengthOptions* out,
    UndercutLengthMeasureError* error) const
{
    if (out == nullptr) {
        FillError(error, ULM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, ULM_ERR_NOT_INITIALIZED, QStringLiteral("UndercutLengthMeasureService not initialized"));
        return false;
    }

    ulm_options apiOptions{};
    const ulm_status status = ulm_get_options(m_impl->ctx, &apiOptions);
    if (status != ULM_OK) {
        FillError(error, status, QString::fromUtf8(ulm_status_string(status)));
        return false;
    }

    FillOptions(apiOptions, out);
    return true;
}

void UndercutLengthMeasureService::shutdown()
{
    if (m_impl == nullptr) {
        return;
    }
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
}

bool UndercutLengthMeasureService::measure(
    const QVector<UndercutSectionInput>& sections,
    UndercutLengthMeasurement* out,
    UndercutLengthMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, ULM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }
    if (sections.isEmpty()) {
        FillError(error, ULM_ERR_SECTIONS, QStringLiteral("sections is empty"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, ULM_ERR_NOT_INITIALIZED, QStringLiteral("UndercutLengthMeasureService not initialized"));
        return false;
    }

    const std::vector<ulm_section> apiSections = ToCApiSections(sections);
    ulm_result result{};
    char message[512] = {0};
    const ulm_status status = ulm_measure(
        m_impl->ctx,
        apiSections.data(),
        apiSections.size(),
        &result,
        message,
        sizeof(message));

    if (status != ULM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(ulm_status_string(status)));
        qWarning(LOG_UNDERCUT_LENGTH_MEASURE) << "ulm_measure failed:" << message;
        return false;
    }

    FillResult(result, out);
    return true;
}

bool UndercutLengthMeasureService::measureFromFile(
    const QString& sectionsPath,
    UndercutLengthMeasurement* out,
    UndercutLengthMeasureError* error)
{
    if (out == nullptr) {
        FillError(error, ULM_ERR_INVALID_ARG, QStringLiteral("out is null"));
        return false;
    }
    if (sectionsPath.isEmpty()) {
        FillError(error, ULM_ERR_INVALID_ARG, QStringLiteral("sectionsPath is empty"));
        return false;
    }

    std::lock_guard<std::mutex> lock(m_impl->mutex);
    if (m_impl->ctx == nullptr) {
        FillError(error, ULM_ERR_NOT_INITIALIZED, QStringLiteral("UndercutLengthMeasureService not initialized"));
        return false;
    }

    const QByteArray pathUtf8 = QDir::toNativeSeparators(sectionsPath).toUtf8();
    ulm_result result{};
    char message[512] = {0};
    const ulm_status status = ulm_measure_from_file(
        m_impl->ctx,
        pathUtf8.constData(),
        &result,
        message,
        sizeof(message));

    if (status != ULM_OK) {
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(ulm_status_string(status)));
        qWarning(LOG_UNDERCUT_LENGTH_MEASURE) << "ulm_measure_from_file failed:" << message;
        return false;
    }

    FillResult(result, out);
    return true;
}

}  // namespace scan_tracking::undercut_length_measure
