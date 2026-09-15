#include "scan_tracking/weld_measure/weld_measure_service.h"

#include "weld_measure_sdk_seh.h"

#include <QtCore/QCoreApplication>
#include <QtCore/QDir>
#include <QtCore/QFileInfo>
#include <QtCore/QLoggingCategory>

#include <mutex>

#include "weld_measure_c_api.h"

Q_LOGGING_CATEGORY(LOG_WELD_MEASURE, "weld_measure.service")

namespace scan_tracking::weld_measure {

namespace {

/// 将 DLL 状态码与文案写入可选的错误输出；error 为空时直接返回。
void FillError(WeldMeasureError* error, wm_status status, const QString& message)
{
    if (error == nullptr) {
        return;
    }
    error->statusCode = static_cast<int>(status);
    // 调用方未给文案时，回退到 DLL 自带的状态字符串。
    error->message = message.isEmpty()
        ? QString::fromUtf8(wm_status_string(status))
        : message;
}

/// 安全销毁 DLL 上下文：经 SEH 包装调用 wm_destroy，并将句柄置空。
/// 若 destroy 过程触发原生异常，仅记日志；调用方已失去可用上下文。
void DestroyContext(wm_context*& ctx)
{
    if (ctx != nullptr) {
        const unsigned nativeFault = sdk_seh::destroy(ctx);
        if (nativeFault != 0) {
            qCritical(LOG_WELD_MEASURE).noquote()
                << QStringLiteral("WeldMeasure destroy 原生异常 SEH=0x%1，服务已隔离。")
                       .arg(nativeFault, 8, 16, QLatin1Char('0'));
        }
        ctx = nullptr;
    }
}

/// 统一生成「原生异常后需重启 IPC」类提示文案。
QString NativeFaultMessage(const char* operation, unsigned nativeFault)
{
    return QStringLiteral("WeldMeasure %1 原生异常 SEH=0x%2；本进程内算法服务已隔离，请重启IPC。")
        .arg(QString::fromLatin1(operation))
        .arg(nativeFault, 8, 16, QLatin1Char('0'));
}

/// 将 C API 的 wm_section_result 拷贝为业务侧 WeldSectionMeasurement。
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

/// 实例私有状态：互斥锁 + DLL 句柄 + 最近一次初始化路径缓存。
struct WeldMeasureService::Impl {
    mutable std::mutex mutex;   ///< 保护 ctx 与路径字段；同一实例串行进入 DLL
    wm_context* ctx = nullptr;  ///< WeldMeasure.dll 不透明上下文，未初始化时为空
    QString configPath;         ///< initializeFromIni 成功后的 ini 绝对/解析路径
    QString modelPath;          ///< 关联模型路径（ini 模式记默认模型路径）
};

WeldMeasureService::WeldMeasureService()
    : m_impl(new Impl)
{
}

WeldMeasureService::~WeldMeasureService()
{
    // 先释放 DLL 上下文，再销毁 Impl，避免句柄泄漏。
    shutdown();
    delete m_impl;
    m_impl = nullptr;
}

QString WeldMeasureService::defaultConfigPath()
{
    // 与安装布局约定一致：可执行目录下的 config/weld_measure。
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/weld_measure/weld_measurement.ini"));
}

QString WeldMeasureService::defaultArmConfigPath()
{
    // 优先英文文件名（便于跨环境脚本），不存在时回退历史中文命名。
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

QString WeldMeasureService::defaultRingArmConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/weld_measure/weld_measurement-ring-arm.ini"));
}

QString WeldMeasureService::defaultRingTelescopicConfigPath()
{
    return QDir(QCoreApplication::applicationDirPath())
        .filePath(QStringLiteral("config/weld_measure/weld_measurement-ring-telescopic.ini"));
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
    // 读路径未加锁：与写入同属单线程初始化路径时通常足够；正式测量前应先看 isReady。
    return m_impl != nullptr ? m_impl->configPath : QString();
}

QString WeldMeasureService::modelPath() const
{
    return m_impl != nullptr ? m_impl->modelPath : QString();
}

bool WeldMeasureService::initializeFromIni(const QString& configPath, WeldMeasureError* error)
{
    std::lock_guard<std::mutex> lock(m_impl->mutex);
    // 重复初始化：先销毁旧上下文，避免句柄泄漏或混用配置。
    DestroyContext(m_impl->ctx);
    m_impl->configPath.clear();
    m_impl->modelPath.clear();

    const QString resolved = configPath.isEmpty() ? defaultConfigPath() : configPath;
    if (!QFileInfo::exists(resolved)) {
        FillError(error, WM_ERR_MODEL, QStringLiteral("Config not found: %1").arg(resolved));
        qWarning(LOG_WELD_MEASURE) << "Config missing:" << resolved;
        return false;
    }

    // DLL 需要原生路径字符串；转为本地分隔符后以 UTF-8 传入。
    const QByteArray pathUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    wm_context* ctx = nullptr;
    wm_status status = WM_ERR_INTERNAL;
    qInfo(LOG_WELD_MEASURE).noquote()
        << QStringLiteral("WeldMeasure initialize begin ini=") << resolved;
    // SEH 包装：捕获 access violation 等；返回非 0 表示原生异常码。
    const unsigned nativeFault =
        sdk_seh::createFromIni(pathUtf8.constData(), &ctx, &status);
    if (nativeFault != 0) {
        const QString message = NativeFaultMessage("create_from_ini", nativeFault);
        FillError(error, WM_ERR_INTERNAL, message);
        qCritical(LOG_WELD_MEASURE).noquote() << message;
        return false;
    }
    if (status != WM_OK || ctx == nullptr) {
        FillError(error, status, QString::fromUtf8(wm_status_string(status)));
        qWarning(LOG_WELD_MEASURE) << "wm_create_from_ini failed:" << wm_status_string(status)
                                   << "ini=" << resolved;
        return false;
    }

    m_impl->ctx = ctx;
    m_impl->configPath = resolved;
    // ini 内已含模型路径；此处缓存默认模型路径便于日志/排查。
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

    // 仅填 ONNX 配置；帧级 ICP/咬边等仍依赖 DLL 内默认值，正式产线应走 initializeFromIni。
    const QByteArray modelUtf8 = QDir::toNativeSeparators(resolved).toUtf8();
    wm_onnx_config onnx;
    wm_onnx_config_default(&onnx);
    onnx.model_path = modelUtf8.constData();

    wm_context* ctx = nullptr;
    wm_status status = WM_ERR_INTERNAL;
    qInfo(LOG_WELD_MEASURE).noquote()
        << QStringLiteral("WeldMeasure initialize begin model=") << resolved;
    const unsigned nativeFault = sdk_seh::create(&onnx, &ctx, &status);
    if (nativeFault != 0) {
        const QString message = NativeFaultMessage("create", nativeFault);
        FillError(error, WM_ERR_INTERNAL, message);
        qCritical(LOG_WELD_MEASURE).noquote() << message;
        return false;
    }
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
    wm_status status = WM_ERR_INTERNAL;
    // options 传 nullptr：使用 create_from_ini 时写入上下文的 Frame/ICP/Undercut 配置。
    const unsigned nativeFault = sdk_seh::measureFrame(
        m_impl->ctx,
        frameIndex1Based,
        xyz,
        pointCount,
        nullptr,
        &result,
        message,
        sizeof(message),
        &status);

    if (nativeFault != 0) {
        // 原生异常后上下文状态未知，禁止 destroy/复用该句柄。
        m_impl->ctx = nullptr;
        const QString faultMessage = NativeFaultMessage("measure_frame", nativeFault);
        FillError(error, WM_ERR_INTERNAL, faultMessage);
        qCritical(LOG_WELD_MEASURE).noquote()
            << faultMessage << QStringLiteral(" frame=") << frameIndex1Based;
        return false;
    }

    if (status != WM_OK) {
        // 算法软失败（模型/配准/截面等）：保留上下文，允许后续帧重试。
        FillError(
            error,
            status,
            message[0] != '\0' ? QString::fromUtf8(message) : QString::fromUtf8(wm_status_string(status)));
        qWarning(LOG_WELD_MEASURE) << "wm_measure_frame failed:" << message
                                   << "frame=" << frameIndex1Based;
        return false;
    }

    // 将 C 结构体结果映射到 Qt/业务结构体。
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

    // 截面模式使用 DLL 默认 options（与正式帧流水线的 ini 参数路径不同）。
    wm_options options;
    wm_options_default(&options);

    wm_section_result result{};
    char message[512] = {0};
    wm_status status = WM_ERR_INTERNAL;
    const unsigned nativeFault = sdk_seh::measureSection(
        m_impl->ctx,
        xyz,
        pointCount,
        &options,
        &result,
        message,
        sizeof(message),
        &status);

    if (nativeFault != 0) {
        // 与 measureFrame 相同：异常后丢弃句柄，不再尝试 destroy。
        m_impl->ctx = nullptr;
        const QString faultMessage = NativeFaultMessage("measure_section", nativeFault);
        FillError(error, WM_ERR_INTERNAL, faultMessage);
        qCritical(LOG_WELD_MEASURE).noquote() << faultMessage;
        return false;
    }

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
