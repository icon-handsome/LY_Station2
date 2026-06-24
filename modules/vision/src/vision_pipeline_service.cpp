#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QMetaType>

namespace scan_tracking {
namespace vision {

void VisionPipelineService::registerMetaTypes()
{
    static bool registered = false;
    if (registered) {
        return;
    }

    qRegisterMetaType<scan_tracking::vision::VisionPipelineState>(
        "scan_tracking::vision::VisionPipelineState");
    qRegisterMetaType<scan_tracking::vision::MultiCameraCaptureBundle>(
        "scan_tracking::vision::MultiCameraCaptureBundle");
    registered = true;
}

VisionPipelineService::VisionPipelineService(
    orbbec_gemini::OrbbecGeminiService* orbbecService,
    QObject* parent)
    : QObject(parent)
    , m_orbbecService(orbbecService)
{
    registerMetaTypes();

    if (m_orbbecService != nullptr) {
        connect(
            m_orbbecService,
            &orbbec_gemini::OrbbecGeminiService::captureFinished,
            this,
            &VisionPipelineService::onOrbbecCaptureFinished,
            Qt::QueuedConnection);
    }
}

void VisionPipelineService::start(const scan_tracking::common::OrbbecGeminiConfig& config)
{
    m_config = config;
    m_pending = PendingCaptureContext{};
    m_started = true;
    setState(
        VisionPipelineState::Ready,
        QStringLiteral("Orbbec 视觉流水线已启动，等待采集请求。"));
}

void VisionPipelineService::stop()
{
    if (!m_started) {
        return;
    }

    m_pending = PendingCaptureContext{};
    m_started = false;
    setState(VisionPipelineState::Stopped, QStringLiteral("Orbbec 视觉流水线已停止。"));
}

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    bool needColorCapture)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("视觉流水线未启动。"));
        return 0;
    }
    if (m_pending.active) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("视觉采集请求正在进行中。"));
        return 0;
    }
    if (m_orbbecService == nullptr) {
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("Orbbec 服务未注入。"));
        return 0;
    }
    if (m_orbbecService->state() != orbbec_gemini::OrbbecGeminiRuntimeState::Ready) {
        emit fatalError(
            VisionErrorCode::DeviceNotFound,
            QStringLiteral("Orbbec 设备未就绪，当前状态=%1")
                .arg(static_cast<int>(m_orbbecService->state())));
        return 0;
    }
    if (m_orbbecService->isBusy()) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("Orbbec 正在采集中。"));
        return 0;
    }

    MultiCameraCaptureRequest request;
    request.requestId = m_nextRequestId++;
    request.taskId = taskId;
    request.segmentIndex = segmentIndex;
    request.needColorCapture = needColorCapture;
    request.orbbecTimeoutMs =
        m_config.captureTimeoutMs > 0 ? m_config.captureTimeoutMs : 5000;
    request.saveToDisk = m_config.saveCaptureToDisk;

    PendingCaptureContext pending;
    pending.active = true;
    pending.bundle.request = request;

    const quint64 orbbecRequestId = m_orbbecService->requestCapture(
        request.orbbecTimeoutMs,
        request.saveToDisk);
    if (orbbecRequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("启动 Orbbec 采集失败。"));
        return 0;
    }

    pending.orbbecRequestId = orbbecRequestId;
    m_pending = pending;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("Orbbec 采集已启动：requestId=%1 段号=%2")
            .arg(request.requestId)
            .arg(segmentIndex));
    return request.requestId;
}

void VisionPipelineService::onOrbbecCaptureFinished(orbbec_gemini::OrbbecCaptureResult result)
{
    if (!m_pending.active || result.requestId != m_pending.orbbecRequestId) {
        return;
    }

    finishCapture(result);
}

void VisionPipelineService::setState(VisionPipelineState state, const QString& description)
{
    m_state = state;
    emit stateChanged(state, description);
}

void VisionPipelineService::finishCapture(const orbbec_gemini::OrbbecCaptureResult& result)
{
    auto completedBundle = m_pending.bundle;
    completedBundle.orbbecResult = result;
    m_pending = PendingCaptureContext{};

    const bool ok = orbbecCapturePayloadReady(result);
    setState(
        ok ? VisionPipelineState::Ready : VisionPipelineState::Error,
        ok ? QStringLiteral("Orbbec 分段采集成功完成。")
           : QStringLiteral("Orbbec 分段采集失败：%1").arg(result.errorMessage));
    emit bundleCaptureFinished(completedBundle);
}

}  // namespace vision
}  // namespace scan_tracking
