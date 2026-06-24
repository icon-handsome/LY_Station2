#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QMetaType>
#include <QtCore/QPointer>
#include <QtCore/QTimer>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"

namespace scan_tracking {
namespace vision {

namespace {

bool mechCapturePayloadReady(const scan_tracking::mech_eye::CaptureResult& result)
{
    if (!result.success()) {
        return false;
    }
    if (result.mode == scan_tracking::mech_eye::CaptureMode::Capture2DOnly) {
        return result.texture2D.isValid();
    }
    return result.pointCloud.isValid();
}

}  // namespace

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
    scan_tracking::mech_eye::MechEyeService* mechEyeService,
    HikCxpCameraService* hikCameraAService,
    HikCxpCameraService* hikCameraBService,
    QObject* parent)
    : QObject(parent)
    , m_mechEyeService(mechEyeService)
    , m_hikCameraAService(hikCameraAService)
    , m_hikCameraBService(hikCameraBService)
{
    registerMetaTypes();

    if (m_mechEyeService != nullptr) {
        connect(
            m_mechEyeService,
            &scan_tracking::mech_eye::MechEyeService::captureFinished,
            this,
            &VisionPipelineService::onMechEyeCaptureFinished,
            Qt::QueuedConnection);
    }
    if (m_hikCameraAService != nullptr) {
        connect(
            m_hikCameraAService,
            &HikCxpCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
    if (m_hikCameraBService != nullptr) {
        connect(
            m_hikCameraBService,
            &HikCxpCameraService::poseCaptureFinished,
            this,
            &VisionPipelineService::onHikPoseCaptureFinished,
            Qt::QueuedConnection);
    }
}

void VisionPipelineService::start(const scan_tracking::common::VisionConfig& config)
{
    m_config = config;
    m_pending = PendingCaptureContext{};
    m_started = true;
    setState(
        VisionPipelineState::Ready,
        QStringLiteral("视觉流水线已启动，等待采集请求。"));
}

void VisionPipelineService::stop()
{
    if (!m_started) {
        return;
    }

    m_pending = PendingCaptureContext{};
    m_started = false;
    setState(VisionPipelineState::Stopped, QStringLiteral("视觉流水线已停止。"));
}

namespace {

constexpr int kMechToHikCaptureDelayMs = 2000;

}  // namespace

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("视觉流水线未启动。"));
        return 0;
    }
    if (m_pending.active) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("视觉采集请求正在进行中。"));
        return 0;
    }
    if (m_mechEyeService == nullptr || m_hikCameraAService == nullptr || m_hikCameraBService == nullptr) {
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("视觉服务不完整。"));
        return 0;
    }

    MultiCameraCaptureRequest request;
    request.requestId = m_nextRequestId++;
    request.taskId = taskId;
    request.segmentIndex = segmentIndex;
    request.mechCaptureMode = mechCaptureMode;
    request.needMechEye2D =
        mechCaptureMode == scan_tracking::mech_eye::CaptureMode::Capture2DAnd3D;
    request.mechEyeCameraKey = m_config.mechEyeCameraKey;
    request.mechEyeTimeoutMs = m_config.mechCaptureTimeoutMs > 0 ? m_config.mechCaptureTimeoutMs : 5000;
    request.hikCameraAKey = m_config.hikCxpCameraA.cameraKey;
    request.hikCameraBKey = m_config.hikCxpCameraB.cameraKey;
    request.hikTimeoutMs =
        m_config.hikCxpCaptureTimeoutMs > 0 ? m_config.hikCxpCaptureTimeoutMs : 5000;

    PendingCaptureContext pending;
    pending.active = true;
    pending.bundle.request = request;

    pending.mechRequestId = m_mechEyeService->requestCapture(
        request.mechEyeCameraKey,
        mechCaptureMode,
        request.mechEyeTimeoutMs);
    if (pending.mechRequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("启动 Mech-Eye 采集失败。"));
        return 0;
    }

    m_pending = pending;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("梅卡采集已启动（CXP 将在梅卡完成后延迟 %1ms）").arg(kMechToHikCaptureDelayMs));
    return request.requestId;
}

void VisionPipelineService::startPendingHikCapture()
{
    if (!m_pending.active || m_pending.hikARequestId != 0) {
        return;
    }

    const auto& request = m_pending.bundle.request;
    m_pending.hikARequestId = m_hikCameraAService->requestPoseCapture(
        request.hikCameraAKey, request.hikTimeoutMs);
    m_pending.hikBRequestId = m_hikCameraBService->requestPoseCapture(
        request.hikCameraBKey, request.hikTimeoutMs);

    if (m_pending.hikARequestId == 0 || m_pending.hikBRequestId == 0) {
        m_pending.active = false;
        emit fatalError(
            VisionErrorCode::CaptureRejected,
            QStringLiteral("梅卡完成后启动 CXP 双目采集失败。"));
        setState(VisionPipelineState::Error, QStringLiteral("CXP 双目采集启动失败。"));
        return;
    }

    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("CXP 双目采集已启动：requestId=%1").arg(request.requestId));
}

void VisionPipelineService::onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    if (!m_pending.active || result.requestId != m_pending.mechRequestId) {
        return;
    }

    m_pending.bundle.mechEyeResult = result;
    m_pending.mechDone = true;

    QTimer::singleShot(kMechToHikCaptureDelayMs, this, [this]() {
        startPendingHikCapture();
    });
}

void VisionPipelineService::onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result)
{
    if (!m_pending.active) {
        return;
    }

    if (result.logicalName == m_config.hikCxpCameraA.logicalName) {
        m_pending.bundle.hikCameraAResult = result;
        m_pending.hikADone = true;
    } else if (result.logicalName == m_config.hikCxpCameraB.logicalName) {
        m_pending.bundle.hikCameraBResult = result;
        m_pending.hikBDone = true;
    } else {
        return;
    }

    finishBundleIfReady();
}

void VisionPipelineService::setState(VisionPipelineState state, const QString& description)
{
    m_state = state;
    emit stateChanged(state, description);
}

void VisionPipelineService::finishBundleIfReady()
{
    if (!m_pending.active || !m_pending.mechDone || !m_pending.hikADone || !m_pending.hikBDone) {
        return;
    }

    auto completedBundle = m_pending.bundle;
    m_pending = PendingCaptureContext{};

    const bool ok = mechCapturePayloadReady(completedBundle.mechEyeResult) &&
                    completedBundle.hikCameraAResult.success() &&
                    completedBundle.hikCameraBResult.success();
    setState(
        ok ? VisionPipelineState::Ready : VisionPipelineState::Error,
        ok ? QStringLiteral("视觉组合采集成功完成。")
           : QStringLiteral("视觉组合采集完成但有错误。"));
    emit bundleCaptureFinished(completedBundle);
}

}  // namespace vision
}  // namespace scan_tracking
