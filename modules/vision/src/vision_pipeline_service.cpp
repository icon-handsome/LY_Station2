#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QLoggingCategory>
#include <QtCore/QDateTime>
#include <QtCore/QFile>
#include <QtCore/QMetaType>
#include <QtCore/QPointer>
#include <QtCore/QTimer>

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"

namespace scan_tracking {
namespace vision {

namespace {

Q_LOGGING_CATEGORY(LOG_VISION_PIPELINE, "vision.pipeline")

constexpr int kMechToHikCaptureDelayMs = 1000;

QString captureTypeLabel(CaptureType type)
{
    switch (type) {
    case CaptureType::SurfaceDefect:
        return QStringLiteral("SurfaceDefect");
    case CaptureType::WeldDefect:
        return QStringLiteral("WeldDefect");
    case CaptureType::NumberRecognition:
        return QStringLiteral("NumberRecognition");
    default:
        return QStringLiteral("Unknown");
    }
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
    qRegisterMetaType<scan_tracking::vision::CaptureType>("scan_tracking::vision::CaptureType");
    registered = true;
}

VisionPipelineService::VisionPipelineService(
    scan_tracking::mech_eye::MechEyeService* mechEyeService,
    HikCxpCameraService* hikCameraAService,
    HikCxpCameraService* hikCameraBService,
    HikCameraCController* hikCameraCController,
    QObject* parent)
    : QObject(parent)
    , m_mechEyeService(mechEyeService)
    , m_hikCameraAService(hikCameraAService)
    , m_hikCameraBService(hikCameraBService)
    , m_hikCameraCController(hikCameraCController)
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
    if (m_hikCameraCController != nullptr) {
        connect(
            m_hikCameraCController,
            &HikCameraCController::imageReceived,
            this,
            &VisionPipelineService::onHikCameraCImageReceived,
            Qt::QueuedConnection);
        connect(
            m_hikCameraCController,
            &HikCameraCController::captureCompleted,
            this,
            &VisionPipelineService::onHikCameraCCaptureCompleted,
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

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode)
{
    return requestCaptureBundle(segmentIndex, taskId, mechCaptureMode, false);
}

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode,
    bool telescopicConcurrentHikC)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("视觉流水线未启动。"));
        return 0;
    }
    if (m_pending.active) {
        emit fatalError(VisionErrorCode::Busy, QStringLiteral("视觉采集请求正在进行中。"));
        return 0;
    }
    if (m_mechEyeService == nullptr) {
        emit fatalError(VisionErrorCode::InvalidConfig, QStringLiteral("梅卡相机服务不可用。"));
        return 0;
    }

    const bool useCxp =
        !telescopicConcurrentHikC && m_config.hikCxpEnabled &&
        m_hikCameraAService != nullptr && m_hikCameraBService != nullptr;
    const bool useHikCameraC =
        m_hikCameraCController != nullptr && (telescopicConcurrentHikC || !useCxp);

    if (!telescopicConcurrentHikC && !useHikCameraC && !useCxp) {
        emit fatalError(
            VisionErrorCode::InvalidConfig,
            QStringLiteral("视觉服务不完整：需要 CXP 双目或海康智能 C。"));
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
    request.mechEyeTimeoutMs =
        m_config.mechCaptureTimeoutMs > 0 ? m_config.mechCaptureTimeoutMs : 5000;
    if (useCxp) {
        request.hikCameraAKey = m_config.hikCxpCameraA.cameraKey;
        request.hikCameraBKey = m_config.hikCxpCameraB.cameraKey;
        request.hikTimeoutMs =
            m_config.hikCxpCaptureTimeoutMs > 0 ? m_config.hikCxpCaptureTimeoutMs : 5000;
    }

    PendingCaptureContext pending;
    pending.active = true;
    pending.useHikCameraC = useHikCameraC;
    pending.hikCTriggerOnly = telescopicConcurrentHikC && useHikCameraC;
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

    if (telescopicConcurrentHikC && useHikCameraC) {
        triggerHikCameraCConcurrent(true);
        setState(
            VisionPipelineState::Capturing,
            QStringLiteral("伸缩杆联动：梅卡与海康 C 拍照指令已同时发出。"));
    } else if (useHikCameraC) {
        setState(
            VisionPipelineState::Capturing,
            QStringLiteral("梅卡采集已启动（海康 C 将在梅卡完成后延迟 %1ms）")
                .arg(kMechToHikCaptureDelayMs));
    } else {
        setState(
            VisionPipelineState::Capturing,
            QStringLiteral("梅卡采集已启动（CXP 将在梅卡完成后延迟 %1ms）")
                .arg(kMechToHikCaptureDelayMs));
    }
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

    if (m_pending.hikARequestId == 0) {
        m_pending.bundle.hikCameraAResult.logicalName = m_config.hikCxpCameraA.logicalName;
        m_pending.bundle.hikCameraAResult.errorCode = VisionErrorCode::CaptureRejected;
        m_pending.bundle.hikCameraAResult.errorMessage =
            QStringLiteral("CXP 左目采集启动失败。");
        m_pending.hikADone = true;
    }
    if (m_pending.hikBRequestId == 0) {
        m_pending.bundle.hikCameraBResult.logicalName = m_config.hikCxpCameraB.logicalName;
        m_pending.bundle.hikCameraBResult.errorCode = VisionErrorCode::CaptureRejected;
        m_pending.bundle.hikCameraBResult.errorMessage =
            QStringLiteral("CXP 右目采集启动失败。");
        m_pending.hikBDone = true;
    }
    if (m_pending.hikADone && m_pending.hikBDone) {
        finishBundleIfReady();
        return;
    }

    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("CXP 双目采集已启动：requestId=%1").arg(request.requestId));
}

void VisionPipelineService::triggerHikCameraCConcurrent(bool triggerOnly)
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone) {
        return;
    }

    if (m_hikCameraCController == nullptr ||
        !m_hikCameraCController->requestCapture(CaptureType::SurfaceDefect)) {
        qWarning(LOG_VISION_PIPELINE).noquote()
            << QStringLiteral("[VisionPipeline] 海康 C 拍照指令发送失败（TCP 未连接或未就绪）");
        m_pending.hikCDone = true;
        m_pending.bundle.hikCameraCImagePath.clear();
        return;
    }

    qInfo(LOG_VISION_PIPELINE).noquote()
        << QStringLiteral("[VisionPipeline] 海康 C 拍照指令已发送 requestId=")
        << m_pending.bundle.request.requestId;

    if (triggerOnly) {
        m_pending.hikCDone = true;
        return;
    }

    const int timeoutMs =
        m_config.hikCaptureTimeoutMs > 0 ? m_config.hikCaptureTimeoutMs : 5000;
    QPointer<VisionPipelineService> self(this);
    QTimer::singleShot(timeoutMs, this, [self]() {
        if (self != nullptr) {
            self->onHikCameraCCaptureTimeout();
        }
    });
}

void VisionPipelineService::startPendingHikCameraCCapture()
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone) {
        return;
    }

    triggerHikCameraCConcurrent(false);

    if (m_pending.hikCDone) {
        finishBundleIfReady();
        return;
    }

    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("海康 C 采集已触发：requestId=%1").arg(m_pending.bundle.request.requestId));
}

void VisionPipelineService::completeHikCameraCCapture(const QString& imagePath)
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone ||
        m_pending.hikCTriggerOnly) {
        return;
    }

    if (imagePath.trimmed().isEmpty() || !QFile::exists(imagePath)) {
        return;
    }

    m_pending.bundle.hikCameraCImagePath = imagePath;
    m_pending.hikCDone = true;
    finishBundleIfReady();
}

void VisionPipelineService::onHikCameraCCaptureTimeout()
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone) {
        return;
    }

    m_pending.hikCDone = true;
    m_pending.bundle.hikCameraCImagePath.clear();
    finishBundleIfReady();
}

void VisionPipelineService::onMechEyeCaptureFinished(scan_tracking::mech_eye::CaptureResult result)
{
    if (!m_pending.active || result.requestId != m_pending.mechRequestId) {
        return;
    }

    m_pending.bundle.mechEyeResult = result;
    m_pending.mechDone = true;

    if (m_pending.hikCTriggerOnly) {
        finishBundleIfReady();
        return;
    }

    QPointer<VisionPipelineService> self(this);
    QTimer::singleShot(kMechToHikCaptureDelayMs, this, [self]() {
        if (self == nullptr || !self->m_pending.active) {
            return;
        }
        if (self->m_pending.useHikCameraC) {
            self->startPendingHikCameraCCapture();
        } else {
            self->startPendingHikCapture();
        }
    });
}

void VisionPipelineService::onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result)
{
    if (!m_pending.active || m_pending.useHikCameraC) {
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

void VisionPipelineService::onHikCameraCImageReceived(
    scan_tracking::vision::CaptureType type,
    QString filePath,
    qint64 fileSize)
{
    Q_UNUSED(type);
    Q_UNUSED(fileSize);
    if (!m_pending.active || m_pending.hikCTriggerOnly) {
        return;
    }
    completeHikCameraCCapture(filePath);
}

void VisionPipelineService::onHikCameraCCaptureCompleted(
    scan_tracking::vision::CaptureType type,
    QByteArray imageData)
{
    if (!m_pending.active || !m_pending.useHikCameraC || m_pending.hikCDone ||
        m_pending.hikCTriggerOnly) {
        return;
    }

    if (imageData.isEmpty()) {
        return;
    }

    const QString saveDir = QStringLiteral("./smart_camera_images");
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString filePath =
        QStringLiteral("%1/pipeline_%2_%3.jpg").arg(saveDir, captureTypeLabel(type), timestamp);

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly)) {
        return;
    }
    if (file.write(imageData) != imageData.size()) {
        return;
    }
    file.close();

    completeHikCameraCCapture(filePath);
}

void VisionPipelineService::setState(VisionPipelineState state, const QString& description)
{
    m_state = state;
    emit stateChanged(state, description);
}

void VisionPipelineService::finishBundleIfReady()
{
    if (!m_pending.active || !m_pending.mechDone) {
        return;
    }

    if (m_pending.useHikCameraC) {
        if (!m_pending.hikCDone) {
            return;
        }
    } else if (!m_pending.hikADone || !m_pending.hikBDone) {
        return;
    }

    auto completedBundle = m_pending.bundle;
    m_pending = PendingCaptureContext{};

    const bool ok = completedBundle.success();
    const bool partialOk = ok && !completedBundle.allCamerasOk();
    setState(
        ok ? VisionPipelineState::Ready : VisionPipelineState::Error,
        ok ? (partialOk ? QStringLiteral("视觉组合采集完成（部分相机成功）。")
                        : QStringLiteral("视觉组合采集成功完成。"))
           : QStringLiteral("视觉组合采集完成但有错误。"));
    emit bundleCaptureFinished(completedBundle);
}

}  // namespace vision
}  // namespace scan_tracking
