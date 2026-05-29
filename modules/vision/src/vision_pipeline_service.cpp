#include "scan_tracking/vision/vision_pipeline_service.h"

#include <QtCore/QMetaType>
#include <QtCore/QPointer>
#include <QtCore/QTimer>

#include <thread>
#include <qdebug.h>
#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/vision/hik_cxp_camera_service.h"
#include "scan_tracking/vision/lb_pose_detection_adapter.h"
#include "scan_tracking/vision/lbn_pose_detection_adapter.h"

namespace scan_tracking {
namespace vision {

namespace {

LbnPoseResult makeIdentityLbnBypassResult()
{
    LbnPoseResult result;
    result.invoked = true;
    result.success = true;
    result.poseMatrix.valid = true;
    result.matchedPointCount = 0;
    result.message = QStringLiteral(
        "TODO(marker): 未安装标记点，useIdentityRtWithoutMarkers=true，Rt 使用 4×4 单位阵。");
    return result;
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
    m_processing = false;
    if (const auto* configManager = scan_tracking::common::ConfigManager::instance()) {
        m_lbPoseConfig = configManager->lbPoseConfig();
        m_lbnPoseConfig = configManager->lbnPoseConfig();
    } else {
        m_lbPoseConfig = {};
        m_lbnPoseConfig = {};
    }
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
    m_processing = false;
    m_started = false;
    setState(VisionPipelineState::Stopped, QStringLiteral("视觉流水线已停止。"));
}

namespace {

constexpr int kMechToHikCaptureDelayMs = 2000;

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

quint64 VisionPipelineService::requestCaptureBundle(
    int segmentIndex,
    quint32 taskId,
    scan_tracking::mech_eye::CaptureMode mechCaptureMode)
{
    if (!m_started) {
        emit fatalError(VisionErrorCode::NotStarted, QStringLiteral("视觉流水线未启动。"));
        return 0;
    }
    if (m_pending.active || m_processing) {
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

    qInfo() << QStringLiteral("[VisionPipeline] 段号=") << segmentIndex
            << QStringLiteral(" 需梅卡2D=") << request.needMechEye2D
            << QStringLiteral(" 梅卡模式=") << static_cast<int>(mechCaptureMode)
            << QStringLiteral(" CXP延迟ms=") << kMechToHikCaptureDelayMs;
    pending.mechRequestId = m_mechEyeService->requestCapture(
        request.mechEyeCameraKey,
        mechCaptureMode,
        request.mechEyeTimeoutMs);
    if (pending.mechRequestId == 0) {
        emit fatalError(VisionErrorCode::CaptureRejected, QStringLiteral("启动 Mech-Eye 采集失败。"));
        return 0;
    }
    pending.mechDone = false;
    pending.hikADone = false;
    pending.hikBDone = false;

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
    qInfo() << QStringLiteral("[VisionPipeline] 梅卡已完成，启动 CXP 双目采集 段号=")
            << request.segmentIndex;

    m_pending.hikARequestId = m_hikCameraAService->requestPoseCapture(
        request.hikCameraAKey, request.hikTimeoutMs);
    m_pending.hikBRequestId = m_hikCameraBService->requestPoseCapture(
        request.hikCameraBKey, request.hikTimeoutMs);

    if (m_pending.hikARequestId == 0 || m_pending.hikBRequestId == 0) {
        m_pending.active = false;
        m_processing = false;
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

    qInfo() << QStringLiteral("[VisionPipeline] 梅卡采集结束，%1ms 后启动 CXP 双目")
                   .arg(kMechToHikCaptureDelayMs);
    QTimer::singleShot(kMechToHikCaptureDelayMs, this, [this]() {
        startPendingHikCapture();
    });
}

void VisionPipelineService::onHikPoseCaptureFinished(scan_tracking::vision::HikPoseCaptureResult result)
{
    if (!m_pending.active) {
        return;
    }

    // 用 logicalName 区分来源（两个相机服务的 requestId 可能重复）
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

    const auto bundle = m_pending.bundle;
    m_pending = PendingCaptureContext{};

    if (!mechCapturePayloadReady(bundle.mechEyeResult)) {
        m_processing = false;
        setState(
            VisionPipelineState::Error,
            QStringLiteral("Mech-Eye 采集失败：%1").arg(bundle.mechEyeResult.errorMessage));
        emit bundleCaptureFinished(bundle);
        return;
    }

    const bool hikReady =
        bundle.hikCameraAResult.success() && bundle.hikCameraBResult.success();
    const bool runLbn = bundle.request.needMechEye2D && m_lbnPoseConfig.enabled;

    m_processing = true;
    setState(
        VisionPipelineState::Capturing,
        QStringLiteral("Mech-Eye 采集完成，正在处理视觉结果（海康=%1, LBN=%2）")
            .arg(hikReady ? QStringLiteral("就绪") : QStringLiteral("跳过"))
            .arg(runLbn ? QStringLiteral("开") : QStringLiteral("关")));

    const auto lbConfig = m_lbPoseConfig;
    const auto lbnConfig = m_lbnPoseConfig;
    QPointer<VisionPipelineService> self(this);
    std::thread([self, bundle, lbConfig, lbnConfig, hikReady, runLbn]() mutable {
        auto completedBundle = bundle;

        if (runLbn) {
            if (lbnConfig.useIdentityRtWithoutMarkers) {
                completedBundle.lbnPoseResult = makeIdentityLbnBypassResult();
                qWarning() << "[LBN位姿]" << completedBundle.lbnPoseResult.message;
            } else {
                qInfo() << QStringLiteral("[LBN位姿] 开始检测")
                        << QStringLiteral(" 纹理=") << bundle.mechEyeResult.texture2D.width << QStringLiteral("x")
                        << bundle.mechEyeResult.texture2D.height
                        << QStringLiteral(" 点云网格=") << bundle.mechEyeResult.pointCloud.width << QStringLiteral("x")
                        << bundle.mechEyeResult.pointCloud.height;
                completedBundle.lbnPoseResult = runLbnPoseDetection(bundle.mechEyeResult, lbnConfig);
            }
            const auto& lbn = completedBundle.lbnPoseResult;
            qInfo() << QStringLiteral("[LBN位姿] 完成：已调用=") << lbn.invoked << QStringLiteral(" 成功=") << lbn.success
                    << QStringLiteral(" 说明=") << lbn.message << QStringLiteral(" 匹配点数=") << lbn.matchedPointCount;
        } else {
            completedBundle.lbnPoseResult.invoked = false;
            completedBundle.lbnPoseResult.message =
                bundle.request.needMechEye2D
                    ? QStringLiteral("LBN 位姿检测已在配置中禁用。")
                    : QStringLiteral("非转动点位，跳过 LBN 位姿检测。");
        }

        if (hikReady) {
            qInfo() << QStringLiteral("[LB位姿] 开始检测，左目=") << bundle.hikCameraAResult.frame.width << QStringLiteral("x")
                    << bundle.hikCameraAResult.frame.height
                    << QStringLiteral(" 右目=") << bundle.hikCameraBResult.frame.width << QStringLiteral("x")
                    << bundle.hikCameraBResult.frame.height;

            completedBundle.lbPoseResult = runLbPoseDetection(
                completedBundle.hikCameraAResult.frame,
                completedBundle.hikCameraBResult.frame,
                lbConfig);

            const auto& lr = completedBundle.lbPoseResult;
            qInfo() << QStringLiteral("[LB位姿] 完成：成功=") << lr.success << QStringLiteral(" 说明=") << lr.message
                    << QStringLiteral(" 帧点数=") << lr.framePointCount;
            if (lr.poseMatrix.valid) {
                qInfo() << QStringLiteral("[LB位姿] Rt 矩阵：");
                for (int row = 0; row < 4; ++row) {
                    qInfo().noquote() << QString("  [%1, %2, %3, %4]")
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 0]), 12, 'f', 6)
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 1]), 12, 'f', 6)
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 2]), 12, 'f', 6)
                        .arg(static_cast<double>(lr.poseMatrix.values[row * 4 + 3]), 12, 'f', 6);
                }
            }
        } else {
            completedBundle.lbPoseResult.invoked = false;
            completedBundle.lbPoseResult.success = false;
            completedBundle.lbPoseResult.message = QStringLiteral("CXP 双目未就绪，跳过 LB 位姿检测");
        }

        if (!self) {
            return;
        }

        QMetaObject::invokeMethod(
            self.data(),
            [self, completedBundle, hikReady]() mutable {
                if (!self || !self->m_started) {
                    return;
                }

                self->m_processing = false;
                const bool ok = mechCapturePayloadReady(completedBundle.mechEyeResult) &&
                                completedBundle.hikCameraAResult.success() &&
                                completedBundle.hikCameraBResult.success();
                self->setState(
                    ok ? VisionPipelineState::Ready : VisionPipelineState::Error,
                    ok ? QStringLiteral("视觉组合采集成功完成。")
                       : QStringLiteral("视觉组合采集完成但有错误。"));
                emit self->bundleCaptureFinished(completedBundle);
            },
            Qt::QueuedConnection);
    }).detach();
}

}  // namespace vision
}  // namespace scan_tracking
