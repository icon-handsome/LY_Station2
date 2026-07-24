#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/mech_eye/mech_eye_service.h"
#include "scan_tracking/mech_eye/point_cloud_processor.h"
#include "scan_tracking/vision/vision_pipeline_service.h"

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

namespace {

std::array<float, 16> identityMatrix4x4()
{
    return {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f,
    };
}

/// LB 成功时用 Rt_global 作为 T0，stereo=I（与第一工位 applySegmentPoseStitching 一致）。
/// 保留 pointCloudRaw=原始云，pointCloud=变换后云（供检测与 cloud_stitched 落盘）。
void applyLbPoseStitchingIfNeeded(vision::MultiCameraCaptureBundle* bundle)
{
    if (bundle == nullptr) {
        return;
    }

    const auto& lb = bundle->lbPoseResult;
    if (!lb.invoked) {
        return;
    }
    if (!lb.success || !lb.poseMatrix.valid) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("LB 位姿失败，跳过点云变换：") << lb.message;
        return;
    }
    if (!bundle->mechEyeResult.pointCloud.isValid()) {
        qWarning(LOG_FLOW).noquote() << QStringLiteral("LB 成功但 Mech 点云无效，跳过变换。");
        return;
    }

    // shared_ptr 别名保留原始缓冲，避免深拷贝；变换失败则清掉 raw 标记。
    bundle->mechEyeResult.pointCloudRaw = bundle->mechEyeResult.pointCloud;

    mech_eye::PointCloudFrame stitched;
    QString stitchMessage;
    if (!mech_eye::transformPointCloudFrame(
            bundle->mechEyeResult.pointCloudRaw,
            lb.poseMatrix.values,
            identityMatrix4x4(),
            &stitched,
            &stitchMessage)) {
        bundle->mechEyeResult.pointCloudRaw = mech_eye::PointCloudFrame{};
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("点云 LB 变换失败：") << stitchMessage;
        return;
    }

    if (!stitched.isValid() || stitched.pointCount <= 0 ||
        stitched.pointsXYZ == nullptr ||
        static_cast<int>(stitched.pointsXYZ->size()) < stitched.pointCount * 3) {
        bundle->mechEyeResult.pointCloudRaw = mech_eye::PointCloudFrame{};
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("点云 LB 变换结果无效，保留原始云，不替换 pointCloud。");
        return;
    }

    bundle->mechEyeResult.pointCloud = std::move(stitched);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("点云已按 LB Rt_global 变换：") << stitchMessage
        << QStringLiteral(" framePoints=") << lb.framePointCount
        << QStringLiteral(" rawKept=") << bundle->mechEyeResult.pointCloudRaw.isValid();
}

}  // namespace

void StateMachine::notifyScanStarted(int segmentIndex, quint32 taskId)
{
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        maybeEmitPathStarted(cfgMgr->activePathId());
    }
    emit scanStarted(segmentIndex, taskId);
}

void StateMachine::onBundleCaptureFinished(vision::MultiCameraCaptureBundle bundle)
{
    if (m_activeTask.definition == nullptr) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：无活动任务 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (m_activeTask.definition->stage == protocol::Stage::SelfCheck) {
        finishSelfCheckCapture(bundle);
        return;
    }
    if (!isScanCaptureStage(m_activeTask.definition->stage)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：当前阶段非扫描采集 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (m_activeTask.completionAnnounced) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：任务已收尾 requestId=")
            << bundle.request.requestId;
        return;
    }
    if (bundle.request.requestId != m_activeTask.captureRequestId) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("bundleCaptureFinished 忽略：requestId 不匹配 active=")
            << m_activeTask.captureRequestId
            << QStringLiteral(" bundle=") << bundle.request.requestId;
        return;
    }
    if (!acceptWorkpieceGeneration(
            m_activeTask.workpieceGeneration, QStringLiteral("bundleCaptureFinished"))) {
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("bundleCaptureFinished 处理 requestId=")
        << bundle.request.requestId
        << QStringLiteral(" segment=") << bundle.request.segmentIndex
        << QStringLiteral(" points=") << bundle.mechEyeResult.pointCloud.pointCount
        << QStringLiteral(" texture=") << bundle.mechEyeResult.texture2D.width
        << QLatin1Char('x') << bundle.mechEyeResult.texture2D.height
        << QStringLiteral(" textureValid=") << bundle.mechEyeResult.texture2D.isValid()
        << QStringLiteral(" lbInvoked=") << bundle.lbPoseResult.invoked;

    const QString triggerLabel = protocol::triggerName(*m_activeTask.definition);

    int imageCount = 0;
    int cloudFrameCount = 0;
    countBundleFrames(bundle, &imageCount, &cloudFrameCount);
    const QString bundleSummary = bundle.summary();
    const quint32 bundleTaskId = bundle.request.taskId;

    if (bundle.success()) {
        const auto device =
            m_activeTask.definition->stage == protocol::Stage::TelescopicScan
                ? common::ScanDeviceKind::Telescopic
                : common::ScanDeviceKind::Arm;

        // length_volume / 带 CXP 路径：用 LB Rt_global 把 Mech 点云变换到统一坐标系后再缓存/合并。
        applyLbPoseStitchingIfNeeded(&bundle);

        m_scanSegmentCache.storeSegment(
            device,
            bundle.request.segmentIndex,
            bundle.request.taskId,
            std::move(bundle));

        // 按运行实例（taskId）唯一目录落盘本段全部 3D+2D 数据（Mech PLY 为 binary）。
        QString persistError;
        if (!m_scanSegmentCache.persistSegment(
                device, bundle.request.segmentIndex, &persistError)) {
            qWarning(LOG_FLOW).noquote()
                << triggerLabel << QStringLiteral("：采集成功但落盘失败")
                << persistError
                << QStringLiteral(" taskId=") << bundleTaskId
                << QStringLiteral(" runRoot=")
                << m_scanSegmentCache.runCaptureRoot();
        } else {
            qInfo(LOG_FLOW).noquote()
                << triggerLabel << QStringLiteral("：已落盘至")
                << m_scanSegmentCache.runCaptureRoot()
                << QStringLiteral(" taskId=") << bundle.request.taskId;
        }

        const auto* configMgr = common::ConfigManager::instance();
        const int armExpected = configMgr != nullptr ? configMgr->enabledArmPointCount() : 0;
        const int telescopicExpected =
            configMgr != nullptr ? configMgr->enabledTelescopicPointCount() : 0;
        const int pathId = configMgr != nullptr ? configMgr->activePathId() : 0;
        qInfo(LOG_FLOW).noquote()
            << triggerLabel << QStringLiteral("：采集成功") << bundleSummary
            << QStringLiteral(" pathId=") << pathId
            << QStringLiteral(" imageCount=") << imageCount
            << QStringLiteral(" cloudFrameCount=") << cloudFrameCount
            << QStringLiteral(" cache arm=")
            << m_scanSegmentCache.cachedCountForDevice(common::ScanDeviceKind::Arm)
            << QStringLiteral("/") << armExpected
            << QStringLiteral(" telescopic=")
            << m_scanSegmentCache.cachedCountForDevice(common::ScanDeviceKind::Telescopic)
            << QStringLiteral("/") << telescopicExpected;

        if (m_scanSegmentCache.meetsDeviceQuotas(armExpected, telescopicExpected)) {
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("pathId=") << pathId
                << QStringLiteral(" 扫描齐套。可 Trig_Inspection；若 PLC 直接再发段号 1，"
                                  "IPC 将自动清缓存并切换到下一条启用路径。");
        }

        completeScanSegmentCapture(1, imageCount, cloudFrameCount, protocol::AckState::Completed, true);
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << triggerLabel << QStringLiteral("：采集失败") << bundle.summary();
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::onVisionPipelineFatalError(vision::VisionErrorCode code, QString message)
{
    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[VisionPipeline] 致命错误：")
        << static_cast<int>(code)
        << message;

    if (m_activeTask.definition == nullptr || m_activeTask.completionAnnounced) {
        return;
    }

    if (m_activeTask.definition->stage == protocol::Stage::SelfCheck) {
        setAlarm(3, 723, message);
        constexpr quint16 kFailCapture = 1u << 4;
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        return;
    }

    if (!isScanCaptureStage(m_activeTask.definition->stage)) {
        return;
    }

    setAlarm(3, 723, message);
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::completeScanSegmentCapture(
    quint16 resultCode,
    int imageCount,
    int cloudFrameCount,
    protocol::AckState finalAckState,
    bool dataValid)
{
    const int segmentIndex = m_activeTask.scanSegmentIndex;
    const auto device = activeScanDeviceKind();
    const int pathId =
        common::ConfigManager::instance() != nullptr
            ? common::ConfigManager::instance()->activePathId()
            : 0;

    // 对齐工位1：严重错误（Res>=5）按 scanFailurePolicy 清理，默认不整表抹掉已扫段。
    if (resultCode >= 5) {
        applyScanFailurePolicy(pathId, device, segmentIndex, resultCode);
    }

    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::TelescopicScan) {
        writeTelescopicScanResult(segmentIndex, imageCount, cloudFrameCount);
    } else {
        writeScanSegmentResult(segmentIndex, imageCount, cloudFrameCount);
    }
    completeActiveTask(resultCode, finalAckState, dataValid);
    emit scanFinished(segmentIndex, resultCode, imageCount, cloudFrameCount);
}

common::ScanDeviceKind StateMachine::activeScanDeviceKind() const
{
    if (m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::TelescopicScan) {
        return common::ScanDeviceKind::Telescopic;
    }
    return common::ScanDeviceKind::Arm;
}

QString StateMachine::currentScanFailurePolicy() const
{
    if (const auto* cfg = common::ConfigManager::instance()) {
        const QString policy = cfg->flowControlConfig().scanFailurePolicy.trimmed().toLower();
        if (policy == QLatin1String("path") || policy == QLatin1String("workpiece")) {
            return policy;
        }
    }
    return QStringLiteral("segment");
}

void StateMachine::applyScanFailurePolicy(
    int pathId,
    common::ScanDeviceKind device,
    int segmentIndex,
    quint16 resultCode)
{
    const QString policy = currentScanFailurePolicy();

    if (policy == QLatin1String("workpiece")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] 策略=workpiece，整表清缓存 Res=") << resultCode
            << QStringLiteral(" pathId=") << pathId;
        bumpWorkpieceGeneration(QStringLiteral("scan_fail_workpiece"));
        clearTransientWorkpieceRuntimeState();
        resetScanSegmentCache();
        resetActivePathToFirstEnabled();
        clearPathProgressTracking(QStringLiteral("scan_fail_workpiece"));
        return;
    }

    if (policy == QLatin1String("path")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] 策略=path，清当前路径段缓存 Res=") << resultCode
            << QStringLiteral(" pathId=") << pathId;
        clearScanSegmentCacheForPathSwitch();
        m_lastInspectedPathId = -1;
        m_lastInspectedRunKey.clear();
        if (pathId > 0) {
            m_emittedPathFinished.remove(pathId);
        }
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[ScanFail] 策略=segment，仅剔失败本段 Res=") << resultCode
        << QStringLiteral(" pathId=") << pathId
        << QStringLiteral(" device=")
        << common::ConfigManager::scanDeviceKindToString(device)
        << QStringLiteral(" 段号=") << segmentIndex;
    if (segmentIndex > 0 && m_scanSegmentCache.removeSegment(device, segmentIndex)) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] 已从段缓存移除失败段");
    }
    // 失败段可能曾被标为已检测：允许同路径重试检测。
    if (m_lastInspectedPathId == pathId) {
        m_lastInspectedPathId = -1;
        m_lastInspectedRunKey.clear();
    }
}

void StateMachine::applyInspectionTimeoutFailurePolicy()
{
    // 对齐工位1：检测超时先作废在途后台结果，再按策略清缓存。
    bumpWorkpieceGeneration(QStringLiteral("inspection_timeout"));

    const QString policy = currentScanFailurePolicy();
    const int pathId =
        common::ConfigManager::instance() != nullptr
            ? common::ConfigManager::instance()->activePathId()
            : 0;

    // 超时不得视为「已检测完成」，否则会挡住同路径重试。
    m_lastInspectedPathId = -1;
    m_lastInspectedRunKey.clear();

    if (policy == QLatin1String("workpiece")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] Inspection 超时策略=workpiece，整表清缓存");
        clearTransientWorkpieceRuntimeState();
        resetScanSegmentCache();
        resetActivePathToFirstEnabled();
        clearPathProgressTracking(QStringLiteral("inspection_timeout_workpiece"));
        return;
    }

    if (policy == QLatin1String("path")) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("[ScanFail] Inspection 超时策略=path，清当前路径段缓存 pathId=")
            << pathId;
        clearScanSegmentCacheForPathSwitch();
        if (pathId > 0) {
            m_emittedPathFinished.remove(pathId);
        }
        return;
    }

    qWarning(LOG_FLOW).noquote()
        << QStringLiteral("[ScanFail] Inspection 超时策略=segment，保留段缓存供重试 pathId=")
        << pathId;
}

void StateMachine::onMechEyeFatalError(mech_eye::CaptureErrorCode code, QString message)
{
    Q_UNUSED(code);
    qCritical(LOG_FLOW) << "[MechEye] 致命错误:" << message;
    emit protocolEvent(QStringLiteral("Mech-Eye: %1").arg(message));

    if (m_activeTask.definition == nullptr ||
        !isScanCaptureStage(m_activeTask.definition->stage) ||
        m_activeTask.completionAnnounced) {
        return;
    }

    setAlarm(3, 723, message);
    completeScanSegmentCapture(7, 0, 0, protocol::AckState::Failed, false);
}

void StateMachine::resetScanSegmentCache()
{
    m_scanSegmentCache.reset();
    qInfo(LOG_FLOW).noquote() << QStringLiteral("扫描段缓存已清空（含运行实例目录绑定）。");
}

void StateMachine::clearScanSegmentCacheForPathSwitch()
{
    m_scanSegmentCache.clearSegmentsKeepRunRoot();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已清段缓存并保留运行实例目录：")
        << m_scanSegmentCache.runCaptureRoot();
}

}  // namespace scan_tracking::flow_control
