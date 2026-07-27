#include "scan_tracking/flow_control/state_machine.h"

#include "scan_tracking/flow_control/detail/state_machine_internal.h"

#include "scan_tracking/common/config_manager.h"
#include "scan_tracking/flow_control/station2_inspection.h"
#include "scan_tracking/mech_eye/mech_eye_types.h"
#include "scan_tracking/vision/hik_camera_c_controller.h"
#include "scan_tracking/vision/vision_pipeline_service.h"
#include "scan_tracking/vision/vision_types.h"

#include <QtCore/QPointer>
#include <QtCore/QTimer>
#include <QtCore/QCoreApplication>
#include <QtCore/QThread>

#include <cstring>
#include <thread>
#include <utility>

namespace scan_tracking::flow_control {

using namespace state_machine_internal;

int StateMachine::resolveExpectedScanSegmentCount() const
{
    const auto* configMgr = common::ConfigManager::instance();
    if (configMgr == nullptr) {
        return 0;
    }
    const int fromScanPaths = configMgr->enabledScanPointCount();
    if (fromScanPaths > 0) {
        return fromScanPaths;
    }
    return configMgr->trackingConfig().scanSegmentTotal;
}

InspectionQuota StateMachine::buildActiveInspectionQuota() const
{
    InspectionQuota quota;
    if (const auto* configMgr = common::ConfigManager::instance()) {
        quota.expectedArmCount = configMgr->enabledArmPointCount();
        quota.expectedTelescopicCount = configMgr->enabledTelescopicPointCount();
        quota.pathId = configMgr->activePathId();
        quota.pathName = configMgr->activePathName();
        quota.algorithm = configMgr->activePathAlgorithm();
        if (quota.total() <= 0) {
            quota.expectedArmCount = resolveExpectedScanSegmentCount();
        }
    }
    return quota;
}

bool StateMachine::isBackgroundMeasurableAlgorithm(const QString& algorithm)
{
    const QString trimmed = algorithm.trimmed();
    return trimmed == QLatin1String("weld_section") ||
           trimmed == QLatin1String("thickness_inner_surface") ||
           trimmed == QLatin1String("length_volume");
}

InspectionResult StateMachine::evaluateCachedInspection(quint32 taskId) const
{
    const quint32 effectiveTaskId = taskId != 0 ? taskId : m_scanSegmentCache.runTaskId();
    const InspectionQuota quota = buildActiveInspectionQuota();

    // GUI/主线程禁止阻塞抢评估锁，否则会卡死事件循环与 Modbus 轮询。
    if (QCoreApplication::instance() != nullptr &&
        QThread::currentThread() == QCoreApplication::instance()->thread()) {
        InspectionResult result;
        if (!tryEvaluateStation2Inspection(
                m_scanSegmentCache, effectiveTaskId, quota, &result)) {
            qWarning(LOG_FLOW).noquote() << result.message;
        }
        return result;
    }

    return evaluateStation2Inspection(m_scanSegmentCache, effectiveTaskId, quota);
}

InspectionResult StateMachine::lastInspectionResult() const
{
    return m_lastInspectionResult;
}

InspectionResult StateMachine::evaluateInspectionForActiveTask() const
{
    return evaluateCachedInspection(m_activeTask.taskId);
}

void StateMachine::finishInspection(const InspectionResult& result)
{
    m_codeReadPending = false;
    m_codeReadCameraIp.clear();

    if (!acceptWorkpieceGeneration(
            m_activeTask.workpieceGeneration, QStringLiteral("finishInspection"))) {
        return;
    }

    m_lastInspectionResult = result;
    m_hasLastInspectionResult = true;

    publishInspectionOutcome(result, QStringLiteral("Trig_Inspection"));
    markCurrentPathInspectionDone();
    if (result.resultCode == 1) {
        const int pathId = result.pathId > 0
            ? result.pathId
            : (common::ConfigManager::instance() != nullptr
                   ? common::ConfigManager::instance()->activePathId()
                   : 0);
        maybeEmitPathFinished(pathId, result.resultCode);
    }

    const bool dataValid = result.resultCode == 1 || result.resultCode == 2;
    const protocol::AckState ackState = dataValid ? protocol::AckState::Completed
                                                  : protocol::AckState::Failed;
    completeActiveTask(result.resultCode, ackState, dataValid);

    // 检测成功后由 IPC 自清缓存并切下一条路径；不依赖 PLC Trig_ResultReset / pathId。
    if (result.resultCode == 1) {
        prepareNextScanPathAfterSuccess();
    }
}

void StateMachine::writeFakeInspectionPlcSuccess()
{
    InspectionSummary summary;
    summary.resultCode = 1;
    summary.ngReasonWord0 = 0;
    summary.ngReasonWord1 = 0;
    summary.measureItemCount = 0;
    writeInspectionResult(summary);
}

void StateMachine::publishInspectionOutcomeToHmiOnly(
    const InspectionResult& result,
    const QString& triggerLabel)
{
    m_lastInspectionResult = result;
    m_hasLastInspectionResult = true;

    emit inspectionFinished(
        result.resultCode,
        result.ngReasonWord0,
        result.ngReasonWord1,
        result.measureItemCount,
        result.measurement,
        result.message);

    emit inspectionResultReady(result);

    qInfo(LOG_FLOW).noquote()
        << triggerLabel << QStringLiteral("：后台真结果（仅 HMI/内存） Res=")
        << result.resultCode
        << QStringLiteral(" pathId=") << result.pathId
        << QStringLiteral(" pathName=") << result.pathName
        << QStringLiteral(" algorithm=") << result.algorithm
        << QStringLiteral(" qualityCode=") << result.measurement.qualityCode
        << QStringLiteral(" lengthMm=") << result.measurement.lengthMm
        << QStringLiteral(" volumeL=") << result.measurement.volumeLiters
        << QStringLiteral(" thicknessMm=") << result.measurement.thicknessMm
        << QStringLiteral(" segments=") << result.sourcePointCount
        << QStringLiteral(" message=") << result.message;
}

void StateMachine::applyBackgroundInspectionResult(
    quint64 generation,
    const InspectionResult& result,
    const QString& triggerLabel)
{
    if (!acceptWorkpieceGeneration(generation, triggerLabel + QStringLiteral(".bgDone"))) {
        return;
    }
    publishInspectionOutcomeToHmiOnly(result, triggerLabel);
}

void StateMachine::joinBackgroundInspectionSolves()
{
    std::thread running;
    {
        std::lock_guard<std::mutex> lock(m_bgSolveThreadsMutex);
        // 立刻丢掉未执行快照，避免 stop 期间继续拖住点云。
        m_bgSolvePending.reset();
        running = std::move(m_bgSolveThread);
    }
    if (!running.joinable()) {
        std::lock_guard<std::mutex> lock(m_bgSolveThreadsMutex);
        m_bgSolveWorkerBusy = false;
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("等待后台解算线程结束…");
    running.join();
    {
        std::lock_guard<std::mutex> lock(m_bgSolveThreadsMutex);
        m_bgSolveWorkerBusy = false;
        m_bgSolvePending.reset();
    }
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("后台解算线程已接合。");
}

void StateMachine::startBackgroundInspectionSolve(
    const ScanSegmentCache& cacheSnapshot,
    quint32 taskId,
    const InspectionQuota& quota,
    quint64 generation,
    const QString& triggerLabel)
{
    qInfo(LOG_FLOW).noquote()
        << triggerLabel << QStringLiteral("：PLC 假成功放行，启动后台解算 pathId=")
        << quota.pathId
        << QStringLiteral(" name=") << quota.pathName
        << QStringLiteral(" algorithm=") << quota.algorithm
        << QStringLiteral(" taskId=") << taskId
        << QStringLiteral(" cacheSeg=") << cacheSnapshot.cachedSegmentCount()
        << QStringLiteral(" workpieceGen=") << generation;

    // 工作线程禁止使用 QPointer（非线程安全）。判活/是否回投只读 shared atomic；
    // receiver 仅在 gate 仍开且 stop 尚未 join 完成时用于 invokeMethod（对象仍存活）。
    const auto acceptResults = m_bgSolveAcceptResults;
    StateMachine* const receiver = this;

    BackgroundInspectionJob job;
    job.cacheSnapshot = cacheSnapshot;
    job.taskId = taskId;
    job.quota = quota;
    job.generation = generation;
    job.triggerLabel = triggerLabel;

    std::thread finishedThread;
    {
        std::lock_guard<std::mutex> lock(m_bgSolveThreadsMutex);
        if (m_stopped.load(std::memory_order_acquire) ||
            !acceptResults->load(std::memory_order_acquire)) {
            qWarning(LOG_FLOW).noquote()
                << triggerLabel << QStringLiteral("：StateMachine 已 stop，跳过后台解算。");
            return;
        }

        // 已有执行中任务：只保留最新 pending，丢掉更早未执行快照（内存上限=执行中+1）。
        if (m_bgSolveWorkerBusy) {
            if (m_bgSolvePending.has_value()) {
                qWarning(LOG_FLOW).noquote()
                    << QStringLiteral("后台解算繁忙，丢弃未执行任务 pathId=")
                    << m_bgSolvePending->quota.pathId
                    << QStringLiteral(" algorithm=") << m_bgSolvePending->quota.algorithm
                    << QStringLiteral("，改排 pathId=") << quota.pathId
                    << QStringLiteral(" algorithm=") << quota.algorithm;
            } else {
                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("后台解算进行中，新任务进入唯一等待槽 pathId=")
                    << quota.pathId
                    << QStringLiteral(" algorithm=") << quota.algorithm;
            }
            m_bgSolvePending = std::move(job);
            return;
        }

        m_bgSolveWorkerBusy = true;
        if (m_bgSolveThread.joinable()) {
            finishedThread = std::move(m_bgSolveThread);
        }
        m_bgSolveThread = std::thread(
            [acceptResults, receiver, job = std::move(job)]() mutable {
                for (;;) {
                    const quint64 generation = job.generation;
                    const QString triggerLabel = job.triggerLabel;
                    const InspectionResult result = evaluateStation2Inspection(
                        job.cacheSnapshot, job.taskId, job.quota);
                    // 解算结束立刻丢掉点云快照，只保留结果回投所需字段。
                    job.cacheSnapshot.reset();

                    if (acceptResults->load(std::memory_order_acquire)) {
                        QMetaObject::invokeMethod(
                            receiver,
                            [acceptResults, receiver, generation, result, triggerLabel]() {
                                if (!acceptResults->load(std::memory_order_acquire)) {
                                    return;
                                }
                                receiver->applyBackgroundInspectionResult(
                                    generation, result, triggerLabel);
                            },
                            Qt::QueuedConnection);
                    }

                    {
                        std::lock_guard<std::mutex> lock(receiver->m_bgSolveThreadsMutex);
                        if (!acceptResults->load(std::memory_order_acquire) ||
                            !receiver->m_bgSolvePending.has_value()) {
                            receiver->m_bgSolvePending.reset();
                            receiver->m_bgSolveWorkerBusy = false;
                            return;
                        }
                        job = std::move(*receiver->m_bgSolvePending);
                        receiver->m_bgSolvePending.reset();
                    }
                }
            });
    }

    // 上一轮已退出的线程在锁外接合，避免 joinable std::thread 泄漏堆积。
    if (finishedThread.joinable()) {
        finishedThread.join();
    }
}

void StateMachine::releaseInspectionAndSolveInBackground()
{
    const InspectionQuota quota = buildActiveInspectionQuota();
    const QString algorithm = quota.algorithm.trimmed();

    if (!isBackgroundMeasurableAlgorithm(algorithm)) {
        // 未接入/不适用：仍同步回真实 Res，避免 PLC 误以为测完。
        const InspectionResult result = evaluateInspectionForActiveTask();
        finishInspection(result);
        return;
    }

    if (!isActivePathQuotaComplete() || m_scanSegmentCache.cachedSegmentCount() == 0) {
        const InspectionResult result = evaluateInspectionForActiveTask();
        finishInspection(result);
        return;
    }

    // 先快照再假成功/切路，点云经 shared_ptr 共享，切路清缓存不影响快照存活。
    const ScanSegmentCache cacheSnapshot = m_scanSegmentCache;
    const quint32 taskId =
        m_activeTask.taskId != 0 ? m_activeTask.taskId : cacheSnapshot.runTaskId();
    const quint64 generation = workpieceGeneration();

    markCurrentPathInspectionDone();
    writeFakeInspectionPlcSuccess();
    maybeEmitPathFinished(quota.pathId, 1);
    completeActiveTask(1, protocol::AckState::Completed, true);

    // 启动后台后再清缓存切路（prepareNext 内 AutoInspection 因已 mark 而跳过）。
    startBackgroundInspectionSolve(
        cacheSnapshot,
        taskId,
        quota,
        generation,
        QStringLiteral("Trig_Inspection"));
    prepareNextScanPathAfterSuccess();
}

bool StateMachine::isActiveCodeReadTrigger() const
{
    return m_activeTask.definition != nullptr &&
           std::strcmp(m_activeTask.definition->name, "Trig_CodeRead") == 0;
}

QString StateMachine::currentInspectionRunKey() const
{
    if (!m_scanSegmentCache.runCaptureRoot().isEmpty()) {
        return m_scanSegmentCache.runCaptureRoot();
    }
    return QStringLiteral("task%1_n%2")
        .arg(m_scanSegmentCache.runTaskId())
        .arg(m_scanSegmentCache.cachedSegmentCount());
}

void StateMachine::markCurrentPathInspectionDone()
{
    const auto* cfgMgr = common::ConfigManager::instance();
    m_lastInspectedPathId = cfgMgr != nullptr ? cfgMgr->activePathId() : -1;
    m_lastInspectedRunKey = currentInspectionRunKey();
}

bool StateMachine::alreadyInspectedCurrentPathRun() const
{
    const auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return false;
    }
    return m_lastInspectedPathId == cfgMgr->activePathId() &&
           !m_lastInspectedRunKey.isEmpty() &&
           m_lastInspectedRunKey == currentInspectionRunKey();
}

void StateMachine::publishInspectionOutcome(
    const InspectionResult& result,
    const QString& triggerLabel)
{
    InspectionSummary summary;
    summary.resultCode = result.resultCode;
    summary.ngReasonWord0 = result.ngReasonWord0;
    summary.ngReasonWord1 = result.ngReasonWord1;
    summary.measureItemCount = result.measureItemCount;
    writeInspectionResult(summary);

    if (!result.measurement.codeValue.isEmpty() && isModbusConnected()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            result.measurement.codeValue);
    }

    emit inspectionFinished(
        result.resultCode,
        result.ngReasonWord0,
        result.ngReasonWord1,
        result.measureItemCount,
        result.measurement,
        result.message);

    emit inspectionResultReady(result);

    qInfo(LOG_FLOW).noquote()
        << triggerLabel << QStringLiteral("：已完成 Res=") << result.resultCode
        << QStringLiteral(" pathId=") << result.pathId
        << QStringLiteral(" pathName=") << result.pathName
        << QStringLiteral(" algorithm=") << result.algorithm
        << QStringLiteral(" qualityCode=") << result.measurement.qualityCode
        << QStringLiteral(" lengthMm=") << result.measurement.lengthMm
        << QStringLiteral(" volumeL=") << result.measurement.volumeLiters
        << QStringLiteral(" thicknessMm=") << result.measurement.thicknessMm
        << QStringLiteral(" segments=") << result.sourcePointCount
        << QStringLiteral(" message=") << result.message;
}

void StateMachine::maybeAutoRunInspectionBeforeLeavingPath()
{
    // 临时策略：PLC 暂不发 Trig_Inspection 时，切路/清缓存前自动跑齐套路径的算法。
    if (alreadyInspectedCurrentPathRun()) {
        return;
    }
    if (!isActivePathQuotaComplete()) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("切路前跳过自动检测：当前路径未齐套（缓存不足）。");
        return;
    }

    const InspectionQuota quota = buildActiveInspectionQuota();
    const QString algorithm = quota.algorithm.trimmed();
    if (algorithm.isEmpty() ||
        algorithm == QLatin1String("code_read") ||
        algorithm == QLatin1String("self_check") ||
        algorithm == QLatin1String("weld_pending") ||
        !isBackgroundMeasurableAlgorithm(algorithm)) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("切路前跳过自动检测：algorithm=")
            << (algorithm.isEmpty() ? QStringLiteral("<empty>") : algorithm)
            << QStringLiteral("（由专用触发处理或不适用）。");
        return;
    }

    if (m_scanSegmentCache.cachedSegmentCount() == 0) {
        return;
    }

    const ScanSegmentCache cacheSnapshot = m_scanSegmentCache;
    const quint32 taskId = cacheSnapshot.runTaskId();
    const quint64 generation = workpieceGeneration();

    markCurrentPathInspectionDone();

    // PLC 假成功：写检测结果字 + Inspection 通道 Ack/Res，放行观察/联锁。
    writeFakeInspectionPlcSuccess();
    maybeEmitPathFinished(quota.pathId, 1);
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::Inspection) {
        if (const protocol::TriggerDefinition* inspectionTrig = protocol::triggerByOffset(
                protocol::registers::modbusIndexFromPlcAddress(40024))) {
            sendRes(*inspectionTrig, 1);
            sendAck(*inspectionTrig, protocol::AckState::Completed);
        }
    }

    startBackgroundInspectionSolve(
        cacheSnapshot,
        taskId,
        quota,
        generation,
        QStringLiteral("AutoInspection"));
}

void StateMachine::prepareNextScanPathAfterSuccess()
{
    // PLC 不做 ResultReset：检测成功 / 开下一路时 IPC 自行清段缓存与扫描完成寄存器，再切路径。
    // 故意保留本次 Inspection 结果寄存器，供 PLC/HMI 读取。
    maybeAutoRunInspectionBeforeLeavingPath();

    clearScanSegmentCacheForPathSwitch();
    if (isModbusConnected()) {
        clearScanSegmentDoneRegisters();
    }

    auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return;
    }

    const int fromPathId = cfgMgr->activePathId();
    const QString fromName = cfgMgr->activePathName();
    const int toPathId = cfgMgr->advanceToNextEnabledPath();
    if (toPathId <= 0) {
        qWarning(LOG_FLOW) << QStringLiteral("已清缓存，但无下一条启用路径可切换。");
        return;
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已自动清缓存并切换扫描路径：")
        << fromPathId << QStringLiteral("(") << fromName << QStringLiteral(")")
        << QStringLiteral(" -> ") << toPathId
        << QStringLiteral("(") << cfgMgr->activePathName() << QStringLiteral(")")
        << QStringLiteral(" algorithm=") << cfgMgr->activePathAlgorithm()
        << QStringLiteral(" 配额 arm=") << cfgMgr->enabledArmPointCount()
        << QStringLiteral(" telescopic=") << cfgMgr->enabledTelescopicPointCount()
        << QStringLiteral("；PLC 可直接按新路径从段号 1 继续采集。");

    // 新路径立刻告知 HMI「当前进行中」（不必等下一段扫到才 started）
    m_emittedPathStarted.remove(toPathId);
    maybeEmitPathStarted(toPathId);
}

bool StateMachine::applyPlcScanPathId(
    const QVector<quint16>& commandBlock,
    const QVector<quint16>& previousCommandBlock,
    bool onlyOnChange)
{
    namespace regs = protocol::registers;
    if (regs::kScanPathId < 0 || regs::kScanPathId >= commandBlock.size()) {
        return false;
    }

    const int requested = static_cast<int>(
        regs::plcAnalogToUInt16(commandBlock.value(regs::kScanPathId), 0));
    // 0=未指定：沿用当前活跃路径（含 IPC 自动切路结果）。
    if (requested <= 0) {
        return false;
    }

    if (onlyOnChange && !previousCommandBlock.isEmpty() &&
        regs::kScanPathId < previousCommandBlock.size()) {
        const int previousRequested = static_cast<int>(
            regs::plcAnalogToUInt16(previousCommandBlock.value(regs::kScanPathId), 0));
        if (previousRequested == requested) {
            return false;
        }
    }

    auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return false;
    }

    const int fromPathId = cfgMgr->activePathId();
    if (fromPathId == requested) {
        return false;
    }

    // 先对即将离开的路径做自动检测（需仍指向旧 pathId 且缓存未清）。
    maybeAutoRunInspectionBeforeLeavingPath();

    if (!cfgMgr->setActivePathId(requested)) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("PLC ScanPathId(40047)=") << requested
            << QStringLiteral(" 无效，保持当前 pathId=") << fromPathId;
        return false;
    }

    // pathId 变化：清段缓存与扫描 Done，避免旧路径段混入新路径检测；保留同一运行实例落盘目录。
    clearScanSegmentCacheForPathSwitch();
    if (isModbusConnected()) {
        clearScanSegmentDoneRegisters();
    }

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("已按 PLC ScanPathId(40047) 切换扫描路径：")
        << fromPathId << QStringLiteral(" -> ") << requested
        << QStringLiteral("(") << cfgMgr->activePathName() << QStringLiteral(")")
        << QStringLiteral(" algorithm=") << cfgMgr->activePathAlgorithm()
        << QStringLiteral(" 配额 arm=") << cfgMgr->enabledArmPointCount()
        << QStringLiteral(" telescopic=") << cfgMgr->enabledTelescopicPointCount();

    // PLC 切路：若离开的路径此前已检测完成则补 finished；新路径允许重新 started
    if (m_lastInspectedPathId == fromPathId) {
        maybeEmitPathFinished(fromPathId, 1);
    }
    m_emittedPathStarted.remove(requested);
    m_emittedPathFinished.remove(requested);
    m_emittedAllPathsFinished = false;
    maybeEmitPathStarted(requested);
    return true;
}

bool StateMachine::isActivePathQuotaComplete() const
{
    const auto* cfgMgr = common::ConfigManager::instance();
    if (cfgMgr == nullptr) {
        return false;
    }
    return m_scanSegmentCache.meetsDeviceQuotas(
        cfgMgr->enabledArmPointCount(), cfgMgr->enabledTelescopicPointCount());
}

void StateMachine::maybeAdvancePathOnNewCycleStart(int localIndex)
{
    if (localIndex != 1) {
        return;
    }
    if (!isActivePathQuotaComplete()) {
        return;
    }

    const int fromPathId =
        common::ConfigManager::instance() != nullptr
            ? common::ConfigManager::instance()->activePathId()
            : 0;
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("检测到 pathId=") << fromPathId
        << QStringLiteral(" 已齐套且 PLC 再次下发段号 1：按新路径周期自动切换（无需 Inspection/ResultReset）。");
    prepareNextScanPathAfterSuccess();
}

void StateMachine::startCodeReadCapture()
{
    auto* hik = m_hikCameraCController;
    if (hik == nullptr || !hik->isStarted()) {
        finishCodeRead(2, QString(), QStringLiteral("海康智能相机 C 未启动，无法编号识别。"));
        return;
    }

    QString cameraIp;
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        cameraIp = cfgMgr->visionConfig().armGroup.hikCameraC.ipAddress.trimmed();
    }
    if (cameraIp.isEmpty()) {
        cameraIp = hik->configuredCameraIps().value(0).trimmed();
    }
    if (cameraIp.isEmpty()) {
        finishCodeRead(2, QString(), QStringLiteral("未配置机械臂侧海康 C IP。"));
        return;
    }
    if (!hik->isCameraConnected(cameraIp)) {
        finishCodeRead(
            2,
            QString(),
            QStringLiteral("机械臂侧海康 C 未连接：%1").arg(cameraIp));
        return;
    }

    const bool scanStage =
        m_activeTask.definition != nullptr &&
        isScanCaptureStage(m_activeTask.definition->stage);
    QString algorithm;
    if (const auto* cfgMgr = common::ConfigManager::instance()) {
        algorithm = cfgMgr->activePathAlgorithm();
    }
    const bool inspectionCodeRead =
        m_activeTask.definition != nullptr &&
        m_activeTask.definition->stage == protocol::Stage::Inspection &&
        algorithm == QLatin1String("code_read");
    // 段扫/检测上的编号：发完 start 即放行。延后一拍再写终态 Ack，
    // 给 PLC 采样时间（段扫已无 Ack=1，终态为 Idle→2/3）。
    const bool fireAndForget = scanStage || inspectionCodeRead;

    m_codeReadPending = !fireAndForget;
    m_codeReadSoftPending = fireAndForget;
    m_codeReadCameraIp = cameraIp;
    m_codeReadWorkpieceGeneration = workpieceGeneration();
    setTaskProgress(30);
    publishIpcStatus();

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("编号识别：向海康 C 发送 NumberRecognition")
        << QStringLiteral(" ip=") << cameraIp
        << QStringLiteral(" pathId=")
        << (common::ConfigManager::instance()
                ? common::ConfigManager::instance()->activePathId()
                : 0)
        << QStringLiteral(" trigger=")
        << (m_activeTask.definition != nullptr
                ? protocol::triggerName(*m_activeTask.definition)
                : QStringLiteral("-"))
        << (fireAndForget ? QStringLiteral(" mode=fire_and_forget")
                          : QStringLiteral(" mode=wait_ocr"))
        << QStringLiteral(" workpieceGen=") << m_codeReadWorkpieceGeneration;

    if (!hik->requestCapture(vision::CaptureType::NumberRecognition, cameraIp)) {
        m_codeReadSoftPending = false;
        m_codeReadWorkpieceGeneration = 0;
        finishCodeRead(
            2,
            QString(),
            QStringLiteral("向海康 C 发送编号拍照失败：%1").arg(cameraIp));
        return;
    }

    if (fireAndForget) {
        const quint64 generation = m_codeReadWorkpieceGeneration;
        QPointer<StateMachine> self(this);
        QTimer::singleShot(150, this, [self, generation]() {
            if (self == nullptr) {
                return;
            }
            if (!self->acceptWorkpieceGeneration(
                    generation, QStringLiteral("codeRead.fireAndForgetAck"))) {
                return;
            }
            // OCR 可能已先到并清掉 softPending；只要任务未收尾仍需写 Ack=Completed。
            if (self->m_activeTask.definition == nullptr ||
                self->m_activeTask.completionAnnounced) {
                return;
            }
            self->finishCodeRead(
                1,
                QString(),
                QStringLiteral("编号采集已触发（不等待 OCR；回包稍后写寄存器）"));
        });
    }
}

void StateMachine::onHikOcrTextReceived(QString cameraIp, QString text)
{
    const QString code = text.trimmed();
    if (code.isEmpty()) {
        if (m_codeReadPending) {
            finishCodeRead(2, QString(), QStringLiteral("OCR 回包为空。"));
        }
        return;
    }

    if (m_codeReadPending) {
        if (!acceptWorkpieceGeneration(
                m_codeReadWorkpieceGeneration, QStringLiteral("codeRead.ocrPending"))) {
            return;
        }
        if (!m_codeReadCameraIp.isEmpty() && cameraIp.trimmed() != m_codeReadCameraIp) {
            return;
        }
        finishCodeRead(1, code, QStringLiteral("编号识别成功：%1").arg(code));
        return;
    }

    if (m_codeReadSoftPending) {
        applyLateCodeReadOcr(cameraIp, code);
    }
}

void StateMachine::applyLateCodeReadOcr(const QString& cameraIp, const QString& codeValue)
{
    if (!acceptWorkpieceGeneration(
            m_codeReadWorkpieceGeneration, QStringLiteral("codeRead.ocrLate"))) {
        m_codeReadSoftPending = false;
        m_codeReadCameraIp.clear();
        m_codeReadWorkpieceGeneration = 0;
        return;
    }
    if (!m_codeReadCameraIp.isEmpty() && cameraIp.trimmed() != m_codeReadCameraIp) {
        return;
    }

    m_codeReadSoftPending = false;
    m_codeReadCameraIp.clear();
    m_codeReadWorkpieceGeneration = 0;

    if (isModbusConnected()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            codeValue);
    }
    notifyCodeReadFinished(1, codeValue);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("编号 OCR 迟到回包已写入寄存器 code=") << codeValue
        << QStringLiteral(" ip=") << cameraIp;
}

void StateMachine::scheduleCodeReadScanFinalizeWatchdog(int trigOffset)
{
    // 给 PLC 留出采样 Done/Ack/Res 的时间；超时仍占着任务会导致后续触发全部被忽略。
    const int holdDoneMs = 500;
    const quint64 generation = workpieceGeneration();
    QPointer<StateMachine> self(this);
    QTimer::singleShot(holdDoneMs, this, [self, trigOffset, holdDoneMs, generation]() {
        if (self == nullptr) {
            return;
        }
        if (!self->acceptWorkpieceGeneration(
                generation, QStringLiteral("codeRead.finalizeWatchdog"))) {
            return;
        }
        if (self->m_activeTask.definition == nullptr ||
            !self->m_activeTask.completionAnnounced) {
            return;
        }
        if (self->m_activeTask.definition->trigOffset != trigOffset) {
            return;
        }

        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("编号段扫完成已超过 ") << holdDoneMs
            << QStringLiteral("ms，PLC 仍未拉低 Trig，强制释放任务以免卡死；")
            << QStringLiteral("该 Trig 需先回 0 才可再次触发。");
        self->m_blockTrigUntilIdleOffset = trigOffset;
        self->finalizeCompletedTaskIfTriggerReleased(self->m_lastCommandBlock, true);
    });
}

void StateMachine::finishCodeRead(quint16 resultCode, const QString& codeValue, const QString& message)
{
    if (m_activeTask.definition == nullptr || m_activeTask.completionAnnounced) {
        m_codeReadPending = false;
        if (!m_codeReadSoftPending) {
            m_codeReadCameraIp.clear();
        }
        return;
    }

    const bool scanStage = isScanCaptureStage(m_activeTask.definition->stage);
    // fire-and-forget 成功收尾后保留 softPending，供迟到 OCR 写寄存器。
    const bool keepSoftPending = resultCode == 1 && m_codeReadSoftPending;

    m_codeReadPending = false;
    if (!keepSoftPending) {
        m_codeReadSoftPending = false;
        m_codeReadCameraIp.clear();
    }

    const QString effectiveMessage = message.isEmpty()
        ? (resultCode == 1
               ? QStringLiteral("编号识别成功")
               : QStringLiteral("编号识别失败"))
        : message;

    if (isModbusConnected() && !codeValue.isEmpty()) {
        writeAsciiPlaceholder(
            protocol::registers::kCodeValueAscii,
            protocol::registers::kCodeValueRegisterCount,
            resultCode == 1 ? codeValue : QString());
    }

    // Trig_ScanSegment / Trig_TelescopicScan 兼跑编号：回写段扫寄存器，并占位齐套缓存。
    if (scanStage) {
        const QString triggerLabel = protocol::triggerName(*m_activeTask.definition);
        const int trigOffset = m_activeTask.definition->trigOffset;
        const bool ok = resultCode == 1;
        bool shouldAdvancePath = false;
        if (ok) {
            const auto device =
                m_activeTask.definition->stage == protocol::Stage::TelescopicScan
                    ? common::ScanDeviceKind::Telescopic
                    : common::ScanDeviceKind::Arm;
            vision::MultiCameraCaptureBundle placeholder;
            placeholder.request.segmentIndex = m_activeTask.scanSegmentIndex;
            placeholder.request.taskId = m_activeTask.taskId;
            m_scanSegmentCache.storeSegment(
                device,
                m_activeTask.scanSegmentIndex,
                m_activeTask.taskId,
                std::move(placeholder));

            const auto* configMgr = common::ConfigManager::instance();
            const int armExpected =
                configMgr != nullptr ? configMgr->enabledArmPointCount() : 0;
            const int telescopicExpected =
                configMgr != nullptr ? configMgr->enabledTelescopicPointCount() : 0;
            shouldAdvancePath =
                m_scanSegmentCache.meetsDeviceQuotas(armExpected, telescopicExpected);
            if (shouldAdvancePath) {
                qInfo(LOG_FLOW).noquote()
                    << QStringLiteral("pathId=")
                    << (configMgr != nullptr ? configMgr->activePathId() : 0)
                    << QStringLiteral(" 编号段扫齐套（Done/Ack/Res 保持至 PLC 拉低 Trig"
                                      "；超时将强制释放）。");
            }
        }

        notifyCodeReadFinished(resultCode, codeValue);

        // 须在 complete 之前置位：complete 内可能立刻 finalize（Trig 已为 0）。
        if (shouldAdvancePath) {
            m_advancePathAfterTriggerRelease = true;
            qInfo(LOG_FLOW).noquote()
                << QStringLiteral("编号段扫齐套：保留 Done/Ack/Res 供 PLC 读取，"
                                  "待 Trig 释放（或看门狗超时）后再自动切下一路。");
        }

        // PLC 常同时看 DoneIndex + ImageCount/CloudFrameCount；cloud=0 可能被当成无有效应答。
        completeScanSegmentCapture(
            ok ? 1 : 7,
            ok ? 1 : 0,
            ok ? 1 : 0,
            ok ? protocol::AckState::Completed : protocol::AckState::Failed,
            ok);

        qInfo(LOG_FLOW).noquote()
            << triggerLabel << QStringLiteral("（编号）：已完成 Res=") << (ok ? 1 : 7)
            << QStringLiteral(" code=") << codeValue
            << QStringLiteral(" message=") << effectiveMessage
            << (keepSoftPending ? QStringLiteral(" softOcr=1") : QString());

        // 若 PLC 一直拉高 Trig（例如等 OCR 文本），必须靠看门狗强制收尾。
        if (m_activeTask.definition != nullptr && m_activeTask.completionAnnounced) {
            scheduleCodeReadScanFinalizeWatchdog(trigOffset);
        }
        return;
    }

    // Trig_Inspection + algorithm=code_read：走 Inspection 结果通道，便于 HMI 统一展示
    if (!isActiveCodeReadTrigger()) {
        InspectionResult result;
        if (const auto* cfgMgr = common::ConfigManager::instance()) {
            result.pathId = cfgMgr->activePathId();
            result.pathName = cfgMgr->activePathName();
            result.algorithm = cfgMgr->activePathAlgorithm();
        }
        result.resultCode = resultCode;
        result.measureItemCount = resultCode == 1 ? 1 : 0;
        result.measurement.qualityCode = resultCode == 1 ? 1 : 2;
        result.measurement.codeValue = codeValue;
        result.message = effectiveMessage;
        finishInspection(result);
        notifyCodeReadFinished(resultCode, codeValue);
        return;
    }

    const bool dataValid = resultCode == 1;
    const protocol::AckState ackState =
        dataValid ? protocol::AckState::Completed : protocol::AckState::Failed;
    completeActiveTask(resultCode, ackState, dataValid);
    notifyCodeReadFinished(resultCode, codeValue);

    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("Trig_CodeRead：已完成 Res=") << resultCode
        << QStringLiteral(" code=") << codeValue
        << QStringLiteral(" message=") << effectiveMessage;

    if (resultCode == 1) {
        if (const auto* cfgMgr = common::ConfigManager::instance()) {
            maybeEmitPathFinished(cfgMgr->activePathId(), resultCode);
        }
        prepareNextScanPathAfterSuccess();
    }
}

void StateMachine::startSelfCheckCapture()
{
    auto* vision = m_visionPipeline;
    if (vision == nullptr || !vision->isStarted()) {
        constexpr quint16 kFailCxpOrVision = 1u << 2;
        writeSelfCheckFailWords({kFailCxpOrVision});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCxpOrVision);
        return;
    }

    vision::VisionPipelineService::BundleCaptureOptions options;
    options.useMechEye = true;
    options.useHikCxp = true;
    options.useHikSmartC = false;

    const int segmentIndex =
        m_activeTask.scanSegmentIndex > 0 ? m_activeTask.scanSegmentIndex : 1;
    const quint64 requestId = vision->requestCaptureBundle(
        segmentIndex,
        m_activeTask.taskId,
        mech_eye::CaptureMode::Capture2DAnd3D,
        false,
        options);
    if (requestId == 0) {
        constexpr quint16 kFailCapture = 1u << 4;
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        return;
    }

    m_activeTask.captureRequestId = requestId;
    setTaskProgress(40);
    publishIpcStatus();
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("自检：已发起组合采集 requestId=") << requestId
        << QStringLiteral(" 段号=") << segmentIndex
        << QStringLiteral(" channels=梅卡+CXP");
}

void StateMachine::finishSelfCheckCapture(const vision::MultiCameraCaptureBundle& bundle)
{
    if (m_activeTask.definition == nullptr ||
        m_activeTask.definition->stage != protocol::Stage::SelfCheck ||
        m_activeTask.completionAnnounced) {
        return;
    }
    if (bundle.request.requestId != m_activeTask.captureRequestId) {
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("自检采集完成忽略：requestId 不匹配 active=")
            << m_activeTask.captureRequestId
            << QStringLiteral(" bundle=") << bundle.request.requestId;
        return;
    }

    constexpr quint16 kFailCapture = 1u << 4;
    const bool cxpOk = bundle.cxpParticipated() &&
                       bundle.hikCameraAResult.success() &&
                       bundle.hikCameraBResult.success();
    const bool ok = bundle.mechEyeResult.success() && cxpOk;

    if (!ok) {
        writeSelfCheckFailWords({kFailCapture});
        completeActiveTask(2, protocol::AckState::Failed, false);
        notifySelfCheckFinished(2, kFailCapture);
        qWarning(LOG_FLOW).noquote()
            << QStringLiteral("自检采集失败：") << bundle.summary()
            << QStringLiteral(" cxpParticipated=") << bundle.cxpParticipated();
        return;
    }

    writeSelfCheckFailWords({0});
    completeActiveTask(1, protocol::AckState::Completed, true);
    notifySelfCheckFinished(1, 0);
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("自检采集通过：") << bundle.summary()
        << QStringLiteral(" 段号=") << m_activeTask.scanSegmentIndex;
}

}  // namespace scan_tracking::flow_control
