#include "scan_tracking/hoist_assist/hoist_assist_service.h"

namespace scan_tracking::hoist_assist {

namespace {

bool tf1Passes(const TfDistanceSample& sample)
{
    // TF1（现场 COM5）：必须看到 2.35 m 以外的空间。
    return sample.valid && sample.distanceCm > 235;
}

bool tf2Passes(const TfDistanceSample& sample)
{
    // TF2（现场 COM6）：对射筒体距离必须在 1.70 m 到 1.90 m 之间。
    return sample.valid && sample.distanceCm >= 170 && sample.distanceCm <= 190;
}

// TF 采样超时阈值：超时后将该路 valid 置为 false，避免断线仍沿用旧帧通过检查
constexpr qint64 kTfSampleTimeoutMs = 1500;

// 后续若要把碰撞 / 海康重新计入成败，改为 true 即可，输入更新逻辑无需改。
constexpr bool kJudgeCollisionAndHik = false;

}  // namespace

HoistAssistService::HoistAssistService(QObject* parent)
    : QObject(parent)
{
    qRegisterMetaType<HoistAssistState>("scan_tracking::hoist_assist::HoistAssistState");
    qRegisterMetaType<HoistAssistResult>("scan_tracking::hoist_assist::HoistAssistResult");
}

void HoistAssistService::start()
{
    // 允许显控重复下发「开始吊装」：每轮都清空缓存，重新边沿判定。
    m_clock.start();
    m_result = HoistAssistResult{};
    m_tf1LastUpdateMs = -1;
    m_tf2LastUpdateMs = -1;
    m_lastOutcome = Outcome::None;
    m_running = true;
    publishState(HoistAssistState::Running, QStringLiteral("吊装辅助已启动，等待双 TF 定位结果"));
    emit resultChanged(m_result);
    recompute();
}

void HoistAssistService::evaluate()
{
    recompute();
}

void HoistAssistService::stop()
{
    if (!m_running && m_state == HoistAssistState::Stopped) {
        return;
    }
    m_running = false;
    m_lastOutcome = Outcome::None;
    publishState(HoistAssistState::Stopped, QStringLiteral("吊装辅助已停止"));
}

void HoistAssistService::resetInputs()
{
    m_result = HoistAssistResult{};
    m_tf1LastUpdateMs = -1;
    m_tf2LastUpdateMs = -1;
    m_lastOutcome = Outcome::None;
    if (m_running) {
        publishState(HoistAssistState::Running, QStringLiteral("吊装辅助输入已复位，等待结果"));
        emit resultChanged(m_result);
    } else {
        m_state = HoistAssistState::Idle;
    }
}

void HoistAssistService::updateTfDistance(TfSensorId sensor, int distanceCm, bool valid)
{
    TfDistanceSample sample;
    sample.distanceCm = distanceCm;
    sample.valid = valid;
    if (sensor == TfSensorId::Sensor1) {
        m_result.tf1 = sample;
        m_tf1LastUpdateMs = m_clock.isValid() ? m_clock.elapsed() : -1;
    } else {
        m_result.tf2 = sample;
        m_tf2LastUpdateMs = m_clock.isValid() ? m_clock.elapsed() : -1;
    }
    if (m_running) {
        recompute();
    }
}

void HoistAssistService::updateCollisionResult(
    collision_monitor::CollisionAlertLevel level,
    bool valid)
{
    m_result.collisionResultReceived = valid;
    m_result.collisionLevel = level;
    m_result.collisionSafe = valid && level == collision_monitor::CollisionAlertLevel::None;
    if (m_running) {
        recompute();
    }
}

void HoistAssistService::updateHikCameraResult(bool ok, bool valid)
{
    m_result.hikResultReceived = valid;
    m_result.hikPassed = valid && ok;
    if (m_running) {
        recompute();
    }
}

void HoistAssistService::recompute()
{
    // 传感器断线后不得继续用上一帧数据保持通过状态
    const qint64 nowMs = m_clock.isValid() ? m_clock.elapsed() : -1;
    if (nowMs >= 0) {
        if (m_result.tf1.valid
            && (m_tf1LastUpdateMs < 0 || nowMs - m_tf1LastUpdateMs > kTfSampleTimeoutMs)) {
            m_result.tf1.valid = false;
        }
        if (m_result.tf2.valid
            && (m_tf2LastUpdateMs < 0 || nowMs - m_tf2LastUpdateMs > kTfSampleTimeoutMs)) {
            m_result.tf2.valid = false;
        }
    }
    m_result.tfPassed = tf1Passes(m_result.tf1) && tf2Passes(m_result.tf2);
    m_result.allChecksPassed = kJudgeCollisionAndHik
        ? (m_result.tfPassed && m_result.collisionSafe && m_result.hikPassed)
        : m_result.tfPassed;
    m_result.failReason = HoistAssistFailReason::None;

    if (!m_running) {
        emit resultChanged(m_result);
        return;
    }

    // 本轮已通过则锁定：测距在阈值附近抖动时不再反复推 passed/failed，也不撤销拍照门禁。
    // 下一件须显控再发 cmd.start_hoist_assist（start 会清 m_lastOutcome）。
    if (m_lastOutcome == Outcome::Passed) {
        m_result.allChecksPassed = true;
        m_result.failReason = HoistAssistFailReason::None;
        m_result.message = QStringLiteral("吊装辅助 TF 定位通过");
        emit resultChanged(m_result);
        return;
    }

    // 碰撞 / 海康分支保留：kJudgeCollisionAndHik=true 时重新计入成败。
    if (kJudgeCollisionAndHik && m_result.collisionResultReceived && !m_result.collisionSafe) {
        m_result.failReason = HoistAssistFailReason::Collision;
        m_result.message = QStringLiteral("Mid360 碰撞检测未通过");
        publishState(HoistAssistState::Unsafe, m_result.message);
        publishOutcome(Outcome::Failed);
    } else if (m_result.tf1.valid && m_result.tf2.valid && !m_result.tfPassed) {
        m_result.failReason = HoistAssistFailReason::TfConstraint;
        m_result.message = QStringLiteral(
            "TF 定位约束未通过（TF1>235cm 且 TF2 在 170~190cm）");
        publishState(HoistAssistState::Unsafe, m_result.message);
        publishOutcome(Outcome::Failed);
    } else if (kJudgeCollisionAndHik && m_result.tfPassed && m_result.collisionSafe
               && m_result.hikResultReceived && !m_result.hikPassed) {
        m_result.failReason = HoistAssistFailReason::HikRoi;
        m_result.message = QStringLiteral("海康 C 焊缝/ROI 判定未通过");
        publishState(HoistAssistState::Unsafe, m_result.message);
        publishOutcome(Outcome::Failed);
    } else if (m_result.allChecksPassed) {
        m_result.message = QStringLiteral("吊装辅助 TF 定位通过");
        publishState(HoistAssistState::Running, m_result.message);
        publishOutcome(Outcome::Passed);
    } else {
        m_result.message = QStringLiteral("等待吊装辅助 TF 定位结果");
        publishState(HoistAssistState::Running, m_result.message);
        // 失败后回到等待态可再次判定通过；已通过则在上方锁定，不会走到这里。
        if (m_lastOutcome == Outcome::Failed) {
            m_lastOutcome = Outcome::None;
        }
    }
    emit resultChanged(m_result);
}

void HoistAssistService::publishOutcome(Outcome outcome)
{
    if (m_lastOutcome == outcome) {
        return;
    }
    m_lastOutcome = outcome;
    if (outcome == Outcome::Passed) {
        emit checkPassed(m_result);
    } else if (outcome == Outcome::Failed) {
        emit checkFailed(m_result);
    }
}

void HoistAssistService::publishState(HoistAssistState state, const QString& message)
{
    const bool changed = m_state != state;
    const bool messageChanged = m_lastStateMessage != message;
    m_state = state;
    if (changed || messageChanged) {
        m_lastStateMessage = message;
        emit stateChanged(m_state, message);
    }
}

}  // namespace scan_tracking::hoist_assist
