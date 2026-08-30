#include "scan_tracking/hoist_assist/hoist_assist_service.h"

namespace scan_tracking::hoist_assist {

namespace {

bool tf1Passes(const TfDistanceSample& sample)
{
    // TF1 (现场 COM5): 必须看到 2.35 m 以外的空间。
    return sample.valid && sample.distanceCm > 235;
}

bool tf2Passes(const TfDistanceSample& sample)
{
    // TF2 (现场 COM6): 对射筒体距离必须在 1.70 m 到 1.90 m 之间。
    return sample.valid && sample.distanceCm >= 170 && sample.distanceCm <= 190;
}

constexpr qint64 kTfSampleTimeoutMs = 1500;

}  // namespace

HoistAssistService::HoistAssistService(QObject* parent)
    : QObject(parent)
{
}

void HoistAssistService::start()
{
    if (m_running) {
        return;
    }
    m_clock.start();
    resetInputs();
    m_running = true;
    publishState(HoistAssistState::Running, QStringLiteral("吊装辅助已启动，等待三类传感器结果"));
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
    publishState(HoistAssistState::Stopped, QStringLiteral("吊装辅助已停止"));
}

void HoistAssistService::resetInputs()
{
    m_result = HoistAssistResult{};
    m_tf1LastUpdateMs = -1;
    m_tf2LastUpdateMs = -1;
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
}

void HoistAssistService::updateCollisionResult(
    collision_monitor::CollisionAlertLevel level,
    bool valid)
{
    m_result.collisionResultReceived = valid;
    m_result.collisionLevel = level;
    m_result.collisionSafe = valid && level == collision_monitor::CollisionAlertLevel::None;
}

void HoistAssistService::updateHikCameraResult(bool ok, bool valid)
{
    m_result.hikResultReceived = valid;
    m_result.hikPassed = valid && ok;
}

void HoistAssistService::recompute()
{
    // A disconnected sensor must not remain passed forever on its last frame.
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
    m_result.allChecksPassed = m_result.tfPassed && m_result.collisionSafe && m_result.hikPassed;

    if (!m_running) {
        emit resultChanged(m_result);
        return;
    }

    if (m_result.collisionResultReceived && !m_result.collisionSafe) {
        m_result.message = QStringLiteral("Mid360 碰撞检测未通过");
        publishState(HoistAssistState::Unsafe, m_result.message);
    } else if (m_result.tfPassed && m_result.collisionSafe && m_result.hikResultReceived
               && !m_result.hikPassed) {
        m_result.message = QStringLiteral("海康 C 焊缝/ROI 判定未通过");
        publishState(HoistAssistState::Unsafe, m_result.message);
    } else if (m_result.allChecksPassed) {
        m_result.message = QStringLiteral("吊装辅助检查全部通过");
        publishState(HoistAssistState::Running, m_result.message);
    } else {
        m_result.message = QStringLiteral("等待吊装辅助检查结果");
        publishState(HoistAssistState::Running, m_result.message);
    }
    emit resultChanged(m_result);
}

void HoistAssistService::publishState(HoistAssistState state, const QString& message)
{
    const bool changed = m_state != state;
    m_state = state;
    if (changed || !message.isEmpty()) {
        emit stateChanged(m_state, message);
    }
}

}  // namespace scan_tracking::hoist_assist
