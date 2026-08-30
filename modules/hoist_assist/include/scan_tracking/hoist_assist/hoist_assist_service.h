#pragma once

#include <QtCore/QObject>

#include "scan_tracking/collision_monitor/collision_monitor_types.h"
#include "scan_tracking/hoist_assist/hoist_assist_types.h"

namespace scan_tracking::hoist_assist {

class HoistAssistService final : public QObject {
    Q_OBJECT

public:
    explicit HoistAssistService(QObject* parent = nullptr);

    void start();
    void stop();
    void resetInputs();

    bool isRunning() const { return m_running; }
    HoistAssistState state() const { return m_state; }
    const HoistAssistResult& result() const { return m_result; }

    // distanceCm is the raw TF value; valid must be false on checksum,
    // strength, timeout, or other transport errors.
    void updateTfDistance(TfSensorId sensor, int distanceCm, bool valid);

    /// 按固定周期重新判断当前缓存的 TF、碰撞和海康结果。
    void evaluate();

    // The first implementation intentionally treats every non-None alert as
    // unsafe. Detailed stop/alarm policy will be added with PLC integration.
    void updateCollisionResult(
        collision_monitor::CollisionAlertLevel level,
        bool valid);

    // Result from the third Hik C weld/ROI model. No result is considered
    // unsafe until the camera callback is actually received.
    void updateHikCameraResult(bool ok, bool valid = true);

signals:
    void stateChanged(scan_tracking::hoist_assist::HoistAssistState state, QString message);
    void resultChanged(scan_tracking::hoist_assist::HoistAssistResult result);

private:
    void recompute();
    void publishState(HoistAssistState state, const QString& message);

    bool m_running = false;
    HoistAssistState m_state = HoistAssistState::Idle;
    HoistAssistResult m_result;
};

}  // namespace scan_tracking::hoist_assist
