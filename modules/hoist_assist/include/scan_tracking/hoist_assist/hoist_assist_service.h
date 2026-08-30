#pragma once

#include <QtCore/QElapsedTimer>
#include <QtCore/QObject>

#include "scan_tracking/collision_monitor/collision_monitor_types.h"
#include "scan_tracking/hoist_assist/hoist_assist_types.h"

namespace scan_tracking::hoist_assist {

// 吊装辅助服务：汇总 TF 测距、Mid360 碰撞检测、海康 C 焊缝/ROI 三类结果并输出状态
//
// 职责：
// - 缓存各路传感器最新输入，并按周期重新判定
// - TF 采样超时后自动失效，避免断线后仍沿用旧数据通过检查
// - 任一类检查未通过时进入 Unsafe；全部通过则保持 Running
//
// 典型流程：start → 持续 update* / evaluate → stop
class HoistAssistService final : public QObject {
    Q_OBJECT

public:
    explicit HoistAssistService(QObject* parent = nullptr);

    /// 启动吊装辅助：复位输入并进入 Running，等待三类传感器结果。
    void start();
    /// 停止吊装辅助：进入 Stopped，不再根据输入切换 Unsafe/Running。
    void stop();
    /// 清空缓存的传感器结果与 TF 时间戳；运行中会回到等待结果状态。
    void resetInputs();

    bool isRunning() const { return m_running; }
    HoistAssistState state() const { return m_state; }
    const HoistAssistResult& result() const { return m_result; }

    /// 更新指定 TF 传感器距离。
    /// param distanceCm 原始 TF 距离（厘米）
    /// param valid 校验和、强度、超时或其它传输错误时应为 false
    void updateTfDistance(TfSensorId sensor, int distanceCm, bool valid);

    /// 按固定周期重新判断当前缓存的 TF、碰撞和海康结果。
    void evaluate();

    /// 更新 Mid360 碰撞检测结果。
    /// 首版将所有非 None 告警均视为不安全；细粒度停机/报警策略待对接 PLC 后补充。
    void updateCollisionResult(
        collision_monitor::CollisionAlertLevel level,
        bool valid);

    /// 更新第三路海康 C 焊缝/ROI 模型结果。
    /// 在真正收到相机回调之前，未收到结果不视为通过（也不据此直接判 Unsafe）。
    void updateHikCameraResult(bool ok, bool valid = true);

signals:
    /// 运行状态变化（或带有新的说明文案）时发出。
    void stateChanged(scan_tracking::hoist_assist::HoistAssistState state, QString message);
    /// 综合结果刷新时发出（含未运行时的输入更新）。
    void resultChanged(scan_tracking::hoist_assist::HoistAssistResult result);

private:
    /// 综合三类输入重新判定，并按需发布状态与结果信号。
    void recompute();
    /// 更新内部状态；状态变化或 message 非空时发出 stateChanged。
    void publishState(HoistAssistState state, const QString& message);

    bool m_running = false;
    HoistAssistState m_state = HoistAssistState::Idle;
    QString m_lastStateMessage;
    HoistAssistResult m_result;
    QElapsedTimer m_clock;           // 用于 TF 采样超时判定的单调时钟
    qint64 m_tf1LastUpdateMs = -1;   // TF1 最近一次更新时的 elapsed（毫秒），-1 表示尚无有效时间戳
    qint64 m_tf2LastUpdateMs = -1;   // TF2 最近一次更新时的 elapsed（毫秒），-1 表示尚无有效时间戳
};

}  // namespace scan_tracking::hoist_assist
