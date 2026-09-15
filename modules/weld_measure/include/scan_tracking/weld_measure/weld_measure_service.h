#pragma once

#include <cstddef>

#include <QtCore/QString>

#include "scan_tracking/weld_measure/weld_measure_types.h"

namespace scan_tracking::weld_measure {

/// WeldMeasure.dll（C API）的薄封装服务。
///
/// 职责：
/// - 持有不透明上下文 `wm_context*`，对外提供 Qt 友好接口；
/// - 同一实例内的初始化/测量/销毁调用串行化（内部互斥），避免并发进入 DLL；
/// - 经 SEH 包装层调用原生 API，捕获访问违例等原生异常，防止拖垮宿主进程。
///
/// 典型用法：`initializeFromIni()` → 多次 `measureFrame()` → `shutdown()` / 析构。
class WeldMeasureService {
public:
    WeldMeasureService();
    ~WeldMeasureService();

    WeldMeasureService(const WeldMeasureService&) = delete;
    WeldMeasureService& operator=(const WeldMeasureService&) = delete;

    /// 默认配置：`<applicationDir>/config/weld_measure/weld_measurement.ini`
    static QString defaultConfigPath();
    /// 机械臂直焊缝 ini（通常 11 帧）；优先 ascii 文件名，否则回退中文文件名。
    static QString defaultArmConfigPath();
    /// 伸缩杆直焊缝 ini（通常 7 帧）；优先 ascii 文件名，否则回退中文文件名。
    static QString defaultTelescopicConfigPath();
    /// 机械臂环焊缝 ini（通常 18 帧）。
    static QString defaultRingArmConfigPath();
    /// 伸缩杆环焊缝 ini（通常 18 帧）。
    static QString defaultRingTelescopicConfigPath();
    /// 默认 ONNX 模型：`<applicationDir>/models/weld_measure/pointnet_weld_seam_V7.3_good.onnx`
    static QString defaultModelPath();

    /// 是否已成功创建 DLL 上下文（可测量）。
    bool isReady() const;
    /// 最近一次 `initializeFromIni` 使用的配置路径；仅模型初始化时为空。
    QString configPath() const;
    /// 当前关联的模型路径（ini 初始化时记为默认模型路径）。
    QString modelPath() const;

    /// 推荐入口：通过 `wm_create_from_ini` 加载 ONNX 与 Frame/ICP/Undercut 等参数。
    /// @param configPath 空则使用 `defaultConfigPath()`。
    /// @param error 可选；失败时写入状态码与说明。
    bool initializeFromIni(const QString& configPath = QString(), WeldMeasureError* error = nullptr);

    /// 遗留/冒烟入口：仅按 ONNX 模型路径创建上下文（不含完整 ini 帧配置）。
    /// @param modelPath 空则使用 `defaultModelPath()`。
    bool initialize(const QString& modelPath = QString(), WeldMeasureError* error = nullptr);

    /// 销毁 DLL 上下文并清空路径缓存；可重复调用。
    void shutdown();

    /// 正式帧流水线：可选 ROI → 降采样 → ICP → 多截面提取 → 均值与咬边统计。
    /// @param frameIndex1Based 与已加载 ini 中 Frame1..N 对应（从 1 起）。
    /// @param xyz 交错存储的点云 x,y,z，长度 = pointCount * 3。
    /// @param out 成功时写入帧级结果；不可为空。
    bool measureFrame(
        int frameIndex1Based,
        const float* xyz,
        size_t pointCount,
        WeldFrameMeasurement* out,
        WeldMeasureError* error = nullptr);

    /// 遗留接口：对已切好的单截面点云做测量。
    bool measureSection(
        const float* xyz,
        size_t pointCount,
        WeldSectionMeasurement* out,
        WeldMeasureError* error = nullptr);

private:
    struct Impl;
    Impl* m_impl = nullptr;
};

}  // namespace scan_tracking::weld_measure
