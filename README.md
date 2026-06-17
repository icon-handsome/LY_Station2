# Scan Tracking — 兰铀第二工位 IPC

第二工位多模式检测工控机控制台程序（`stationId=2`）。第一工位封头检测算法与流程已从本仓库移除；相机模块与 Modbus/HMI 框架保留，供第二工位流程开发使用。

## 当前状态（2026-06-16）

- **保留**：Mech-Eye、海康 CXP/Mono/C、Orbbec Gemini、Livox MID-360、Tfmini Plus、Modbus、HMI、Handler 框架
- **已移除**：Po_Kou / LB / LBN / Hole / Thickness 等 `third_party` 算法、`modules/tracking`、第一工位配置与文档
- **已实现**：`Trig_ScanSegment` 按 `scan_paths` 执行真实组合采集，段缓存 + `output/run_*` 落盘
- **已实现**：`Trig_Inspection` 从段缓存校验并写 PLC + 推送 HMI（**占位**：未接入缺陷/编号/3D 算法，`qualityCode=1` 表示缓存校验通过）

## 默认配置

[`config.ini`](config.ini) 默认 profile：

```ini
[Station]
stationId=2
profileIni=config/station_profiles/station2_cylinder_semi.ini
scanPathsConfigPath=config/scan_paths/station2_cylinder_semi.json
defaultWorkMode=MODE_CYLINDER_SEMI
```

切换封头/成品模式：改用 `station2_endcap.ini` 或 `station2_semi_finished.ini`。

## 构建与运行

```powershell
cmd /c tools\scan_tracking_dev.cmd configure-debug
cmd /c tools\scan_tracking_dev.cmd build-debug
cmd /c tools\scan_tracking_dev.cmd run-debug
```

CMake 预设：`win-msvc2019-qtcore-ninja-debug` / `release`

## 文档

| 主题 | 文档 |
|------|------|
| 文档总索引 | [`docs/README.md`](docs/README.md) |
| 第二工位流程与续接 | [`docs/station2/双工位改造进度与续接指南.md`](docs/station2/双工位改造进度与续接指南.md) |
| 扫描路径 JSON | [`docs/station2/扫描路径配置说明.md`](docs/station2/扫描路径配置说明.md) |
| 时序图（2.x） | [`docs/sequence_diagrams/README.md`](docs/sequence_diagrams/README.md) |
| Modbus 协议 | [`docs/protocols/封头检测工位PLC-IPC Modbus通信协议_v0.1.md`](docs/protocols/封头检测工位PLC-IPC%20Modbus通信协议_v0.1.md) |
| HMI 交接 | [`docs/hmi/HMI开发交接说明.md`](docs/hmi/HMI开发交接说明.md) |
