# Scan Tracking

封头检测工位 IPC 侧控制台程序。

## 本机环境

- Windows + MSVC
- Qt 5.15.2: `C:\Qt\5.15.2\msvc2019_64`
- MSVC v142 / 19.29 x64: `C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\14.29.30133`
- CMake 3.21+
- Ninja
- Windows SDK
- PCL 1.12.0: `C:\Program Files\PCL 1.12.0`
- Eigen 3.4.0: `third_party\eigen-3.4.0`
- Mech-Eye SDK: `third_party\Mech-Eye SDK-2.5.4`
- 海康 MVS SDK（含 CXP GenTL）: `third_party\MVS`
- OpenCV: `third_party\opencv-3.4.3-vc14_vc15`（LB/LBN/Po_Kou 共用）
- CXP 双目（CH250，PCIe 采集卡）: 主流程 A/B 输入，替代原 GigE 海康 A/B

## 打开方式

1. 用 Visual Studio 打开仓库根目录 `D:\work\LY\IPC-192.168.110.173_track-main`
2. 选择 CMake 预设 `win-msvc2019-qtcore-ninja-debug`
3. 直接运行 `scan-tracking.exe`

项目默认启用严格工具链检查。请固定使用仓库内 CMake 预设：

- `win-msvc2019-qtcore-ninja-debug`
- `win-msvc2019-qtcore-ninja-release`

不要使用 Qt Creator 自动生成的旧 Kit、`out/` 缓存目录，或未带 preset 的手写 CMake 配置；选错 Qt、MSVC 版本或 x86 工具链时，CMake 会在配置阶段直接报错。

## 配置

配置文件位于仓库根目录：

- `config.ini`

启动时程序会优先读取这个文件。

### 常用配置项

| 节 | 键 | 说明 |
|----|-----|------|
| `[Vision]` | `hikCxpEnabled` | 必须为 `true`；主流程 CXP 双目 A/B 开关 |
| `[Vision]` | `hikCxpCameraA/BKey` | CXP 左/右目匹配键（序列号，非 IP） |
| `[Vision]` | `hikCxpExposureTimeUs` / `hikCxpGain` | CXP 曝光/增益 |
| `[Station]` | `stationId` | 工位编号，`1`=第一工位封头（默认），`2`=第二工位多模式骨架 |
| `[Station]` | `stationName` | HMI 与日志展示用工位名称 |
| `[Station]` | `scanPathsConfigPath` | 当前工位扫描路径 JSON；为空时回退根目录 `scan_paths_config.json` |
| `[Station]` | `defaultWorkMode` | 工位默认模式：`MODE_END_CAP` / `MODE_CYLINDER_SEMI` / `MODE_SEMI_FINISHED` |
| `[Station]` | `profileIni` | 可选工位 profile；存在时覆盖 `[Station]` 同名字段 |
| `[LbPose]` | `trackConfigFile` | LB 算法配置入口，指向 `third_party/LB/track_config.ini`（标定/GeoHash/Recon 以该文件为准） |
| `[LbPose]` | `templateFile` | 可选：覆盖 `track_config.ini` 的模板点云路径 |
| `[LbnPose]` | `useIdentityRtWithoutMarkers` | 转盘无标记联调时 `true`，Rt 用单位阵 |
| `[Tracking]` | `scanSegmentTotal` | PLC 扫描总段数（上限 16，现场可设 10） |
| `[Bevel]` | `configPath` / `templateDir` | Po_Kou 坡口测量配置与模板目录 |
| `[Hmi]` | `tcpPort` / `allowDebugTriggerInspection` | 显控 TCP 与调试检测开关 |

数据流与联调说明见 [`docs/station1/算法使用API.md`](docs/station1/算法使用API.md) §10.1 与 [`docs/station1/多点位扫描与位姿跟踪完整流程.md`](docs/station1/多点位扫描与位姿跟踪完整流程.md)。

### 双工位配置切换

本仓库采用同一套代码支持工位切换，入口是 `config.ini` 的 `[Station]` 段。默认 `stationId=1`，缺省或未配置 `[Station]` 时仍按第一工位行为运行，并保留根目录 `scan_paths_config.json` 作为兼容回退。

第一工位使用 `profileIni=config/station_profiles/station1_endcap.ini` 与 `scanPathsConfigPath=config/scan_paths/station1_default.json`。第二工位骨架可切换到 `config/station_profiles/station2_endcap.ini`、`station2_cylinder_semi.ini` 或 `station2_semi_finished.ini`；本阶段仅解析和日志透出 `enable*` 开关，不改变模块创建或 PLC 触发处理。

### FlowControl Handler 架构

PLC 触发任务由 `modules/flow_control/include/scan_tracking/flow_control/handlers` 与 `src/handlers` 下的 Handler 分发，`StateMachine` 保留共享状态、握手和异步回调。`enableLoadGrasp`、`enableUnloadCalc`、`enablePoseCheck` 已生效；在第二工位 profile 下误触发 `Trig_LoadGrasp`、`Trig_UnloadCalc`、`Trig_PoseCheck` 会立即拒绝并写 `Res=8`。`Res=8` 表示当前工位 profile 未启用该触发器。

## 开发文档

| 主题 | 文档 |
|------|------|
| 文档总索引 | [`docs/README.md`](docs/README.md) |
| 未完成项 / 进度 | [`docs/station1/项目未完成事项清单_v1.0.md`](docs/station1/项目未完成事项清单_v1.0.md)（v1.7） |
| 扫描流程与落盘 | [`docs/station1/多点位扫描与位姿跟踪完整流程.md`](docs/station1/多点位扫描与位姿跟踪完整流程.md) |
| 蓝友 / LBN / 缓存 API | [`docs/station1/算法使用API.md`](docs/station1/算法使用API.md) |
| Modbus 协议 | [`docs/protocols/封头检测工位PLC-IPC Modbus通信协议_v0.1.md`](docs/protocols/封头检测工位PLC-IPC%20Modbus通信协议_v0.1.md) |
| 第二工位 / 双工位改造 | [`docs/station2/README.md`](docs/station2/README.md)（进度续接、流程参考、重构计划） |
| 双工位时序图 | [`docs/sequence_diagrams/README.md`](docs/sequence_diagrams/README.md) |

## 构建

推荐用仓库内脚本：

```powershell
cmd /c tools\scan_tracking_dev.cmd configure-debug
cmd /c tools\scan_tracking_dev.cmd build-debug
```

CXP 双目冒烟测试（需采集卡与相机在线）：

```powershell
build\win-msvc2019-qtcore-ninja-debug\modules\vision\scan_tracking_cxp_smoke.exe
```

输出目录见 `config.ini` `[Vision] hikCxpSmokeOutputDir`。

## 运行

```powershell
cmd /c tools\scan_tracking_dev.cmd run-debug
```

## 日志

日志会输出到运行目录下的 `logs` 文件夹。

## HMI / 显控 TCP

IPC 作为 **TCP Server** 与麒麟 OS **Qt 显控**通信（协议 v1.0）。端口在 `config.ini` 的 **`[Hmi] tcpPort`**（默认 **9900**），`enabled=false` 可关闭服务。

| 用途 | 说明 |
|------|------|
| 必读交接 | [`docs/hmi/HMI开发交接说明.md`](docs/hmi/HMI开发交接说明.md)（模块地图、已实现/待办、代码入口） |
| 现场联调 | [`docs/hmi/HMI现场联调_阶段0-1.md`](docs/hmi/HMI现场联调_阶段0-1.md)（阶段 0/1/3.1 验收步骤） |
| 协议定义 | [`docs/protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md`](docs/protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md) |

**本仓库仅含 Core 服务端**：`modules/hmi_server`（`HmiTcpServer`、`HmiSession`、`hmi_protocol`）。  
**显控 TCP 客户端**在麒麟 OS **独立 Qt 工程**中实现，按协议文档对接即可。

## 说明

- `LB` / `LBN` / 蓝友第一工位检测已接入主流程；视觉链路默认使用 CXP 双目替代 GigE 海康 A/B。
- **分段扫描**：`Trig_ScanSegment` 成功后缓存当前路径分段点云与图像；`Trig_Inspection` 从内存缓存按 `[Tracking]` 外/内/孔三段解算（非全部段融合）。
- **位姿拼接落盘**：`Trig_Inspection` 前将最近一次位姿拼接结果写入 `output/run_*/matrix` 与 `output/run_*/pointcloud`。
- 目前程序已可在本机启动。
- 如果未连接真实相机或 Modbus 服务，日志里会出现对应设备未发现或连接失败提示。
