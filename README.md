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
- OpenCV: `third_party\LB\opencv-3.4.3-vc14_vc15`
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
| `[LbPose]` | `leftIntrinsic3x3` 等 | LB 双目标定矩阵与 2D 重建参数 |
| `[LbnPose]` | `useIdentityRtWithoutMarkers` | 转盘无标记联调时 `true`，Rt 用单位阵 |
| `[Tracking]` | `scanSegmentTotal` | PLC 扫描总段数（上限 16，现场可设 10） |
| `[Bevel]` | `configPath` / `templateDir` | Po_Kou 坡口测量配置与模板目录 |
| `[Hmi]` | `tcpPort` / `allowDebugTriggerInspection` | 显控 TCP 与调试检测开关 |

数据流与联调说明见 [`docs/算法使用API.md`](docs/算法使用API.md) §10.1 与 [`docs/多点位扫描与位姿跟踪完整流程.md`](docs/多点位扫描与位姿跟踪完整流程.md)。

## 开发文档

| 主题 | 文档 |
|------|------|
| 未完成项 / 进度 | [`docs/项目未完成事项清单_v1.0.md`](docs/项目未完成事项清单_v1.0.md)（v1.6） |
| 扫描流程与落盘 | [`docs/多点位扫描与位姿跟踪完整流程.md`](docs/多点位扫描与位姿跟踪完整流程.md) |
| PLC 联调前检查 | [`docs/待完善细节清单_PLC联调前.md`](docs/待完善细节清单_PLC联调前.md) |
| 蓝友 / 缓存 API | [`docs/算法使用API.md`](docs/算法使用API.md) |
| Modbus 协议 | [`docs/封头检测工位PLC-IPC Modbus通信协议_v0.1.md`](docs/封头检测工位PLC-IPC%20Modbus通信协议_v0.1.md) |
| LBN 离线调通 | [`docs/LBN离线调通交接说明.md`](docs/LBN离线调通交接说明.md) |

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
| 必读交接 | [`docs/HMI开发交接说明.md`](docs/HMI开发交接说明.md)（模块地图、已实现/待办、代码入口） |
| 现场联调 | [`docs/HMI现场联调_阶段0-1.md`](docs/HMI现场联调_阶段0-1.md)（阶段 0/1/3.1 验收步骤） |
| 协议定义 | [`docs/封头检测工位_TCP_IP显控通信协议_v1.0.md`](docs/封头检测工位_TCP_IP显控通信协议_v1.0.md) |

**本仓库仅含 Core 服务端**：`modules/hmi_server`（`HmiTcpServer`、`HmiSession`、`hmi_protocol`）。  
**显控 TCP 客户端**在麒麟 OS **独立 Qt 工程**中实现，按协议文档对接即可。

## 说明

- `LB` / `LBN` / 蓝友第一工位检测已接入主流程；视觉链路默认使用 CXP 双目替代 GigE 海康 A/B。
- **分段扫描**：`Trig_ScanSegment` 成功后缓存当前路径分段点云与图像；`Trig_Inspection` 从内存缓存按 `[Tracking]` 外/内/孔三段解算（非全部段融合）。
- **位姿拼接落盘**：`Trig_Inspection` 前将最近一次位姿拼接结果写入 `output/run_*/matrix` 与 `output/run_*/pointcloud`。
- 目前程序已可在本机启动。
- 如果未连接真实相机或 Modbus 服务，日志里会出现对应设备未发现或连接失败提示。
