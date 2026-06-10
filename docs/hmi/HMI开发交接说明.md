# HMI 显控 TCP 开发交接说明

**文档版本**: v1.4  
**最后更新**: 2026-06-01  
**适用范围**: 本仓库（IPC Core，Windows）— **仅 TCP Server 端**；麒麟 OS Qt 显控为独立 Client 工程。

> **v1.2 变更**：蓝友检测结果经 `publishInspectionResult` 直推 `event.inspection.finished`（含失败）；新增 `cmd.debug_trigger_inspection` 与 `[Hmi] allowDebugTriggerInspection` 配置开关。  
> **v1.3 变更**：补充分段落盘与调试检测加载 PLY 说明（`ScanTracking_CaptureCache`）。  
> **v1.4 变更**：主流程改内存缓存；调试检测从内存读取三段点云；位姿拼接落盘至 `output/run_*`。

> **新接手请先读**：本文 → [`HMI现场联调_阶段0-1.md`](./HMI现场联调_阶段0-1.md) → [`封头检测工位_TCP_IP显控通信协议_v1.0.md`](../protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md)

---

## 1. 架构一览

```text
┌─────────────────────┐     TCP :9900      ┌──────────────────────┐
│  Qt 显控 (麒麟 OS)   │ ◄──────────────► │  IPC Core (本仓库)    │
│  TCP Client         │  4B BE + JSON    │  HmiTcpServer         │
│  【独立仓库实现】    │                  │  modules/hmi_server  │
└─────────────────────┘                  └──────────┬───────────┘
                                                    │ 信号绑定
                    ┌───────────────────────────────┼───────────────────────────────┐
                    │ StateMachine │ Modbus │ MechEye │ VisionPipeline │ CXP A/B │
                    └───────────────────────────────────────────────────────────────┘
```

- **协议**：v1.0，帧格式 `[uint32 大端长度][UTF-8 JSON]`，信封字段 `version/msgId/type/timestamp/payload`。
- **角色**：Core = TCP Server（单客户端）；显控 = TCP Client。
- **本仓库不包含** 显控客户端代码；Qt 侧按协议自行实现或使用 `serializeFrame` / `buildEnvelope` 语义（可参考 `hmi_protocol.h`）。

---

## 2. 模块与 CMake 目标（仅 Server）

| 路径 / 目标 | 职责 |
|-------------|------|
| `modules/hmi_server/` → `scan_tracking_hmi_protocol` | 协议常量、`buildEnvelope`、`serializeFrame` |
| `modules/hmi_server/` → `scan_tracking_hmi_server` | `HmiTcpServer`、`HmiSession`；监听、推送、命令分发 |
| `app/src/console_runtime.cpp` | 读 `config.ini [Hmi]` 创建 `HmiTcpServer`、`bindServiceSignals()`、`start()` |
| `config.ini` `[Hmi]` | `enabled`、`tcpPort`（默认 9900）、`allowDebugTriggerInspection`（默认 false） |

**日志分类**：`hmi.server`、`hmi.session`；带 `[TCPIP]` 前缀的收发摘要（高频 status/心跳不刷屏）。

### 2.1 蓝友检测结果推送链路（2026-05 演示版）

```text
Trig_Inspection / cmd.debug_trigger_inspection
  → TrackingService::inspectSegments（蓝友 runFirstStationDetection）
  → deliverInspectionResult / HmiTcpServer::publishInspectionResult
  → TCP event.inspection.finished（resultCode=1 或 2 均推送）
```

- 正式 PLC 路径：`console_runtime` 注入 `InspectionResultNotifier`，检测完成即推显控。
- 调试路径：`cmd.debug_trigger_inspection` 走与正式检测相同的蓝友链路；若内存无点云，会先 **`loadSegmentCaptureResultsForInspection`** 从路径级内存缓存加载 `[Tracking]` 三段点云。**不写 PLC**；是否清缓存取决于状态机复位策略。须 `allowDebugTriggerInspection=true`。
- **分段缓存**：`event.scan.finished` 等可带 `cachedSegmentIndices`；位姿拼接产物在 `output/run_*/`（HMI 协议**未**暴露路径，复盘请直接看该目录）。
- **多路径**：可缓存多段，蓝友仍只取 `[Tracking]` 三个段位（外/内/孔）；路径级融合见未完成清单 §2.3.2。

---

## 3. 已实现能力（Core Server）

### 3.1 传输与连接（阶段 0）

- [x] 长度前缀帧、粘包/半包、最大 1MB
- [x] 单客户端；新连接踢旧连接
- [x] `core.hello` / `hmi.hello`
- [x] 心跳：Core 2s `heartbeat.ping`；客户端任意消息重置 6s 超时
- [x] 连接后全量 `status.*` + 周期 500ms 轮询（payload 变更去重，稳态不刷 `status.camera`）

### 3.2 监视面（阶段 1）

- [x] `status.system` / `status.plc` / `status.camera` / `status.device`
- [x] 状态变化即时推送
- [x] `event.scan.*`、`event.bundle.captured`、`event.image.captured`
- [x] `event.alarm`、业务事件（绑定 StateMachine）

### 3.3 控制与调试命令

- [x] `cmd.start` / `cmd.stop` / `cmd.reset` / `cmd.clear_alarm`
- [x] `cmd.get_status` / `cmd.get_config`
- [x] `cmd.modbus_connect` / `cmd.modbus_disconnect`
- [x] `cmd.capture_mech_eye` / `cmd.capture_bundle`
- [x] `cmd.refresh_camera`（当前仅刷新 MechEye）
- [x] `cmd.debug_trigger_inspection`（**须** `allowDebugTriggerInspection=true`；用缓存点云跑蓝友并推 `event.inspection.finished`）

### 3.4 检测测量结构化（阶段 3.1）

- [x] `event.inspection.finished` 含 8 项算法测量字段
- [x] 检测失败（NG）同样推送事件，`outlinerErrorLog` / `inlinerErrorLog` 可用

### 3.5 配置与设备故障字

- [x] `config.ini [Hmi]`：`enabled`、`tcpPort`、`allowDebugTriggerInspection`
- [x] `status.device.faultWord0` 位表聚合，见协议 §2.4

---

## 4. 刻意未开放 / 待办（Core 侧）

| 项 | 状态 | 说明 |
|----|------|------|
| `cmd.trigger_scan` / `cmd.trigger_inspection` 等 | **拒绝** | 须 PLC→状态机触发（防撞机）；**勿与** `cmd.debug_trigger_inspection` 混淆 |
| `cmd.debug_trigger_inspection` | **配置门控** | 默认 `allowDebugTriggerInspection=false`；生产/日常开发保持 false |
| `event.task.*` | 未实现 | 协议常量已有 |
| `event.log` | 默认关闭 | `kForwardQtLogsToHmi=false` |
| 无显控连接时补发最后一帧检测结果 | 未实现 | TODO(hmi-demo) |
| 多路径蓝友融合结果 | 未实现 | 见 §2.3.2、`multiPathNote` 响应字段 |
| Qt 显控 Client + UI | **外部仓库** | 本仓库不维护客户端 |
| HMI 展示 LBN 结果 | 未做 | 见 LBN 交接说明 |

---

## 5. 关键代码入口

| 场景 | 文件 |
|------|------|
| 服务启动 / 检测推送回调注册 | `app/src/console_runtime.cpp` |
| 命令分发 / 状态推送 / `publishInspectionResult` | `modules/hmi_server/src/hmi_tcp_server.cpp` |
| 调试命令 / 缓存点云检测 | `StateMachine::runDebugInspectionOnCachedSegments` |
| 蓝友检测与回调 | `modules/tracking/src/tracking_service.cpp` |
| 帧组包 / 协议常量 | `modules/hmi_server/src/hmi_protocol.cpp`、`hmi_protocol.h` |

---

## 6. 现场联调（Core + 外部 Qt Client）

1. `config.ini`：`[Hmi] enabled=true`、`tcpPort=9900`；演示调试命令时另设 `allowDebugTriggerInspection=true` 并重启 Core
2. 启动 `scan-tracking.exe`，确认 HMI TCP 监听日志
3. 由 **Qt 显控** 或任意遵守协议的 TCP 客户端连接并联调（见联调文档）
4. 验收步骤：[`HMI现场联调_阶段0-1.md`](./HMI现场联调_阶段0-1.md)

---

## 7. Qt 显控侧（不在本仓库）

1. 在麒麟 Qt 工程中实现 TCP Client（`QTcpSocket`）
2. 按 `hmi_protocol.h` 实现长度头 + JSON 解帧与 `type` 分发
3. 可引用本仓库 `scan_tracking_hmi_protocol` 的常量与 `serializeFrame` 逻辑（拷贝或子模块，勿在本仓库加 Client 目标）
4. UI 绑定 `status.*` / `event.*`；**不要**调用 `cmd.trigger_*`（Core 会拒绝）
5. 演示检测页可调用 `cmd.debug_trigger_inspection`（仅当 Core 侧开关为 true）

---

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| [封头检测工位_TCP_IP显控通信协议_v1.0.md](../protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md) | 消息与 payload |
| [HMI现场联调_阶段0-1.md](./HMI现场联调_阶段0-1.md) | 联调验收 |
| [算法使用API.md](../station1/算法使用API.md) | 测量字段对照 |
| [项目未完成事项清单_v1.0.md](../station1/项目未完成事项清单_v1.0.md) | §三 HMI |
