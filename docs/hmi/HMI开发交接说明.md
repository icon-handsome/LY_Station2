# HMI 显控 TCP 开发交接说明

**文档版本**: v1.5  
**最后更新**: 2026-06-16  
**适用范围**: 本仓库（IPC_Station2，第二工位专用）— **仅 TCP Server 端**；麒麟 OS Qt 显控为独立 Client 工程。

> **v1.5 变更（第二工位骨架）**：本仓已移除 `TrackingService` 与第一工位坡口算法；`event.inspection.finished` 测量字段精简为 `quality_code`；`cmd.debug_trigger_inspection` / `cmd.set_bevel_recipe` 保留协议占位（返回未实现）；已删除 `[Hmi] allowDebugTriggerInspection`。  
> **v1.4 变更**：主流程改内存缓存；调试检测从内存读取三段点云；位姿拼接落盘至 `output/run_*`。（**第一工位路径已不在本仓**）

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
| `config.ini` `[Hmi]` | `enabled`、`tcpPort`（默认 9900） |

**日志分类**：`hmi.server`、`hmi.session`；带 `[TCPIP]` 前缀的收发摘要（高频 status/心跳不刷屏）。

### 2.1 检测结果推送链路（第二工位）

```text
Trig_Inspection（PLC）或 cmd.debug_trigger_inspection（显控）
  → InspectionHandler（当前 Res=8 占位；实现后填业务）
  → HmiTcpServer::publishInspectionResult
  → TCP event.inspection.finished
```

- 显控 TCP 连接成功后推送**初始占位帧**（`resultCode=0`，`message="等待检测"`）。
- 第二工位检测实现后：在 Handler 或业务层构造 `InspectionResult` 并调用 `publishInspectionResult`。
- `cmd.debug_trigger_inspection`：当前推送 Res=8 占位结果，**不写 PLC**；用于显控 UI 联调。
- `cmd.set_bevel_recipe`：返回失败说明（第一工位坡口配方已移除）。

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
- [x] `status.plc` 辅机字段：`telescopicRodStatus` / `rollerSetFreqHz` / `rollerRunFreqHz` / `electromagnetStatus` / `estopButtonStatus`（两工位均推送；status=2 时 `event.alarm` code 920/921）
- [x] 状态变化即时推送
- [x] `event.scan.*`、`event.bundle.captured`、`event.image.captured`
- [x] `event.alarm`、业务事件（绑定 StateMachine）

### 3.3 控制与调试命令

- [x] `cmd.start` / `cmd.stop` / `cmd.reset` / `cmd.clear_alarm`
- [x] `cmd.get_status` / `cmd.get_config`
- [x] `cmd.modbus_connect` / `cmd.modbus_disconnect`
- [x] `cmd.capture_mech_eye` / `cmd.capture_bundle`
- [x] `cmd.refresh_camera`（当前仅刷新 MechEye）
- [x] `cmd.debug_trigger_inspection`（占位：推送 Res=8 的 `event.inspection.finished`，不写 PLC）
- [x] `cmd.set_bevel_recipe`（占位：返回「第二工位未实现」）

### 3.4 检测测量结构化

- [x] `event.inspection.finished` 含 `resultCode`、`ngReasonWord*`、`measureItemCount`、`quality_code`、`message`
- [x] 连接后初始占位帧 + 检测完成推送（OK/NG/未实现均推送）

### 3.5 配置与设备故障字

- [x] `config.ini [Hmi]`：`enabled`、`tcpPort`
- [x] `status.device.faultWord0` 位表聚合，见协议 §2.4

---

## 4. 刻意未开放 / 待办（Core 侧）

| 项 | 状态 | 说明 |
|----|------|------|
| `cmd.trigger_scan` / `cmd.trigger_inspection` 等 | **拒绝** | 须 PLC→状态机触发（防撞机） |
| `cmd.debug_trigger_inspection` | **占位** | 推送未实现结果；不写 PLC |
| `cmd.set_bevel_recipe` | **占位** | 第一工位配方已移除 |
| `event.task.*` | 未实现 | 协议常量已有 |
| `event.log` | 默认关闭 | `kForwardQtLogsToHmi=false` |
| 无显控连接时补发最后一帧检测结果 | 未实现 | TODO(hmi-demo) |
| Qt 显控 Client + UI | **外部仓库** | 本仓库不维护客户端 |

---

## 5. 关键代码入口

| 场景 | 文件 |
|------|------|
| 服务启动 | `app/src/console_runtime.cpp` |
| 命令分发 / 状态推送 / `publishInspectionResult` | `modules/hmi_server/src/hmi_tcp_server.cpp` |
| 检测 Handler 占位 | `modules/flow_control/src/handlers/inspection_handler.cpp` |
| 帧组包 / 协议常量 | `modules/hmi_server/src/hmi_protocol.cpp`、`hmi_protocol.h` |

---

## 6. 现场联调（Core + 外部 Qt Client）

1. `config.ini`：`[Hmi] enabled=true`、`tcpPort=9900`
2. 启动 `scan-tracking.exe`，确认 HMI TCP 监听日志
3. 由 **Qt 显控** 或任意遵守协议的 TCP 客户端连接并联调（见联调文档）
4. 验收步骤：[`HMI现场联调_阶段0-1.md`](./HMI现场联调_阶段0-1.md)

---

## 7. Qt 显控侧（不在本仓库）

1. 在麒麟 Qt 工程中实现 TCP Client（`QTcpSocket`）
2. 按 `hmi_protocol.h` 实现长度头 + JSON 解帧与 `type` 分发
3. 可引用本仓库 `scan_tracking_hmi_protocol` 的常量与 `serializeFrame` 逻辑（拷贝或子模块，勿在本仓库加 Client 目标）
4. UI 绑定 `status.*` / `event.*`；**不要**调用 `cmd.trigger_*`（Core 会拒绝）
5. 演示检测页可调用 `cmd.debug_trigger_inspection`（当前为占位推送）

---

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| [封头检测工位_TCP_IP显控通信协议_v1.0.md](../protocols/封头检测工位_TCP_IP显控通信协议_v1.0.md) | 消息与 payload |
| [HMI现场联调_阶段0-1.md](./HMI现场联调_阶段0-1.md) | 联调验收 |
| [第二工位骨架删减说明.md](../station2/第二工位骨架删减说明.md) | 本仓 HMI 保留范围 |
