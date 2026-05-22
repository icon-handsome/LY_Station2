# HMI 显控 TCP 开发交接说明

**文档版本**: v1.1  
**最后更新**: 2026-05-22  
**适用范围**: 本仓库（IPC Core，Windows）— **仅 TCP Server 端**；麒麟 OS Qt 显控为独立 Client 工程。

> **新接手请先读**：本文 → [`HMI现场联调_阶段0-1.md`](./HMI现场联调_阶段0-1.md) → [`封头检测工位_TCP_IP显控通信协议_v1.0.md`](./封头检测工位_TCP_IP显控通信协议_v1.0.md)

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
                    │ StateMachine │ Modbus │ MechEye │ VisionPipeline │ Hik A/B │
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

### 3.4 检测测量结构化（阶段 3.1）

- [x] `event.inspection.finished` 含 8 项算法测量字段

### 3.5 配置与设备故障字

- [x] `config.ini [Hmi]`：`enabled`、`tcpPort`
- [x] `status.device.faultWord0` 位表聚合，见协议 §2.4

---

## 4. 刻意未开放 / 待办（Core 侧）

| 项 | 状态 | 说明 |
|----|------|------|
| `cmd.trigger_scan` 等 | **拒绝** | 须 PLC→状态机触发 |
| `event.task.*` | 未实现 | 协议常量已有 |
| `event.log` | 默认关闭 | `kForwardQtLogsToHmi=false` |
| Qt 显控 Client + UI | **外部仓库** | 本仓库不维护客户端 |
| HMI 展示 LBN 结果 | 未做 | 见 LBN 交接说明 |

---

## 5. 关键代码入口

| 场景 | 文件 |
|------|------|
| 服务启动 | `app/src/console_runtime.cpp` |
| 命令分发 / 状态推送 / 事件 | `modules/hmi_server/src/hmi_tcp_server.cpp` |
| 帧组包 | `modules/hmi_server/src/hmi_protocol.cpp` |
| 测量字段 | `modules/tracking/src/tracking_service.cpp` |

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
4. UI 绑定 `status.*` / `event.*`；勿暴露未开放的 `cmd.trigger_*`

---

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| [封头检测工位_TCP_IP显控通信协议_v1.0.md](./封头检测工位_TCP_IP显控通信协议_v1.0.md) | 消息与 payload |
| [HMI现场联调_阶段0-1.md](./HMI现场联调_阶段0-1.md) | 联调验收 |
| [算法使用API.md](./算法使用API.md) | 测量字段对照 |
| [项目未完成事项清单_v1.0.md](./项目未完成事项清单_v1.0.md) | §三 HMI |
