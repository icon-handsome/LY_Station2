# 封头检测工位 IPC-Qt 显控通信协议

**版本**：v1.0（2026-05-25 增补 `cmd.debug_trigger_inspection` 与检测推送说明）  
**适用范围**：第一工位核心控制程序（Windows）与 Qt 显控界面（麒麟 OS）的 TCP/IP 通信。

---

## 1. 通信基础约定

### 1.1 网络架构
- **TCP Server**：核心控制程序（Core）。
- **TCP Client**：Qt 显控程序（Qt）。单客户端连接，不支持多个显控端同时连接。
- **异常处理**：
  - 心跳机制：双方周期发送心跳包（建议 2000ms），超过 6 秒未收到心跳判断为断连。
  - 断连行为：Qt 断连时 Core 不自动暂停，保持继续运行。

### 1.2 报文格式
TCP 是流式协议，为解决粘包和半包问题，采用长度前缀的帧格式：
```text
[4 字节大端长度头] + [JSON 正文 UTF-8]
```
- **长度头**：`uint32_t`，大端网络字节序（Big-Endian），表示后续 JSON 正文的字节数。
- **正文**：不包含大点云和 2D 图像原始数据，仅传输数据摘要。

### 1.3 JSON 基本结构
所有通信的 JSON 正文统一格式如下：
```json
{
  "version": "1.0",
  "msgId": "uuid-或自增序列号",
  "type": "消息类型",
  "timestamp": 1710000000000,
  "payload": {}
}
```
- `version`：协议版本，固定 `"1.0"`。
- `msgId`：请求-响应匹配的唯一标识。
- `type`：消息类型，见后续定义。
- `timestamp`：发送时的 Unix 毫秒时间戳。
- `payload`：实际数据，依 `type` 变化。

### 1.4 消息模型
1. **request**：Qt 主动发起请求或发送控制命令。
2. **response**：Core 对 request 的应答，`msgId` 必须与请求一致。
3. **event**：Core 主动上报状态、结果、报警、日志等，Core 自行生成 `msgId`。

---

## 2. 状态上报与事件 (Event)

由 Core 主动发给 Qt，频率根据字段要求不同。

### 2.1 心跳
- `type`: `heartbeat.ping` (发送方), `heartbeat.pong` (接收方响应)
- `payload`: `{}`

### 2.2 系统运行状态 (`status.system`)
- **频率**：状态变更上报，或 500ms 周期。
- **payload 字段**：
  - `ipcState` (int): 0=未初始化, 1=初始化中, 2=就绪, 3=忙碌, 4=暂停, 5=故障
  - `appState` (string): "Init", "Ready", "Scanning", "Error"
  - `stage` (int): 当前工艺阶段 0~10
  - `alarmLevel` (int): 0=无, 1=提示, 2=黄警, 3=红警
  - `alarmCode`, `warnCode` (int)
  - `ipcReady` (int): 0/1
  - `progress` (int): 0~100

### 2.3 PLC 状态 (`status.plc`)
- **频率**：500ms 周期。
- **payload 字段**：
  - `plcHeartbeat` (int)
  - `plcSystemState` (int): 0=待机, 1=自动, 2=暂停, 3=报警停机, 4=手动
  - `workMode` (int): 0=空闲, 1=上料, 2=扫描, 3=检测, 4=下料
  - `flowEnable` (int): 0/1
  - `safetyWord` (int): 位域字典
  - `taskId` (int), `productType` (int), `recipeId` (int)
  - `scanSegmentIndex` (int), `scanSegmentTotal` (int)
  - `modbusConnected` (bool)

### 2.4 相机与设备状态 (`status.camera` / `status.device`)
- **频率**：
  - Core 每 **500ms** 轮询一次是否需推送，但 **仅当 JSON payload 与上次下发不一致时才真正发送**（变更去重，稳态下不会每 500ms 刷一条）。
  - 相机/流水线/梅卡等 **连接态或 `state` 变化** 时立即尝试推送，同样受 payload 去重约束（海康仅 `connected` 变化才触发 `status.camera` 内容变化，采图过程中的文字状态不会刷屏）。
  - 新客户端接入时 **强制全量** 各推送一次（含 `status.camera`）。
- **显控侧预期**：稳态监视下 `status.camera` 很少出现；连接瞬间可能连收数条（`mechEye`/`hikA`/`hikB`/`hikC`/`pipeline` 分别就绪时）；不应出现无变化的周期性刷屏。
- **camera payload**:
  - `mechEye`: `{ state(int), model, sn, connected(bool) }`
  - `hikA`, `hikB`, `hikC`: `{ roleName, connected(bool) }`（`hikC` 的 `connected`：MVS SDK 已连接 **或** 智能相机 TCP 已接入）
  - `pipeline`: `{ state(int) }`
- **device payload**:
  - `onlineWord0`, `faultWord0` (int) 位域字典（`online` 与 `fault` 按位对齐，便于显控做设备条）

**`status.device` 位定义（word0，低位 Bit0）**

| Bit | `onlineWord0` | `faultWord0` |
|-----|---------------|--------------|
| 0 | IPC Core 进程在线 | IPC 故障（`ipcState=Fault` / `appState=Error` / 红警 `alarmLevel≥3`） |
| 1 | HMI 客户端已连接 | （保留） |
| 2 | Mech-Eye 可用（非 Idle/Error） | Mech-Eye `Error` |
| 3 | 视觉流水线 `Ready` | 视觉流水线 `Error` |
| 4 | 海康 2D 任一台已连接（A/B/C） | 海康 A/B/C 均未连接（服务已配置时） |
| 5 | Tracking 模块已加载 | （保留） |
| 6 | Modbus 已连接 | Modbus 未连接 |
| 7 | （保留） | 黄警 `alarmLevel≥2` |

### 2.5 流程事件 (`event.scan.*` / `event.task.*`)
- `event.scan.started`: `{ segmentIndex, taskId }`
- `event.scan.finished`: `{ segmentIndex, resultCode, imageCount, cloudFrameCount }`
- `event.image.captured`: `{ requestId, cameraKey, pointCount, width, height, elapsedMs, errorCode }` (不传原图)
- `event.bundle.captured`: `{ segmentIndex, taskId, mechOk, hikAOk, hikBOk, lbOk }`

### 2.6 第一工位检测结果 (`event.inspection.finished`)
- **产生时机**：
  - **正式**：PLC `Trig_Inspection` → `TrackingService::inspectSegments`（蓝友）→ `HmiTcpServer::publishInspectionResult`
  - **调试**：显控 `cmd.debug_trigger_inspection`（须 `config.ini [Hmi] allowDebugTriggerInspection=true`）
- **推送策略**：`resultCode=1`（OK）与 `resultCode=2`（NG）**均推送**；显控须处理失败场景。
- **Core 实现状态**：下列测量字段均由 `InspectionMeasurement` 结构化写入 payload（非仅 `message` 文本）；检测失败时数值字段多为 `0`。
- **payload 字段**（算法参数全集）：
  - `resultCode` (int): 1=OK, 2=NG
  - `ngReasonWord0`, `ngReasonWord1` (int)
  - `measureItemCount` (int)
  - `segmentCount`, `totalPointCount` (int): 参与检测的缓存段数 / 总点数（2026-05 增补）
  - `offsetXmm`, `offsetYmm`, `offsetZmm` (float): 圆柱中心坐标
  - `stableOffsetXmm`, `stableOffsetYmm`, `stableOffsetZmm` (float): 稳定后偏移量
  - `head_angle_tol` (float): 坡口角
  - `straight_height_tol` (float): 直边高度
  - `straight_slope_tol` (float): 直边斜度
  - `inner_diameter` (float): 内径
  - `blunt_height_tol` (float): 钝边高度
  - `inner_diameter_tol` (float): 内径圆度
  - `hole_diameter_tol` (float): 开孔直径
  - `head_depth_tol` (float): 封头深度
  - `outlinerErrorLog`, `inlinerErrorLog` (string): 外/内表面错误日志
  - `message` (string): 概要描述

### 2.7 其他检测校验完成 (`event.xxx.finished`)
- `event.pose_check.finished`: `{ success, resultCode, poseDeviationMm, rt[16], message }`
- `event.load_grasp.finished`: `{ resultCode, x, y, z, rx, ry, rz }`
- `event.unload_calc.finished`: `{ resultCode, x, y, z, rx, ry, rz }`
- `event.self_check.finished`: `{ resultCode, failWord0 }`
- `event.code_read.finished`: `{ resultCode, codeValue(string) }`
- `event.result_reset.finished`: `{ resultCode }`

### 2.8 报警与日志
- `event.alarm`: `{ level, code, message, timestamp }` (单向发送，无需回执)
- `event.log`: `{ severity, category, message, file, line, timestamp }`

---

## 3. Qt 控制命令 (Request / Response)

Qt 发送 request（附带不重复的 `msgId`），Core 执行后返回对应 `msgId` 的 response，response `payload` 基础结构包含：`{ "success": true/false, "message": "描述" }`，部分命令会附加额外字段。

| type | request payload | response payload 附加字段 | 说明 |
|---|---|---|---|
| `cmd.start` | `{}` | - | 启动状态机 |
| `cmd.stop` | `{}` | - | 停止状态机 |
| `cmd.reset` | `{}` | - | 重置状态 |
| `cmd.clear_alarm` | `{}` | - | 清除当前报警记录 |
| `cmd.get_status` | `{}` | `system`, `plc`, `camera`, `device` 全量状态对象 | 主动拉取全量状态 |
| `cmd.get_config` | `{ "section": "..." }` | 指定 section 或全量 JSON | 获取 Core 侧配置 |
| `cmd.trigger_scan` | `{ "segmentIndex": 1, "taskId": 123 }` | - | 触发单段扫描 |
| `cmd.trigger_inspection` | `{ "taskId": 123 }` | - | 触发综合检测（**Core 拒绝**，须 PLC） |
| `cmd.debug_trigger_inspection` | `{}` | `resultCode`, `cachedSegmentIndices`, `cachedSegmentCount`, `inspectionMessage`, `multiPathNote` | **调试**：用内存缓存点云跑蓝友并推 `event.inspection.finished`；须 `allowDebugTriggerInspection=true`；不写 PLC |
| `cmd.trigger_self_check` | `{}` | - | 触发自检 (当前为占位) |
| `cmd.trigger_pose_check` | `{}` | - | 触发位姿校验 (当前为占位) |
| `cmd.trigger_code_read` | `{}` | - | 触发条码读取 (当前为占位) |
| `cmd.trigger_result_reset`| `{}` | - | 触发结果缓存清空 |
| `cmd.capture_mech_eye` | `{ "cameraKey":"...", "mode":0, "timeoutMs":5000 }` | `"requestId": 123` | 单相机独立采图 |
| `cmd.capture_bundle` | `{ "segmentIndex":1, "taskId": 123 }` | `"requestId": 123` | 触发多相机集成采集 |
| `cmd.refresh_camera` | `{}` | - | 刷新相机连接状态 |
| `cmd.modbus_connect` | `{}` | - | 重连 PLC Modbus |
| `cmd.modbus_disconnect` | `{}` | - | 断开 PLC Modbus |

> **备注**：不需要支持 `cmd.set_config` （热修改配置），不涉及直接控制 PLC 寄存器的命令（Qt 不直接控制 PLC）。第二、第三工位参数暂不加入协议。  
> **`cmd.trigger_*` 与 `cmd.debug_trigger_inspection` 区别**：前者一律拒绝（防撞机）；后者为配置门控的演示/联调入口，默认关闭。

---

## 4. 示例

### 4.1 触发检测与响应
**Qt 请求：**
```json
{
  "version": "1.0",
  "msgId": "req-101",
  "type": "cmd.trigger_inspection",
  "timestamp": 1710000000000,
  "payload": {
    "taskId": 123
  }
}
```
**Core 响应：**
```json
{
  "version": "1.0",
  "msgId": "req-101",
  "type": "cmd.trigger_inspection",
  "timestamp": 1710000000050,
  "payload": {
    "success": true,
    "message": "Inspection triggered."
  }
}
```

### 4.2 检测完成事件上报
**Core 上报：**
```json
{
  "version": "1.0",
  "msgId": "evt-200",
  "type": "event.inspection.finished",
  "timestamp": 1710000005000,
  "payload": {
    "resultCode": 1,
    "ngReasonWord0": 0,
    "ngReasonWord1": 0,
    "measureItemCount": 4,
    "offsetXmm": 0.123,
    "offsetYmm": -0.456,
    "offsetZmm": 0.789,
    "head_angle_tol": 29.5,
    "straight_height_tol": 25.0,
    "straight_slope_tol": 0.3,
    "inner_diameter": 600.2,
    "blunt_height_tol": 0.0,
    "inner_diameter_tol": 0.0,
    "hole_diameter_tol": 0.0,
    "head_depth_tol": 0.0,
    "message": "第一工位检测通过"
  }
}
```
