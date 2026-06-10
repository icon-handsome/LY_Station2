# Agent 提示词：阶段 1 — 双工位同代码架构落地

> **用途**：将本文件全文作为 Prompt 交给执行型 AI Agent。  
> **仓库路径**：`docs/station2/Agent提示词-阶段1-双工位架构落地.md`  
> **阶段目标**：只建立「第一工位 + 第二工位共用同一套代码、配置切换工位」的**架构骨架**。  
> **明确不做**：第二工位业务功能、Handler 大重构、新设备模块、PLC 协议变更、删除第一工位逻辑。

---

## 你的角色

你是 C++/Qt 工业软件工程师，在现有 **封头检测工位（第一工位）IPC** 仓库中落地**双工位架构的第一步**。

上级（规划者）会在你完成后做代码审查与验证。**你的任务是最小、可验证、零回归的架构改动**，不是实现第二工位全部功能。

---

## 背景（必读）

- 当前仓库 = 第一工位 IPC，PLC 通过 Modbus 触发 `Trig_*`，IPC 响应。
- 长期目标 = **单仓库、单主线**，`stationId=1` 与 `stationId=2` 靠**配置文件**切换，不靠分支、不靠复制项目。
- 参考文档（只读，勿改）：
  - [`docs/station2/工作流程-AI-Agent参考.md`](./工作流程-AI-Agent参考.md)
  - [`docs/station2/第一工位重构计划.md`](./第一工位重构计划.md)

---

## 本阶段交付物（必须全部完成）

### D1. 新增 `StationProfile` 配置模型与加载逻辑

在 `common` 模块中新增工位配置（**不要**新建独立 CMake 子工程，除非确实需要；优先放在 `common`）：

**新增文件（建议）：**
```
common/include/scan_tracking/common/station_profile.h
common/src/station_profile.cpp   # 若逻辑简单可合并进 config_manager.cpp，但建议独立
```

**定义以下类型（名称可微调，语义不变）：**

```cpp
enum class StationId : int {
    FirstEndCap = 1,    // 第一工位（封头）
    SecondMultiMode = 2 // 第二工位（多模式，本阶段仅骨架）
};

enum class WorkModeId : int {
    Unknown = 0,
    ModeEndCap = 1,           // 第二工位-封头模式（本阶段占位）
    ModeCylinderSemi = 2,     // 第二工位-圆筒半成品（本阶段占位）
    ModeSemiFinished = 3      // 第二工位-半成品/成品（本阶段占位）
};

struct StationProfile {
    StationId stationId = StationId::FirstEndCap;
    QString stationName;              // 如 "第一工位-封头"
    WorkModeId defaultWorkMode = WorkModeId::Unknown;
    QString scanPathsConfigPath;      // 扫描路径 JSON 绝对或相对路径
    // 以下 bool 本阶段只解析+日志，不改变运行时行为（全部为后续预留）
    bool enableLoadGrasp = true;
    bool enableUnloadCalc = true;
    bool enablePoseCheck = true;
    bool enableTelescopicScan = false;
    bool enableHoistAssist = false;
    bool enableCollisionMonitor = false;
};
```

**加载规则：**
1. 从 `config.ini` 的 `[Station]` 段读取（见 D2）。
2. 若 `[Station]` 不存在 → **默认 `stationId=1`**，行为与改前完全一致（向后兼容）。
3. `scanPathsConfigPath` 若为空，走现有 fallback 逻辑（见 D3）。

`ConfigManager` 暴露：
```cpp
const StationProfile& stationProfile() const;
```

启动时打印一行 INFO 日志，例如：
```
[Station] stationId=1 name=第一工位-封头 scanPaths=config/scan_paths/station1_default.json workMode=Unknown
```

---

### D2. 扩展 `config.ini` 与工位 profile 文件

**修改** 根目录 `config.ini`，增加：

```ini
[Station]
; 1=第一工位封头（默认）  2=第二工位多模式
stationId=1
stationName=第一工位-封头
; 相对路径相对于 config.ini 所在目录，或相对于 exe 目录（与现有配置查找策略一致）
scanPathsConfigPath=config/scan_paths/station1_default.json
defaultWorkMode=MODE_END_CAP
profileIni=config/station_profiles/station1_endcap.ini
```

**新增目录与文件：**

```
config/
  station_profiles/
    station1_endcap.ini       # 第一工位 profile（本阶段与 [Station] 冗余可接受，为第二工位铺路）
    station2_endcap.ini       # 第二工位-封头模式骨架（占位）
    station2_cylinder_semi.ini
    station2_semi_finished.ini
  scan_paths/
    station1_default.json     # 从现有 scan_paths_config.json 复制/迁移
    station2_endcap.json      # 占位 JSON（可基于 station1 改 description，enabled 路径可先复制）
    station2_cylinder_semi.json   # 最小合法占位
    station2_semi_finished.json   # 最小合法占位
```

**`station_profiles/*.ini` 最少字段：**
```ini
[Station]
stationId=2
stationName=第二工位-圆筒半成品
scanPathsConfigPath=config/scan_paths/station2_cylinder_semi.json
defaultWorkMode=MODE_CYLINDER_SEMI
enableLoadGrasp=false
enableUnloadCalc=false
enablePoseCheck=false
enableTelescopicScan=true
enableHoistAssist=true
enableCollisionMonitor=true
```

**加载优先级（必须实现并写注释）：**
1. `config.ini` `[Station]` 为基础
2. 若 `profileIni` 非空且文件存在 → **合并覆盖**同名字段（profile 优先级高于 config.ini 的 `[Station]`）
3. 最终得到 `StationProfile`

`defaultWorkMode` 字符串映射：
- `MODE_END_CAP` → `WorkModeId::ModeEndCap`
- `MODE_CYLINDER_SEMI` → `WorkModeId::ModeCylinderSemi`
- `MODE_SEMI_FINISHED` → `WorkModeId::ModeSemiFinished`
- 未知值 → `WorkModeId::Unknown` + WARNING 日志

---

### D3. 扫描路径 JSON 路径解析（保持第一工位零回归）

**修改** `common/src/config_manager.cpp` 中的 `scanPathsConfigPath()` 逻辑：

**新解析顺序：**
1. `ConfigManager::stationProfile().scanPathsConfigPath`（若非空）
2. 旧逻辑：exe 目录下 `scan_paths_config.json`
3. 旧逻辑：项目根目录 `scan_paths_config.json`

**根目录 `scan_paths_config.json`：**
- **保留不删**（向后兼容）
- 在文件头 `_fieldTestNote` 或新增 `_deprecationNote` 注明：推荐改用 `config/scan_paths/station1_default.json`
- `config/scan_paths/station1_default.json` 内容应与当前 `scan_paths_config.json` **一致**（复制即可）

---

### D4. 启动时应用工位配置（最小接入）

**修改** `app/src/console_runtime.cpp`：
- `initModules()` 开头读取 `ConfigManager::stationProfile()` 并打日志（D1 已述）
- **本阶段不要**根据 `enableTelescopicScan` 等开关跳过模块创建（避免行为变化）
- 仅日志输出 profile 中各 `enable*` 字段值，标注 `(reserved, not enforced in stage1)`

**可选（推荐）**：`app/src/main.cpp` 启动 banner 打印 `stationId` + `stationName`。

---

### D5. HMI 状态透出工位信息（最小字段）

**修改** `modules/hmi_server/src/hmi_tcp_server.cpp` 中 `status.plc` 或合适的 status 推送 payload，**新增只读字段**（不破坏现有字段）：

```json
{
  "stationId": 1,
  "stationName": "第一工位-封头",
  "workMode": "MODE_END_CAP"
}
```

若协议文档未定义，在代码注释标明 `// stage1: station metadata extension`。

---

### D6. `StateMachine` 只读访问（不改编排）

**修改** `modules/flow_control/src/state_machine.cpp`：
- 在初始化或首次 `pollPlcState` 时，日志打印当前 `stationId` / `workMode`
- **禁止**本阶段修改任何 `execute*Task()` 业务逻辑
- **禁止**本阶段根据 `stationId` 跳过/禁用触发器

（第二工位触发器启用/禁用留到阶段 2。）

---

### D7. 文档与 README 最小更新

**修改** `README.md`（简短一节即可）说明：
- 双工位配置切换方式
- `config.ini` `[Station]` 字段说明
- `stationId=1` 为默认，与改前行为一致

**不要**新建冗长 markdown；README 增补 ≤ 30 行。

---

## 严格禁止（违反则任务失败）

| 禁止项 | 原因 |
|--------|------|
| 修改 `plc_protocol.h` 寄存器定义 | 本阶段不涉及协议 |
| 修改任何 `execute*Task()` 业务逻辑 | 零回归 |
| 拆分/重构 `state_machine.cpp` 为 Handler | 属于阶段 2 |
| 新增伸缩杆/吊装/碰撞/AI 模块 | 属于后续阶段 |
| 删除根目录 `scan_paths_config.json` | 破坏向后兼容 |
| 删除第一工位算法或 third_party | 非本阶段范围 |
| 修改 `config.ini` 中现有相机/Modbus 等联调参数 | 只增 `[Station]` 段 |
| 执行 `git commit` | **除非用户明确要求**；完成后汇报 diff 即可 |

---

## 代码质量要求

1. **C++17**，风格与仓库现有代码一致（Qt 类型、`QStringLiteral`、日志分类等）。
2. 新增代码要有简短中文或英文注释，说明「阶段 1 占位 / 后续阶段启用」。
3. 路径解析需处理 Windows 路径（仓库在 Windows 开发）。
4. 编译应能通过现有 CMake preset（`win-msvc2019-qtcore-ninja-debug`）；若无法在本机完整链接第三方 SDK，至少保证 **语法正确、CMake 配置无报错**。
5. 若存在可运行的单元测试入口，不要破坏；可为 `StationProfile` 解析加小型测试（可选加分，非必须）。

---

## 验收标准（你必须自检后逐条回复）

完成改动后，在回复中按下列清单汇报 **PASS/FAIL** 及证据：

- [ ] **A1** `stationId` 缺省或未配置 `[Station]` 时，等价于第一工位，`scan_paths_config.json` 仍能被加载
- [ ] **A2** `stationId=1` + `scanPathsConfigPath=config/scan_paths/station1_default.json` 时，扫描路径与改前一致（`scanSegmentTotal=12`、pathId=1 enabled）
- [ ] **A3** `stationId=2` + 任一 `station2_*.ini` 能成功启动（不要求相机/PLC 真连），日志打印正确 profile
- [ ] **A4** `Trig_*` 处理逻辑无 diff（除日志外）；对比 `state_machine.cpp` 中 `execute*Task` 函数体应无业务变更
- [ ] **A5** HMI status 含 `stationId` / `stationName` / `workMode` 新字段
- [ ] **A6** 根目录 `scan_paths_config.json` 仍存在
- [ ] **A7** 列出所有 **新增/修改** 文件路径清单
- [ ] **A8** 若编译过，附编译命令与结果；若未编译，说明原因

---

## 建议执行顺序

```
1. 阅读 config_manager.cpp 的 load() 与 scanPathsConfigPath()
2. 新增 station_profile.h + 解析逻辑
3. 扩展 ConfigManager 加载 [Station] 与 profileIni
4. 创建 config/ 目录与占位文件
5. 复制 scan_paths_config.json → config/scan_paths/station1_default.json
6. 接入 console_runtime / hmi_tcp_server / state_machine 日志
7. 更新 README
8. 自检验收清单
```

---

## 完成后输出格式（Agent 回复模板）

请按此结构回复规划者（用户会转交验证）：

```markdown
## 阶段 1 完成汇报

### 变更摘要
（3-5 句话）

### 文件清单
- 新增：...
- 修改：...

### 验收自检
- A1: PASS/FAIL — ...
- A2: ...
（逐条）

### 如何切换工位（操作说明）
1. 第一工位：config.ini 设置 ...
2. 第二工位：config.ini 设置 ...

### 已知限制 / 下一阶段
- enable* 开关仅解析未生效
- ...

### 编译结果
（命令 + 成功/失败）
```

---

## 给规划者的备注（Agent 可忽略）

本 Prompt 由架构规划生成；用户完成 Agent 落地后会通知规划者做 **代码审查与验证**。Agent 只需专注本文件范围。

---

*Prompt 版本：stage1-v1.0 | 2026-06-09*
