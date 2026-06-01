# 待完善细节清单（PLC 联调前评估）

编制日期：2026-05-15  
**文档修订**：2026-06-01（对齐 CXP 主链路、LB/LBN 分流、位姿拼接落盘）  
代码基线：`feature/cxp-replace-hik-ab`（`df253cb`）

---

## 一、明天 PLC 联调预期成果

### 第一优先级（必须跑通）

1. **心跳通信验证**：IPC_Heartbeat 每秒递增，PLC 能正确读取
2. **设备在线状态**：Device_Online_Word0 正确反映各模块在线状态
3. **四步握手基本流程**：任选一个 Trig（建议 Trig_SelfCheck=25），验证 Trig→Ack=1→Res→Ack=2→Trig=0→Ack=0 完整闭环
4. **Modbus 读写地址对齐**：确认 PLC 侧地址偏移（0基 vs 40001基）、CDAB 浮点字序一致
5. **报警清除**：验证 Cmd_ClearAlarm 能正确清除 IPC 侧报警状态

### 第二优先级（尽量跑通）

6. **Trig_ScanSegment 单段扫描**：PLC 写 ScanSegmentIndex + Trig=1，IPC 响应并返回 Res=1；段总数与 `[Tracking] scanSegmentTotal` 一致（现场可为 10，上限 16）
7. **Trig_Inspection 综合检测**：外/内/孔三段点云已在内存缓存后触发；IPC 从路径级内存缓存读取后解算
8. **落盘验收**：`Trig_Inspection` 前检查 `output/run_*/matrix` 与 `output/run_*/pointcloud` 是否生成
9. **Trig_LoadGrasp 上料抓取**：验证握手流程和坐标回写格式（当前用模拟值）
10. **Trig_UnloadCalc 下料计算**：同上

### 第三优先级（记录问题，后续解决）

11. 各触发任务的超时行为
12. 异常中断后的 Trig_ResultReset 清理流程
13. Safety_Status_Word 的联锁逻辑

---

## 二、各模块完成度与待完善项

### 2.1 状态机/流程控制（完成度 ~85%）

| 序号 | 待完善项 | 当前状态 | 优先级 |
|------|----------|----------|--------|
| 1 | `executeLoadGraspTask()` 上料抓取点解算 | 使用环境变量/硬编码回退值，未接入真实 3D 视觉定位 | 高 |
| 2 | `executeUnloadCalcTask()` 下料坐标计算 | 同上，使用模拟位姿，未接入满料判定 | 高 |
| 3 | `executeCodeReadTask()` OCR/条码读取 | 完全 placeholder，直接返回 Res=9 | 中 |
| 4 | `executeStationMaterialCheckTask()` 有料判断 | 仅检查服务可用性，未执行真实视觉检测 | 中 |
| 5 | 位姿源 `parsePoseSource` 从环境变量读取 | 联调时需改为从真实视觉算法获取 | 高 |
| 6 | `writeLoadGraspResult()` 坐标回写 | 需确认 CDAB 浮点格式与 PLC 侧一致 | 高 |

### 2.2 Modbus 通信（完成度 ~95%）

| 序号 | 待完善项 | 当前状态 | 优先级 |
|------|----------|----------|--------|
| 1 | 寄存器地址与 PLC 实际地址对齐 | 代码用 0 基偏移，需确认 PLC 侧是否一致 | 高（联调首要） |
| 2 | CDAB 浮点字序验证 | 代码已实现，需与 PLC 实际读取验证 | 高 |
| 3 | config.ini 中 host/port/unitId 改为现场 PLC 地址 | 当前 127.0.0.1:502 unitId=3 | 联调前改 |

### 2.3 视觉流水线（完成度 ~75%）

| 序号 | 待完善项 | 当前状态 | 优先级 |
|------|----------|----------|--------|
| 1 | CXP 双目位姿矩阵现场标定替换 | 当前已接入 CXP 双目链路，需用现场标定矩阵替换联调默认值 | 高 |
| 2 | VisionPipeline 中 LB 位姿检测 | LBN+LB 可同时启用（`SCAN_TRACKING_ENABLE_LB_POSE_DETECTION=ON`）；封头段 CXP 双目走 LB，转盘段走 LBN | 中 |
| 2b | LBN 位姿检测（Mech-Eye） | 适配层已接入；**单段**转动点 `T0'=Rt×T0` 已实现；**多路径级**标定重置与循环仍待 §2.2.1 | 中 |
| 3 | 海康相机 C（智能相机）SDK 采图 | 不支持 GigE 数据流输出，只能走 TCP+FTP | 已确认限制 |
| 4 | MechEye 3D 相机真实点云质量验证 | 框架已通；点云在内存缓存中经 `[PointCloudProcessing]` 后处理，需现场验证点云完整性 | 中 |
| 5 | 多相机同步时间戳对齐 | 未实现 | 低 |

**分段采集与落盘（当前实现）**：
- 综合检测输入：按当前路径从内存缓存读取 `[Tracking]` 外/内/孔三段点云。
- 位姿拼接产物：`Trig_Inspection` 前落盘至 `output/run_*/matrix` 与 `output/run_*/pointcloud`。
- Mech-Eye worker 不再写 `Mech-Pictures/`；段缓存按路径/段号维护。

**分段扫描数据流（LB 与落盘）**：

```text
Trig_ScanSegment
  → VisionPipeline（LB 用当次内存 CXP 图；转盘段仅 LBN，封头段走 LB）
  → 写入路径分段内存缓存（按 pathId/segmentIndex）
Trig_Inspection
  → loadSegmentCaptureResultsForInspection（同一路径 3 个 Tracking 段号）
  → persistLastPoseStitchArtifactToDisk（run_* 下矩阵+点云）
  → 蓝友 runFirstStationDetection
```

### 2.4 跟踪服务/算法（完成度 ~80%）

| 序号 | 待完善项 | 当前状态 | 优先级 |
|------|----------|----------|--------|
| 1 | LB 位姿检测现场参数管理 | LB 标定矩阵与 2D 重建参数已迁移至 `config.ini [LbPose]`，仍需现场标定复核 | 中 |
| 2 | LB 图像来源（分两条链路） | **分段扫描**：`VisionPipeline` 内用当次采集的内存 CXP 图跑 LB，**已对接**。**Trig_PoseCheck**：`TrackingService` 仍可能从**文件**读图（与分段采集无关） | 中（PoseCheck 标定参数仍待完善） |
| 3 | 第二检测位算法（SecondOut/SecondInliner）| 完全未接入 | 中 |
| 4 | 蓝优算法配置文件路径 | 依赖当前工作目录，需改为绝对路径 | 中 |
| 5 | 算法多线程并发隔离 | 使用全局变量 `GlobalFirstPoseParams()`，不支持并发 | 低 |

### 2.5 海康智能相机 C（完成度 ~75%）

| 序号 | 待完善项 | 当前状态 | 优先级 |
|------|----------|----------|--------|
| 1 | SDK 参数读写 | 已实现，SCMVS 运行时不可用 | 已确认限制 |
| 2 | TCP 触发拍照 + FTP 接收图像 | 已实现并验证 | 完成 |
| 3 | SDK TriggerSoftware 不输出数据流 | 已确认，SC 系列智能相机限制 | 已确认限制 |
| 4 | 图像后处理（缺陷识别/OCR）| 未实现 | 后续 |

---

## 三、算法调用关系梳理

### 3.1 哪些功能调用哪些算法？

| 业务触发 | 调用的算法 | 输入 | 输出 | 当前状态 |
|----------|-----------|------|------|----------|
| Trig_ScanSegment | MechEye 3D 采集 + CXP 双目采图 + LB/LBN | 段号 | 内存缓存点云/帧；检测前位姿拼接落盘 `output/run_*` | ✅ 框架已通 |
| Trig_Inspection | 蓝优 FirstOutSurfaceDetection + FirstInlinerSurfaceDetection | 外表面/内表面/内孔三段点云 | 圆柱中心、内径、坡口角、直边高度等 | ✅ 已接入 |
| Trig_PoseCheck | LB 位姿检测（TR_Mark_3D_Recon + FastGeoHash） | 左右相机图像 | 4x4 RT 矩阵、位姿偏差 | ✅ 已接入（从文件读图） |
| Trig_LoadGrasp | 未接入真实算法 | 应为 3D 点云 | 6 轴坐标 (X,Y,Z,Rx,Ry,Rz) | ❌ 占位 |
| Trig_UnloadCalc | 未接入真实算法 | 应为俯视相机图像 | 6 轴坐标 + 满料判定 | ❌ 占位 |
| Trig_CodeRead | 未接入 | 应为 2D 图像 | ASCII 编号字符串 | ❌ 占位 |
| Trig_StationMaterialCheck | 未接入真实算法 | 应为 2D/3D 图像 | 有料/无料 | ❌ 占位 |

### 3.2 各算法输出的具体值

**蓝优第一工位检测（Trig_Inspection 后调用）：**
- `cylinder_center` (x,y,z) — 封头圆柱中心坐标
- `cylinder_axis` — 圆柱轴方向向量
- `inner_diameter` — 内径（mm）
- `head_angle_tol` — 坡口角
- `straight_height_tol` — 直边高度
- `straight_slope_tol` — 直边斜度
- `blunt_height_tol` — 钝边高度
- `hole_diameter_tol` — 开孔直径
- `outliner_error_log` / `inliner_error_log` — 错误日志

**HMI TCP（已实现）**：上述测量项（除 `cylinder_axis`）及偏移量、日志，在检测完成时经 `event.inspection.finished` 结构化上报显控，见 `docs/HMI现场联调_阶段0-1.md` §7。

**LB 位姿检测（Trig_PoseCheck 后调用）：**
- `rt[16]` — 4x4 位姿变换矩阵
- `poseDeviationMm` — 位姿偏差（mm）
- `inputPointCount` — 输入点云点数

**上料抓取点（Trig_LoadGrasp，当前占位）：**
- 应输出：Load_X, Load_Y, Load_Z, Load_Rx, Load_Ry, Load_Rz（6 轴坐标）
- 当前：从环境变量 `SCAN_TRACKING_LOAD_GRASP_POSE` 读取，或使用硬编码回退值

**下料坐标（Trig_UnloadCalc，当前占位）：**
- 应输出：Unload_X~Unload_Rz（6 轴坐标）+ 满料状态
- 当前：同上，使用模拟值

---

## 四、联调前必须确认的配置项

```ini
[Modbus]
host=<PLC 实际 IP>        ; 当前 127.0.0.1，联调前改
port=502                   ; 确认 PLC 端口
unitId=3                   ; 确认从站 ID
```

---

## 五、联调建议顺序（与协议文档第 12 章一致）

1. 心跳 + 就绪 + 报警清除
2. Trig_SelfCheck（最简单的握手验证）
3. Trig_LoadGrasp（验证坐标回写格式）
4. Trig_ScanSegment（验证分段扫描流程）
5. Trig_Inspection（验证综合检测结果回写）
6. Trig_UnloadCalc（验证下料坐标回写）
7. Trig_ResultReset（验证清理复位）

---

## 六、已知限制与风险

1. **海康智能相机 C 不支持 GigE 数据流采图**：只能通过 TCP `start\r\n` 触发 + FTP 接收图像
2. **蓝优算法全局变量**：`GlobalFirstPoseParams()` 不支持并发，同一时间只能处理一个工件
3. **LB 位姿检测从文件读图**：`Trig_PoseCheck` 仍从文件读图；分段扫描已用 CXP 实时图
4. **无数据库**：所有数据在内存中，程序重启后丢失
5. **第二检测位未接入**：如果工艺需要焊缝检测，需要额外开发
