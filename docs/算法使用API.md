# 蓝优算法使用说明 API

本文档面向 `scan-tracking` 项目的 Windows 集成场景，说明 `third_party/lanyou_first_detection` 中各检测类的用途、主要 API、当前项目内的最小接入方式，以及手动测试和常见问题。

## 1. 代码位置与当前集成方式

算法源码位于：
- `third_party/lanyou_first_detection`

当前项目中的测试适配层位于：
- `modules/vision/include/scan_tracking/vision/lanyou_detection_adapter.h`
- `modules/vision/src/lanyou_detection_adapter.cpp`
- `modules/vision/tests/lanyou_detection_adapter_smoke_test.cpp`

当前构建方式说明：
- 蓝优算法以源码方式参与编译。
- 默认编译为单一静态库 `lanyou_first_detection`。
- **正式主流程已接入**：`Trig_Inspection` → `lanyou_first_station_adapter::runFirstStationDetection`（外/内/孔三段点云）；分段 PLY 由 `StateMachine` 落盘后按需加载（§10.1、§11）。
- **仍保留**测试适配层 `lanyou_detection_adapter` 与 smoke test，用于无硬件/极小点云联调。

## 2. 依赖与编译要求

Windows 当前依赖：
- MSVC 2019
- CMake 3.21+
- Ninja
- Qt 5.15.2
- PCL 1.12.1
- OpenNI2 运行时

当前工程里与 PCL 相关的重要配置：
- `PCL_DIR=C:/Program Files/PCL 1.12.1/cmake`
- 运行测试程序时，需要把 Qt、PCL、OpenNI2 的 DLL 所在目录加入 `PATH`

注意：
- 蓝优算法是典型的 PCL/C++ 实现，不是 header-only 库。
- 不能只包含头文件使用，必须同时编译并链接其 `.cpp` 实现。
- 若后续需要封装为 DLL，可以在当前静态库方案稳定后再做。

## 3. 主要结果结构体

### 3.1 `FirstPoseDetectionParams`

定义位置：
- `third_party/lanyou_first_detection/include/utils/Params.h`

用途：
- 保存第一检测位的检测输出结果。
- 由 `FirstOutSurfaceDetection` 和 `FirstInlinerSurfaceDetection` 共同写入。

主要字段：
- `bbox_min_pt` / `bbox_max_pt`
  - 输入点云包围盒最小点、最大点。
- `cylinder_axis`
  - 封头圆柱轴方向向量。
- `cylinder_center`
  - 封头圆柱中心点。
- `inner_diameter`
  - 内径结果。
- `inner_diameter_tol`
  - 内径圆度/波动相关结果。
- `inner_circle_perimeter_tol`
  - 内圆周长相关结果。
- `head_depth_tol`
  - 封头深度相关结果。
- `hole_diameter_tol`
  - 开孔直径相关结果。
- `head_angle_tol`
  - 外表面坡口角结果。
- `blunt_height_tol`
  - 钝边高度结果。
- `straight_slope_tol`
  - 直边斜度结果。
- `straight_height_tol`
  - 直边高度结果。
- `inliner_error_log`
  - 第一检测位内表面流程错误信息。
- `outliner_error_log`
  - 第一检测位外表面流程错误信息。

说明：
- 这个结构体是第一检测位的统一结果载体。
- 同一轮检测中，内外表面算法一般写入同一个结果对象。

**HMI 显控上报（`event.inspection.finished`，阶段 3.1 已实现）**

综合检测成功后，下列字段经 `InspectionResult.measurement` 结构化写入 TCP JSON（见 `docs/封头检测工位_TCP_IP显控通信协议_v1.0.md` §2.6）：

| 协议字段 | 算法来源（`FirstPoseDetectionParams`） |
|----------|----------------------------------------|
| `offsetXmm` / `offsetYmm` / `offsetZmm` | `cylinder_center` |
| `head_angle_tol` | `head_angle_tol` |
| `straight_height_tol` | `straight_height_tol` |
| `straight_slope_tol` | `straight_slope_tol` |
| `inner_diameter` | `inner_diameter` |
| `blunt_height_tol` | `blunt_height_tol` |
| `inner_diameter_tol` | `inner_diameter_tol` |
| `hole_diameter_tol` | `hole_diameter_tol` |
| `head_depth_tol` | `head_depth_tol` |
| `outlinerErrorLog` / `inlinerErrorLog` | 适配层错误日志 |

实现位置：`tracking_service.cpp` → `state_machine.cpp` → `hmi_tcp_server.cpp`。

### 3.2 `SecondPoseDetectionParams`

定义位置：
- `third_party/lanyou_first_detection/include/utils/Params.h`

用途：
- 保存第二检测位的检测输出结果。

主要字段：
- `inner_diameter`
- `pipe_length`
- `inner_circle_perimeter_tol`
- `volume_tol`
- `A_welding_error_edge_tol`
- `A_welding_seam_edge_angle_tol`
- `A_welding_left_height_tol`
- `A_welding_undercut_tol`
- `B1_welding_error_edge_tol`
- `B1_welding_seam_edge_angle_tol`
- `B1_welding_left_height_tol`
- `B1_welding_undercut_tol`
- `B2_welding_error_edge_tol`
- `B2_welding_seam_edge_angle_tol`
- `B2_welding_left_height_tol`
- `B2_welding_undercut_tol`

说明：
- 第二检测位外表面结果主要面向焊缝相关特征。
- 当前项目里还没有把第二检测位接入到 smoke test。

## 4. 全局结果接口

定义位置：
- `third_party/lanyou_first_detection/include/utils/Params.h`

### `FirstPoseDetectionParams& GlobalFirstPoseParams()`

用途：
- 获取全局唯一的第一检测位结果对象。

说明：
- 蓝优第一检测位内部实现依赖这个全局结果对象。
- 不建议在多线程场景下直接并发复用。
- 如果后续业务需要并行跑多个工件，需要进一步做实例化隔离改造。

### `void ResetGlobalFirstPoseParams()`

用途：
- 在新一轮检测开始前清空上一轮第一检测位结果。

建议：
- 每次真正业务调用前先执行一次重置，避免脏数据遗留。

## 5. 第一检测位 API

### 5.1 `FirstOutSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/first/FirstOutSurfaceDetection.h`

用途：
- 第一检测位外表面检测。
- 主要关注圆柱姿态、直边高度、直边斜度、坡口角等。

核心接口：

```cpp
class FirstOutSurfaceDetection {
public:
    FirstOutSurfaceDetection();
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    const FirstPoseDetectionParams& GetParams() const;
    FirstPoseDetectionParams& GetParams();
};
```

调用说明：
- `Detect` 输入为 `pcl::PointCloud<pcl::PointXYZ>::Ptr&`。
- 参数是“可写引用”，因此调用时不要传 `const` 点云指针变量。
- 成功时返回 `true`，结果通过 `GetParams()` 获取。
- 失败时返回 `false`，具体原因一般记录在 `outliner_error_log`。

适用输入：
- 以封头外表面为主的点云。
- 点云质量过差、范围错误、配置文件缺失时，通常会失败。

### 5.2 `FirstInlinerSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/first/FirstInlinerSurfaceDetection.h`

用途：
- 第一检测位内表面检测。
- 主要关注内径、开孔等特征。

核心接口：

```cpp
class FirstInlinerSurfaceDetection {
public:
    FirstInlinerSurfaceDetection();
    bool Detect(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& hole_cloud);
    const FirstPoseDetectionParams& GetParams() const;
    FirstPoseDetectionParams& GetParams();
};
```

调用说明：
- `cloud` 为主体点云。
- `hole_cloud` 为孔区域或局部点云。
- 成功时返回 `true`，失败信息通常可从 `inliner_error_log` 获取。

## 6. 第二检测位 API

### 6.1 `SecondOutSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/second/SecondOutSurfaceDetection.h`

用途：
- 第二检测位外表面检测。
- 主要涉及筒体圆柱拟合、焊缝错边、棱角度、余高、咬边等。

核心接口：

```cpp
class SecondOutSurfaceDetection {
public:
    SecondOutSurfaceDetection();
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool Detect(
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& seam_cloud);
    const SecondPoseDetectionParams& GetParams() const;
    SecondPoseDetectionParams& GetParams();
};
```

调用说明：
- 旧接口可直接传单份点云。
- 新接口可传圆柱点云和焊缝点云两份输入。
- 当前代码注释表明双输入接口里“当前仅使用圆柱点云”，后续使用前建议重新确认算法工程师最新约定。

### 6.2 `SecondInlinerSurfaceDetection`

头文件：
- `third_party/lanyou_first_detection/include/detection/second/SecondInlinerSurfaceDetection.h`

用途：
- 第二检测位内表面检测。

核心接口：

```cpp
class SecondInlinerSurfaceDetection {
public:
    SecondInlinerSurfaceDetection();
    bool Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    const SecondPoseDetectionParams& GetParams() const;
    SecondPoseDetectionParams& GetParams();
};
```

说明：
- 当前头文件公开接口比较简单。
- 真正业务接入前，建议结合对应 `.cpp` 再核对其输入点云语义和输出字段含义。

## 7. 当前项目内的测试适配 API

头文件：
- `modules/vision/include/scan_tracking/vision/lanyou_detection_adapter.h`

用途：
- 把 `scan-tracking` 自己的点云结构转为 PCL 点云。
- 提供一个最小 smoke test 调用入口。

### 7.1 `toPclPointCloud(...)`

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr toPclPointCloud(
    const scan_tracking::mech_eye::PointCloudFrame& frame);
```

作用：
- 将项目内 `PointCloudFrame` 转成 `pcl::PointCloud<pcl::PointXYZ>`。

行为说明：
- 输入点数量为 0 时，返回空点云对象。
- 若 `width * height == 点数`，保留有组织点云布局。
- 否则转成 `height = 1` 的无组织点云。
- 会基于坐标是否为有限值设置 `is_dense`。

### 7.2 `runFirstOutDetectionSmoke(...)`

```cpp
LanyouSmokeResult runFirstOutDetectionSmoke(
    const scan_tracking::mech_eye::PointCloudFrame& frame);
```

返回结构：

```cpp
struct LanyouSmokeResult {
    bool invoked = false;
    bool success = false;
    int inputPointCount = 0;
    QString message;
};
```

作用：
- 作为最小打通测试入口，调用 `FirstOutSurfaceDetection::Detect(...)`。

字段含义：
- `invoked`
  - 是否真的进入了算法调用。
- `success`
  - 算法本次是否返回成功。
- `inputPointCount`
  - 输入点数量。
- `message`
  - 调用结果或异常信息。

说明：
- 这个接口用于验证“接线是否打通”。
- 即使 `success=false`，只要 `invoked=true` 且程序未崩溃，也说明集成链路基本正常。

## 8. 最小调用示例

### 8.1 直接调用第一检测位外表面算法

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "detection/first/FirstOutSurfaceDetection.h"
#include "utils/Params.h"

ResetGlobalFirstPoseParams();

auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
    new pcl::PointCloud<pcl::PointXYZ>());

// TODO: 填充 cloud->points

FirstOutSurfaceDetection detector;
bool ok = detector.Detect(cloud);
const auto& params = detector.GetParams();

if (!ok) {
    // params.outliner_error_log 中通常会有失败原因
}
```

### 8.2 在 `scan-tracking` 中通过适配层调用

```cpp
#include "scan_tracking/vision/lanyou_detection_adapter.h"

scan_tracking::mech_eye::PointCloudFrame frame;
// TODO: 填充 frame.pointsXYZ / width / height

auto result = scan_tracking::vision::lanyou::runFirstOutDetectionSmoke(frame);

if (!result.invoked) {
    // 说明还没真正进入算法，比如输入为空
}

if (!result.success) {
    // 说明算法已调用，但检测本身没有成功
    // result.message 可用于打印日志
}
```

## 9. 手动 smoke test 运行方式

当前测试程序：
- `build/win-msvc2019-qtcore-ninja-debug/modules/vision/scan_tracking_lanyou_smoke_test.exe`

运行前建议先设置：

```powershell
cd D:\work\scan-tracking
$env:PATH="C:\Qt\5.15.2\msvc2019_64\bin;C:\Program Files\OpenNI2\Redist;C:\Program Files\PCL 1.12.1\bin;$env:PATH"
```

运行命令：

```powershell
.\build\win-msvc2019-qtcore-ninja-debug\modules\vision\scan_tracking_lanyou_smoke_test.exe
```

预期现象：
- 可能会看到算法内部的配置读取提示。
- 可能会看到 `PointsModel` 的日志输出。
- 最后一行出现：

```text
Lanyou adapter smoke tests passed
```

注意：
- 当前 smoke test 使用的是极小模拟点云。
- 因此“集成成功”的标准是：程序可运行、进入算法、测试断言通过。
- 并不是要求算法在这份模拟点云上产出有效业务特征。

## 10. 配置文件说明

### 10.1 分段采集内存缓存（`config.ini [PointCloudProcessing]`）

`Trig_ScanSegment` 成功后，后台线程 `processPointCloudFrame` 对点云做深度裁剪 / 离群去除 / MLS 平滑 / 体素降采样，结果写入 `StateMachine::m_segmentCaptureResults`；海康 A/B 帧写入 `m_segmentCaptureBundles`。

| 键 | 说明 |
|----|------|
| `enabled` | 总开关；`false` 时直通原始点云 |
| `depthMinMm` / `depthMaxMm` | 后处理 Z 向 PassThrough（mm） |
| `outlierRemovalEnabled` / `outlierMeanK` / `outlierStddevMul` | 统计离群点去除 |
| `smoothingEnabled` / `mlsSearchRadiusMm` / `mlsPolynomialOrder` | MLS 表面平滑 |
| `downsampleEnabled` / `voxelLeafSizeMm` | 体素均匀降采样 |
| `minPointsAfterProcessing` | 任一步后最少点数，不足则本段失败 |

`[FlowControl] scanCacheDirectory` / `retainSegmentPly` 已废弃（仅 LatencyTest 等调试仍可读该路径落盘）。离线 PLY 工具见 `point_cloud_io`。

当前算法目录下有配置文件：
- `third_party/lanyou_first_detection/config/first_config.cfg`
- `third_party/lanyou_first_detection/config/second_config.cfg`
- `third_party/lanyou_first_detection/config/config.cfg`

当前已知现象：
- 某些算法类构造后会尝试按相对路径读取配置。
- 如果当前工作目录不是算法目录附近，可能出现类似：

```text
cannot open ../config/first_config.cfg
cannot open config/first_config.cfg
```

这不一定导致程序崩溃，但可能导致算法退回默认参数或直接失败。

建议：
- 正式接入业务前，最好把配置路径改成明确可控的绝对路径或由主程序统一传入。
- 不建议长期依赖当前工作目录碰巧正确。

## 11. 当前集成边界

当前已经完成：
- Windows 下源码编译通过。
- PCL 依赖接通。
- 项目内点云结构到 PCL 的转换适配。
- 第一工位外/内表面 + 内孔检测适配（`lanyou_first_station_adapter`）。
- **分段采集磁盘缓存**：`Trig_ScanSegment` 成功后写入 `ScanTracking_CaptureCache/pointcloud/*.ply` 与 `hik_mono/*_hikA/B.pgm`；内存仅保留元数据与路径；`Trig_Inspection` 前按 `[Tracking]` 三个段位从磁盘加载 PLY。
- **正式主流程**：`StateMachine` 点云路径缓存 → `loadSegmentCaptureResultsForInspection` → `TrackingService::inspectSegments` → 蓝友 `runFirstStationDetection`。
- **HMI 推送**：检测完成（含 NG）经 `publishInspectionResult` 发送 `event.inspection.finished`。
- 调试命令 `cmd.debug_trigger_inspection`（`allowDebugTriggerInspection` 配置门控）。

**数据流补充**：
- **分段 LB**：在 `VisionPipeline` 完成 bundle 时使用**内存**海康图；PGM 落盘不参与 LB 重算。
- **分段落盘**：不影响 LBN/LB 时序；仅降低内存并在 `Trig_Inspection` 前加载 3 段 PLY。

当前尚未完成：
- 多路径点云融合与 `detectMultiPath()`（见未完成清单 §2.3.2）。
- 第二/第三工位算法接入 IPC 主流程。
- 真实采集点云现场效果充分验证。
- 第二检测位 smoke test。
- 配置文件路径标准化（蓝友 `first_config.cfg` 相对路径）。
- 多线程/多任务并发隔离设计。

## 12. 常见问题

### 12.1 只包含头文件能不能用？

不能。

原因：
- 该算法不是 header-only。
- 主要实现都在 `.cpp` 中。
- 接口还直接依赖 PCL 类型，因此必须完成源码编译和链接。

### 12.2 以后现场能不能直接改？

可以。

因为目前集成的是完整源码，不是黑盒 DLL，所以以下问题都能现场修改：
- CMake/编译适配
- 配置文件路径
- 日志与异常处理
- 输入输出适配
- 部分算法外围逻辑

但如果涉及核心检测逻辑、阈值含义、几何判定依据，仍建议和原算法工程师确认。

### 12.3 为什么 smoke test 里算法可能返回失败，但测试仍然算通过？

因为 smoke test 的目标是验证：
- 数据类型能转过去
- 函数能调起来
- 工程不会崩
- 返回值和错误信息能收回来

它不是效果测试。

## 13. 后续建议

推荐按下面顺序继续推进：
1. 增加第二检测位的 smoke test。
2. 增加“读取真实点云文件并调用算法”的调试入口。
3. 统一配置文件路径管理。
4. 再考虑把算法挂到正式业务流程。

---

## 14. LBN 位姿检测（Mech-Eye 2D + 组织化点云）

**最后更新**: 2026-05-21

### 14.1 代码位置与集成状态

| 层级 | 路径 | 状态 |
|------|------|------|
| 算法核心 | `third_party/LBN/`（`lbn_pose_core.*`、`FastGeoHash.*`） | ✅ 已加入 CMake 目标 `lbn_pose` |
| IPC 适配 | `modules/vision/.../lbn_pose_detection_adapter.*` | ✅ 已实现 |
| 配置 | `common/.../config_manager` → `[LbnPose]` | ✅ |
| 视觉流水线 | `vision_pipeline_service.cpp` | ✅ `enabled` 时调用 |
| 多路径状态机 | `modules/flow_control/.../state_machine.*` | ❌ 未用 `lbnPoseResult` 更新标定矩阵 |

构建开关（根 `CMakeLists.txt`）：
- `SCAN_TRACKING_ENABLE_LBN_POSE_DETECTION`（默认 ON）
- 与 LB 互斥：LBN ON 时链接 `LbnPose::lbn_pose`，LB 适配器编译为 stub

OpenCV：复用 `third_party/LB/opencv-3.4.3-vc14_vc15`（见 `third_party/LBN/CMakeLists.txt`）。

### 14.2 配置项（`config.ini` → `[LbnPose]`）

| 键 | 含义 | 默认 |
|----|------|------|
| `enabled` | 是否调用 LBN；同时为 true 时 Mech-Eye 使用 `Capture2DAnd3D` | `true` |
| `dataRoot` | 模板等资源根目录 | 见 `config_manager.cpp` |
| `templateFile` | GeoHash 模板文件名（相对 `dataRoot`） | — |
| `minDistance` / `maxDistance` / `cosTolerance` / `minPercent` | 传给 `lbn_pose::Config` | 30 / 650 / 0.015 / 0.5 |
| `cloudSearchRadiusPx` | 2D 圆心 lifted 到 3D 的搜索半径 | 20 |

### 14.3 对外 API（IPC 侧）

```cpp
// modules/vision/include/scan_tracking/vision/lbn_pose_detection_adapter.h
LbnPoseResult runLbnPoseDetection(
    const scan_tracking::mech_eye::CaptureResult& mechEyeResult,
    const scan_tracking::common::LbnPoseConfig& config);
```

`LbnPoseResult`（`vision_types.h`）主要字段：`invoked`、`success`、`message`、`poseMatrix`（4×4 行优先）、`matchedPointCount`。

算法命名空间 `lbn_pose::Estimator::estimate(grayImage, alignedCloud)` 定义见 `third_party/LBN/lbn_pose_core.h`。

### 14.4 调用链（当前）

1. `ConfigManager` 加载 `[LbnPose]` 与 `scan_paths_config.json`。
2. `StateMachine::executeScanSegmentTask`：`needMechEye2D = resolveNeedRotationForSegment(segmentIndex)`。
3. `requestCaptureBundle(segment, taskId, needMechEye2D)`：转动点 → `Capture2DAnd3D` 且后台跑 LBN；非转动点 → 仅 3D。
4. Mech-Eye 成功即可跑 LBN（海康失败时跳过 LB，不阻断 LBN）。
5. `onVisionBundleCaptureFinished` → `applyLbnCalibrationUpdate`：`T0' = Rt × T0`（转动点且 LBN 成功）。

HMI 可选 payload 字段 `needMechEye2D`；未传时按 `scan_paths_config.json` 推断。

### 14.5 Visual Studio 中看不到 `third_party/LBN`？

根目录 `.gitignore` 对 `third_party/*` 默认忽略；**文件夹视图**中放行的目录为：`lanyou_first_detection`、`LBN`、`LB`、`MVS` 及 `CMakeLists.txt`。`eigen-3.4.0` 与 `Mech-Eye SDK-2.5.4` **故意不放行**（体积大、日常不改）。

CMake 中 `SCAN_TRACKING_SHOW_EIGEN_IN_IDE` / `SCAN_TRACKING_SHOW_MECHEYE_SDK_IN_IDE` 默认为 `OFF`，不生成 `scan_tracking_third_party_eigen*` / `scan_tracking_third_party_mecheye*` 等 IDE 浏览目标。若临时需要浏览头文件，可在 CMake 配置里将对应选项设为 `ON` 后重新生成。

若仍看不到其它目录：删除 CMake 缓存并重新配置。

### 14.7 离线测试需要准备的数据

在无相机时，可按下列最小集验证 LBN 与标定逻辑：

| 数据 | 格式要求 | 用途 | 是否必需 |
|------|----------|------|----------|
| GeoHash 模板 | `third_party/LBN/data/template-3D-ALL-Shift-Cut-Cut.txt`（仓库已带） | LBN 建库/匹配 | **必需** |
| Mech-Eye 灰度图 | 8-bit 单通道，与点云同宽高（如 1280×1024） | `estimate(gray, cloud)` | **必需**（测 LBN 时） |
| 组织化点云 | `width×height` 个 `float` XYZ，行优先与纹理对齐；可来自 `.ply` 转置或现场导出 | 2D 圆心 lift 到 3D | **必需**（测 LBN 时） |
| `scan_paths_config.json` | 项目根目录；`pointIndex` 与测试用 `segmentIndex` 一致 | `needRotation` 推断 | 测状态机时必需 |
| `config.ini` `[LbnPose]` | `enabled=true`，`templateFile` 指向上述 txt | 配置 | 推荐 |
| 海康双目图 | Mono8 左右图 | LB 双目 | 测 LB 时需要；**测 LBN 可不要** |

**离线工具 `scan_tracking_lbn_offline_runner`（已实现）**：

可执行文件：`build/win-msvc2019-qtcore-ninja-debug/modules/vision/scan_tracking_lbn_offline_runner.exe`  
运行前将 `C:\Qt\5.15.2\msvc2019_64\bin` 与 `third_party/LB/.../opencv/build/x64/vc15/bin` 加入 `PATH`。

```text
scan_tracking_lbn_offline_runner.exe --group testdata/group1 --texture-from-ply
scan_tracking_lbn_offline_runner.exe --group testdata/group1
scan_tracking_lbn_offline_runner.exe -i texture.jpg -p textured.ply
scan_tracking_lbn_offline_runner.exe -p textured.ply --texture-from-ply
```

- **默认（对齐 `third_party/LBN/main.cpp`）**：PLY 无效点 `quiet_NaN`；纹理优先 `*texture_aligned*`，**必须与点云同宽高**（不缩放）；`lbn_pose_core` 插值/提升与 `main.cpp` 相同。
- **`--allow-resize-texture`（扩展）**：允许 jpg 缩放到点云网格（旧 testdata 缩略图，非 main 默认）。
- **`--texture-from-ply`（扩展）**：从顶点 RGB 生成灰度（main 联机不用此路径）。
- **默认**：PLY 无效顶点存 **quiet_NaN**（与 Mech-Eye SDK / `main.cpp` 一致）；`lbn_pose_core` 会拒绝近零伪 3D 点。
- **`--legacy-zero-nan`（旧行为，仅对比）**：无效点写 `(0,0,0)`，可能产生假 3D，勿用于正式验证。
- 失败时会打印 **2D 圆心** 与 **3D 提升坐标**（最多 32 个），便于区分检测问题与 GeoHash 匹配问题。
- 退出码：`0` 成功；`5` 表示 LBN 匹配失败但调用链正常。

**建议测试步骤（无相机）**：

1. 构建并运行 `scan_tracking_lbn_offline_runner --group testdata/group1`（读 PLY 约 30s）。
2. 若 `success=0` 但日志有 `Count of 2D marker centers`，说明加载与 adapter 正常，失败在 GeoHash 匹配。
3. 有相机后再测 `scan-tracking.exe` + `segmentIndex=1`。

**segmentIndex 与配置对应**（`scan_paths_config.json` 路径 1）：

| segmentIndex | needRotation | Mech-Eye 模式 |
|--------------|--------------|---------------|
| 1, 5, 8 | true | 2D+3D + LBN |
| 2,3,4,6,7 | false | 仅 3D |

### 14.6 后续建议

1. 增加 `lbn_offline_runner`（读 bmp/png + ply/csv 点云调用 adapter）。
2. 全量多路径 §2.2.1 与路径级 T0 重置。
3. 现场校验 OpenCV / Mech-Eye SDK DLL 路径。

---

如后续接口有变化，请优先核对以下文件：
- `third_party/lanyou_first_detection/include/utils/Params.h`
- `third_party/lanyou_first_detection/include/detection/first/*.h`
- `third_party/lanyou_first_detection/include/detection/second/*.h`
- `modules/vision/include/scan_tracking/vision/lanyou_detection_adapter.h`
- `third_party/LBN/lbn_pose_core.h`
- `modules/vision/include/scan_tracking/vision/lbn_pose_detection_adapter.h`
