# 坡口测量算法（Po_Kou_Ce_Liang）集成说明

本文档说明 `third_party/Po_Kou_Ce_Liang` 在 scan-tracking 项目中的用途、API 与主流程接入方式。

## 目录与构建

- 算法源码：`third_party/Po_Kou_Ce_Liang`
- CMake 目标：`po_kou_ce_liang`（别名 `Bevel::Measurement`）
- 开关：`SCAN_TRACKING_ENABLE_BEVEL_MEASUREMENT=ON`
- 依赖：PCL 1.12、OpenCV 3.4.3（复用 `third_party/LB/opencv-3.4.3-vc14_vc15`）

## 主流程

```
PLC Trig_Inspection
  → StateMachine::loadMergedPointCloudForInspection()   // 路径级点云合并
  → TrackingService::inspectPointCloud()
  → bevel_measurement_adapter::runBevelMeasurement()
  → bevel::solveBevelFromRawCloud()
  → HMI event.inspection.finished / PLC NG 寄存器
```

## 适配层 API

头文件：`modules/vision/include/scan_tracking/vision/bevel_measurement_adapter.h`

```cpp
scan_tracking::vision::bevel::BevelInspectionResult runBevelMeasurement(
    const scan_tracking::mech_eye::PointCloudFrame& cloud);
```

输出映射到 `InspectionMeasurement`：

| 字段 | 含义 |
|------|------|
| `head_angle_tol` | 坡口角（deg） |
| `blunt_height_tol` | 钝边长度（mm） |
| `bevel_type` | 坡口类型 |
| `icp_fitness` | ICP 拟合度 |
| `quality_code` | 0=合格 |

## 配置

`config.ini`：

```ini
[Bevel]
configPath=bevel/config.txt
templateDir=bevel/data/templates
angleTolDeg=2.0
lengthTolMm=1.0
defaultBevelType=0
defaultAngleDeg=45.0
defaultLengthMm=1.0
```

### 两种标准坡口型号

| `bevel_type` | 坡口角 | 钝边长度 | 模板 |
|--------------|--------|----------|------|
| 0 | 45° | 1 mm | `type_0_*` |
| 1 | 30° | 6 mm | `type_1_*` |

坡口型号与有孔/无孔**独立**：通过 `has_hole`（bool）区分，**未传默认 `false`（无孔）**。

| `has_hole` | 含义 | Core 行为 |
|------------|------|-----------|
| `false` | 无孔 | 记录为无孔，**不调用**测孔径算法；`headMetrics.hole_opening_mm` 为 0 |
| `true` | 有孔 | 记录为有孔；测孔算法接入后将填充 `hole_opening_mm` |

`cmd.get_config` 的 `bevel.presets` 返回坡口型号表；`bevel.recipe` 含当前生效配方（含 `has_hole`）。

Qt 换型示例（45° 无孔，仅传型号时 `has_hole` 默认为 false）：

```json
{ "type": "cmd.set_bevel_recipe", "payload": { "bevel_type": 0 } }
```

30° 有孔：

```json
{ "type": "cmd.set_bevel_recipe", "payload": { "bevel_type": 1, "has_hole": true } }
```

也可显式下发（会覆盖标准值）：

```json
{ "type": "cmd.set_bevel_recipe", "payload": { "bevel_type": 0, "angle_deg": 45.0, "length": 1.0, "has_hole": false } }
```

环境变量：`SCAN_TRACKING_BEVEL_CONFIG_DIR`（指向含 `config.txt` 与 `data/` 的目录）。

运行时资源由构建后处理复制到 exe 旁 `bevel/` 目录。

## 算法 API（第三方）

```cpp
#include "BevelMeasurement.h"

bevel::BevelMeasurementResult solveBevelFromRawCloud(
    const bevel::CloudT::ConstPtr& rawCloud,
    const std::string& configPath,
    const std::string& templateDir = {});
```

## 测试

- `scan_tracking_bevel_smoke_test`：空云拒绝、PCL 转换
- `scan_tracking_tracking_smoke_test`：`inspectPointCloud` 链路

## 常见问题

1. **配置文件找不到**：确认 exe 旁存在 `bevel/config.txt`，或设置 `SCAN_TRACKING_BEVEL_CONFIG_DIR`。
2. **模板加载失败**：确认 `bevel/data/templates/type_0_*` 已部署。
3. **PCL/OpenCV DLL**：使用与 LB 相同的运行时部署（`scan_tracking_deploy_pcl_runtime` / `scan_tracking_deploy_opencv_runtime`）。
