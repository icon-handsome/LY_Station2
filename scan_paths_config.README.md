# 扫描路径配置文件说明

## 文件名称
`scan_paths_config.json`

## 用途
定义封头检测工位的多路径扫描配置，包括：
- 相机标定矩阵
- 所有扫描路径及其点位配置
- 转盘转动策略
- 算法参数配置
- 数据管理策略

---

## 配置结构

### 1. 标定矩阵 (calibrationMatrix)
存储 Mech-Eye 相机到机械臂基座的 4×4 变换矩阵 T0。

**注意**: 此矩阵需要通过标定程序获取，不可随意修改。

---

### 2. 扫描路径 (scanPaths)
定义多条扫描路径，每条路径包含：

| 字段 | 类型 | 说明 |
|------|------|------|
| pathId | int | 路径唯一标识符 |
| pathName | string | 路径名称（便于识别） |
| description | string | 路径用途说明 |
| enabled | bool | 是否启用此路径 |
| totalPoints | int | 路径包含的点位总数 |
| points | array | 点位配置列表 |

#### 点位配置 (points)

| 字段 | 类型 | 说明 |
|------|------|------|
| pointIndex | int | 点位索引（从 1 开始） |
| pointName | string | 点位名称 |
| needRotation | bool | **关键**: 是否需要转动转盘 |
| rotationAngle | float | 转盘目标角度（度） |
| description | string | 点位说明 |

**needRotation 的影响**:
- `true`: Mech-Eye 采集 3D+2D，调用 LBN 算法更新标定矩阵
- `false`: Mech-Eye 仅采集 3D，沿用上次标定矩阵

---

### 3. 执行策略 (executionConfig)

| 字段 | 类型 | 说明 |
|------|------|------|
| executeAllPaths | bool | 是否执行所有启用的路径 |
| selectedPathIds | array | 指定要执行的路径 ID 列表 |
| pathExecutionOrder | string | 路径执行顺序（sequential/parallel） |
| allowPathSkipOnError | bool | 路径失败时是否跳过继续执行 |

**示例**:
```json
{
  "executeAllPaths": false,
  "selectedPathIds": [1, 2],  // 仅执行路径 1 和 2
  "pathExecutionOrder": "sequential",
  "allowPathSkipOnError": false
}
```

---

### 4. 转盘配置 (turntableConfig)

| 字段 | 类型 | 说明 |
|------|------|------|
| enabled | bool | 是否启用转盘控制 |

**说明**: 
- 转盘的具体转动角度在各点位的 `rotationAngle` 字段中指定
- 转盘控制的速度、加速度等参数由 PLC 或机械臂控制系统管理
- IPC 只需要知道是否需要转动以及目标角度

---

### 5. 采集配置 (captureConfig)

#### Mech-Eye 相机
- `capture3DTimeoutMs`: 3D 点云采集超时（毫秒）
- `capture2DTimeoutMs`: 2D 图像采集超时（毫秒）
- `depthRangeMin/Max`: 深度范围（毫米）

#### 海康相机
- `captureTimeoutMs`: 采集超时（毫秒）
- `exposureTimeUs`: 曝光时间（微秒）
- `gain`: 增益（dB）

---

### 6. 算法配置 (algorithmConfig)

#### LBN 算法（新增）
- `enabled`: 是否启用
- `timeoutMs`: 超时时间
- `confidenceThreshold`: 置信度阈值

#### LB 双目跟踪算法
- `enabled`: 是否启用
- `timeoutMs`: 超时时间
- `minFeaturePoints`: 最小特征点数

#### 蓝友最终解算算法
- `enabled`: 是否启用
- `timeoutMs`: 超时时间
- `qualityThreshold`: 质量阈值

---

### 7. 数据管理 (dataManagement)

| 字段 | 类型 | 说明 |
|------|------|------|
| savePointClouds | bool | 是否保存点云文件 |
| saveRawImages | bool | 是否保存原始图像 |
| saveIntermediateResults | bool | 是否保存中间结果 |
| outputDirectory | string | 输出目录路径 |
| maxStorageDays | int | 最大保存天数 |

---

## 使用场景

### 场景 1: 快速预检（4 个点位）
```json
{
  "executeAllPaths": false,
  "selectedPathIds": [4]  // 仅执行快速预检路径
}
```

### 场景 2: 完整检测（外圈 + 内圈）
```json
{
  "executeAllPaths": false,
  "selectedPathIds": [1, 2]  // 执行外圈和内圈路径
}
```

### 场景 3: 焊缝专项检测
```json
{
  "executeAllPaths": false,
  "selectedPathIds": [3]  // 仅执行焊缝检测路径
}
```

### 场景 4: 全面检测（所有路径）
```json
{
  "executeAllPaths": true,
  "selectedPathIds": []  // 执行所有 enabled=true 的路径
}
```

---

## 配置修改指南

### 添加新路径
1. 在 `scanPaths` 数组中添加新的路径对象
2. 设置唯一的 `pathId`
3. 定义路径的所有点位配置
4. 设置 `enabled: true`
5. 在 `executionConfig.selectedPathIds` 中添加新路径 ID

### 修改点位转动策略
1. 找到对应路径和点位
2. 修改 `needRotation` 标志
3. 设置 `rotationAngle` 目标角度

### 调整算法参数
1. 在 `algorithmConfig` 中找到对应算法
2. 修改超时时间或阈值参数
3. 可通过 `enabled: false` 临时禁用某个算法

---

## 注意事项

⚠️ **重要提示**:

1. **标定矩阵 T0**: 不可随意修改，需通过专业标定程序获取
2. **点位索引**: 必须从 1 开始连续编号
3. **转动角度**: 确保在转盘物理限位范围内（通常 0-360 度）
4. **路径 ID**: 必须唯一，不可重复
5. **配置验证**: 修改后建议先在测试环境验证

---

## 配置文件位置

**推荐位置**: 项目根目录
```
IPC-192.168.110.173_track-main/
├── scan_paths_config.json  ← 配置文件
├── config.ini               ← 系统基础配置
├── app/
├── modules/
└── ...
```

---

## 版本历史

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| 1.0 | 2026-05-19 | 初始版本，定义 4 条示例路径 |

---

## 相关文档

- [多点位扫描与位姿跟踪完整流程.md](./docs/多点位扫描与位姿跟踪完整流程.md)
- [封头检测工位PLC-IPC Modbus通信协议_v0.1.md](./docs/封头检测工位PLC-IPC%20Modbus通信协议_v0.1.md)
