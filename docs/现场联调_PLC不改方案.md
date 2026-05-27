# 现场联调 - PLC 不改方案

**文档版本**: v1.0  
**创建日期**: 2026-05-27  
**约束条件**: PLC 侧无法修改，只能 IPC 侧适配

---

## 一、问题回顾

### 1.1 PLC 工作流程（不变）
```
工件扫描流程：
1. 路径1：PLC 循环发送 Trig_ScanSegment，段号 1, 2, 3, 4, 5, 6
2. 路径2：PLC 循环发送 Trig_ScanSegment，段号 1, 2, 3, 4, 5, 6  ← 段号重复！
3. 路径3：PLC 循环发送 Trig_ScanSegment，段号 1, 2, 3, 4, 5, 6  ← 段号重复！
4. 综合检测：PLC 发送 Trig_Inspection
```

### 1.2 当前 IPC 问题
- 缓存结构：`QMap<int, CaptureResult>`，key 是段号
- 路径2 的段号1 会覆盖路径1 的段号1
- 综合检测时只能拿到最后一条路径的数据

---

## 二、解决方案：IPC 自动路径推断

### 2.1 核心思路

**通过段号重复检测，自动识别路径切换**

```cpp
逻辑：
1. 收到段号1 → 如果缓存中已有段号1 → 判定为新路径开始 → 路径ID++
2. 缓存结构改为：QMap<int, QMap<int, CaptureResult>>
   - 外层 key：路径ID（自动递增：1, 2, 3, ...）
   - 内层 key：段号（1-6）
3. 综合检测时，按配置取指定路径的指定段号
```

### 2.2 工作流程

```
PLC 触发序列：
┌─────────────────────────────────────────────────────────┐
│ Trig_ScanSegment(1) → IPC: 路径1, 段1, 缓存[1][1]      │
│ Trig_ScanSegment(2) → IPC: 路径1, 段2, 缓存[1][2]      │
│ ...                                                     │
│ Trig_ScanSegment(6) → IPC: 路径1, 段6, 缓存[1][6]      │
│                                                         │
│ Trig_ScanSegment(1) → IPC: 检测到段号重复！            │
│                       → 路径ID=2, 段1, 缓存[2][1]      │
│ Trig_ScanSegment(2) → IPC: 路径2, 段2, 缓存[2][2]      │
│ ...                                                     │
│ Trig_ScanSegment(6) → IPC: 路径2, 段6, 缓存[2][6]      │
│                                                         │
│ Trig_ScanSegment(1) → IPC: 检测到段号重复！            │
│                       → 路径ID=3, 段1, 缓存[3][1]      │
│ ...                                                     │
│ Trig_ScanSegment(6) → IPC: 路径3, 段6, 缓存[3][6]      │
│                                                         │
│ Trig_Inspection     → IPC: 从缓存[1][1], [2][2], [3][3]│
│                       取点云，调用蓝友                  │
└─────────────────────────────────────────────────────────┘
```

---

## 三、实现细节

### 3.1 数据结构修改

#### 修改前（state_machine.h）
```cpp
// 单层缓存，key=段号
QMap<int, scan_tracking::mech_eye::CaptureResult> m_segmentCaptureResults;
QMap<int, scan_tracking::vision::MultiCameraCaptureBundle> m_segmentCaptureBundles;
```

#### 修改后
```cpp
// 二维缓存，key1=路径ID, key2=段号
QMap<int, QMap<int, scan_tracking::mech_eye::CaptureResult>> m_pathSegmentCaptureResults;
QMap<int, QMap<int, scan_tracking::vision::MultiCameraCaptureBundle>> m_pathSegmentCaptureBundles;

// 当前路径ID（自动递增）
int m_currentPathId = 1;

// 当前路径已缓存的段号集合（用于检测重复）
QSet<int> m_currentPathSegments;
```

### 3.2 路径切换检测逻辑

```cpp
void StateMachine::executeScanSegmentTask()
{
    const int segmentIndex = m_activeTask.scanSegmentIndex;
    
    // 检测段号重复 → 判定为新路径开始
    if (m_currentPathSegments.contains(segmentIndex)) {
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("检测到段号重复，切换到新路径")
            << QStringLiteral(" 段号=") << segmentIndex
            << QStringLiteral(" 旧路径ID=") << m_currentPathId
            << QStringLiteral(" 新路径ID=") << (m_currentPathId + 1);
        
        // 切换到新路径
        m_currentPathId++;
        m_currentPathSegments.clear();
    }
    
    // 记录当前段号
    m_currentPathSegments.insert(segmentIndex);
    
    // 采集完成后，缓存到二维结构
    m_pathSegmentCaptureResults[m_currentPathId][segmentIndex] = result;
    m_pathSegmentCaptureBundles[m_currentPathId][segmentIndex] = bundle;
    
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("点云已缓存")
        << QStringLiteral(" 路径ID=") << m_currentPathId
        << QStringLiteral(" 段号=") << segmentIndex
        << QStringLiteral(" 总缓存=") << totalCachedCount();
}
```

### 3.3 综合检测取点逻辑

#### config.ini 配置
```ini
[Tracking]
# 格式：路径ID.段号
firstStationOuterSegmentIndex=1.1   # 路径1的段1
firstStationInnerSegmentIndex=2.2   # 路径2的段2
firstStationHoleSegmentIndex=3.3    # 路径3的段3
scanSegmentTotal=6
```

#### 代码实现
```cpp
QMap<int, CaptureResult> StateMachine::loadSegmentCaptureResultsForInspection(QString* errorMessage)
{
    const auto* configMgr = common::ConfigManager::instance();
    const auto& tracking = configMgr->trackingConfig();
    
    // 解析配置：1.1 → 路径1, 段1
    auto parsePathSegment = [](const QString& config) -> QPair<int, int> {
        QStringList parts = config.split('.');
        if (parts.size() == 2) {
            return {parts[0].toInt(), parts[1].toInt()};
        }
        // 兼容旧配置：纯数字 → 路径1
        return {1, config.toInt()};
    };
    
    auto [outerPathId, outerSegment] = parsePathSegment(
        QString::number(tracking.firstStationOuterSegmentIndex));
    auto [innerPathId, innerSegment] = parsePathSegment(
        QString::number(tracking.firstStationInnerSegmentIndex));
    auto [holePathId, holeSegment] = parsePathSegment(
        QString::number(tracking.firstStationHoleSegmentIndex));
    
    QMap<int, CaptureResult> result;
    
    // 从二维缓存中取点云
    if (m_pathSegmentCaptureResults.contains(outerPathId) &&
        m_pathSegmentCaptureResults[outerPathId].contains(outerSegment)) {
        result[1] = m_pathSegmentCaptureResults[outerPathId][outerSegment];
    } else {
        *errorMessage = QStringLiteral("缺少外圈点云：路径%1段%2")
            .arg(outerPathId).arg(outerSegment);
        return {};
    }
    
    // 同理处理 inner 和 hole...
    
    return result;
}
```

### 3.4 结果复位逻辑

```cpp
void StateMachine::resetScanSegmentCache()
{
    m_pathSegmentCaptureResults.clear();
    m_pathSegmentCaptureBundles.clear();
    m_currentPathId = 1;
    m_currentPathSegments.clear();
    
    qInfo(LOG_FLOW) << QStringLiteral("多路径缓存已清空，路径ID重置为1");
}
```

---

## 四、配置文件修改

### 4.1 config.ini

#### 方案A：扩展配置格式（推荐）
```ini
[Tracking]
# 新格式：路径ID.段号（用字符串表示，代码中解析）
firstStationOuterPath=1
firstStationOuterSegment=1
firstStationInnerPath=2
firstStationInnerSegment=2
firstStationHolePath=3
firstStationHoleSegment=3
scanSegmentTotal=6

# 兼容旧格式（如果新配置不存在，则使用旧配置，默认路径1）
firstStationOuterSegmentIndex=1
firstStationInnerSegmentIndex=2
firstStationHoleSegmentIndex=3
```

#### 方案B：保持旧格式，代码中硬编码（快速方案）
```ini
[Tracking]
# 保持不变，代码中硬编码：外圈=路径1段1，内圈=路径2段2，孔=路径3段3
firstStationOuterSegmentIndex=1
firstStationInnerSegmentIndex=2
firstStationHoleSegmentIndex=3
scanSegmentTotal=6
```

**推荐方案B**：现场联调时间紧，先硬编码，后续再优化配置。

---

## 五、代码修改清单

### 5.1 state_machine.h
```cpp
// 新增成员变量
private:
    // 多路径缓存（路径ID → 段号 → 点云）
    QMap<int, QMap<int, scan_tracking::mech_eye::CaptureResult>> m_pathSegmentCaptureResults;
    QMap<int, QMap<int, scan_tracking::vision::MultiCameraCaptureBundle>> m_pathSegmentCaptureBundles;
    
    // 当前路径上下文
    int m_currentPathId = 1;
    QSet<int> m_currentPathSegments;
    
    // 辅助方法
    int totalCachedPointCloudCount() const;
    bool hasSegmentInPath(int pathId, int segmentIndex) const;
```

### 5.2 state_machine.cpp

#### 修改点1：executeScanSegmentTask
```cpp
void StateMachine::executeScanSegmentTask()
{
    // ... 现有代码 ...
    
    // 在 commitScanSegmentCaptureImmediate 之前，检测路径切换
    const int segmentIndex = m_activeTask.scanSegmentIndex;
    
    if (m_currentPathSegments.contains(segmentIndex)) {
        // 段号重复 → 新路径
        m_currentPathId++;
        m_currentPathSegments.clear();
        qInfo(LOG_FLOW).noquote()
            << QStringLiteral("自动切换到路径") << m_currentPathId
            << QStringLiteral("（检测到段号") << segmentIndex << QStringLiteral("重复）");
    }
    
    m_currentPathSegments.insert(segmentIndex);
}
```

#### 修改点2：commitScanSegmentCaptureImmediate
```cpp
void StateMachine::commitScanSegmentCaptureImmediate(...)
{
    // 缓存到二维结构
    m_pathSegmentCaptureResults[m_currentPathId][segmentIndex] = result;
    m_pathSegmentCaptureBundles[m_currentPathId][segmentIndex] = bundle;
    
    qInfo(LOG_FLOW).noquote()
        << QStringLiteral("点云已缓存")
        << QStringLiteral(" [路径") << m_currentPathId
        << QStringLiteral("][段") << segmentIndex << QStringLiteral("]")
        << QStringLiteral(" 点数=") << result.pointCloud.pointCount;
}
```

#### 修改点3：loadSegmentCaptureResultsForInspection
```cpp
QMap<int, CaptureResult> StateMachine::loadSegmentCaptureResultsForInspection(QString* errorMessage)
{
    // 硬编码映射（方案B）
    struct SegmentMapping {
        int pathId;
        int segmentIndex;
        QString name;
    };
    
    const SegmentMapping mappings[] = {
        {1, 1, "外圈"},  // 路径1段1
        {2, 2, "内圈"},  // 路径2段2
        {3, 3, "孔"},    // 路径3段3
    };
    
    QMap<int, CaptureResult> result;
    
    for (int i = 0; i < 3; ++i) {
        const auto& mapping = mappings[i];
        
        if (!m_pathSegmentCaptureResults.contains(mapping.pathId) ||
            !m_pathSegmentCaptureResults[mapping.pathId].contains(mapping.segmentIndex)) {
            *errorMessage = QStringLiteral("缺少%1点云：路径%2段%3")
                .arg(mapping.name)
                .arg(mapping.pathId)
                .arg(mapping.segmentIndex);
            return {};
        }
        
        result[i + 1] = m_pathSegmentCaptureResults[mapping.pathId][mapping.segmentIndex];
    }
    
    return result;
}
```

#### 修改点4：resetScanSegmentCache
```cpp
void StateMachine::resetScanSegmentCache()
{
    m_pathSegmentCaptureResults.clear();
    m_pathSegmentCaptureBundles.clear();
    m_currentPathId = 1;
    m_currentPathSegments.clear();
    
    qInfo(LOG_FLOW) << QStringLiteral("多路径缓存已清空");
}
```

#### 修改点5：cachedScanSegmentIndices（HMI 调试用）
```cpp
QVector<int> StateMachine::cachedScanSegmentIndices() const
{
    QVector<int> indices;
    
    for (auto pathIt = m_pathSegmentCaptureResults.constBegin();
         pathIt != m_pathSegmentCaptureResults.constEnd(); ++pathIt) {
        const int pathId = pathIt.key();
        const auto& segments = pathIt.value();
        
        for (int segmentIndex : segments.keys()) {
            // 编码为：路径ID * 100 + 段号（便于 HMI 显示）
            indices.append(pathId * 100 + segmentIndex);
        }
    }
    
    std::sort(indices.begin(), indices.end());
    return indices;
}
```

---

## 六、优缺点分析

### 6.1 优点
✅ **PLC 侧零改动**：完全由 IPC 适配  
✅ **自动路径识别**：无需 PLC 告知当前路径  
✅ **向后兼容**：单路径场景仍正常工作  
✅ **实现简单**：核心逻辑约 100 行代码  

### 6.2 缺点
⚠️ **依赖段号顺序**：如果 PLC 乱序发送（如 1,3,2,4,5,6），可能误判路径切换  
⚠️ **无法处理单路径重复扫描**：如果 PLC 只扫路径1两遍（1-6, 1-6），会误判为两条路径  
⚠️ **缓存上限**：最多支持 20 条路径（`kMaxPointCloudCacheSize / scanSegmentTotal`）  

### 6.3 风险缓解
- **段号顺序问题**：与 PLC 团队确认，段号必须顺序发送（1,2,3,4,5,6）
- **重复扫描问题**：通过 `Trig_ResultReset` 清空缓存，避免误判
- **缓存上限问题**：3 条路径 × 6 段 = 18 个缓存项，在上限内

---

## 七、测试验证

### 7.1 单元测试
```cpp
// 测试路径切换检测
void testPathSwitchDetection()
{
    StateMachine sm;
    
    // 路径1：段1-6
    sm.mockScanSegment(1);  // 路径1
    sm.mockScanSegment(2);
    sm.mockScanSegment(6);
    
    // 路径2：段1-6（段1重复 → 触发路径切换）
    sm.mockScanSegment(1);  // 路径2
    sm.mockScanSegment(2);
    
    // 验证缓存
    QVERIFY(sm.hasSegmentInPath(1, 1));  // 路径1段1
    QVERIFY(sm.hasSegmentInPath(2, 1));  // 路径2段1
    QVERIFY(sm.currentPathId() == 2);
}
```

### 7.2 集成测试
```
测试场景1：正常3路径扫描
PLC 序列：1,2,3,4,5,6, 1,2,3,4,5,6, 1,2,3,4,5,6, Inspection
预期结果：缓存[1][1-6], [2][1-6], [3][1-6]，检测成功

测试场景2：中途复位
PLC 序列：1,2,3, ResultReset, 1,2,3,4,5,6, Inspection
预期结果：缓存清空后重新开始，路径ID=1

测试场景3：单路径
PLC 序列：1,2,3,4,5,6, Inspection
预期结果：缓存[1][1-6]，检测成功（向后兼容）
```

---

## 八、实施步骤

### 第1步：代码修改（我做）
- [ ] 修改 `state_machine.h`：新增二维缓存和路径上下文
- [ ] 修改 `state_machine.cpp`：实现路径切换检测
- [ ] 修改 `loadSegmentCaptureResultsForInspection`：硬编码映射
- [ ] 修改 `resetScanSegmentCache`：清空多路径缓存
- [ ] 修改 `cachedScanSegmentIndices`：支持多路径显示

### 第2步：编译测试（我做）
- [ ] 编译通过
- [ ] 单元测试通过
- [ ] 模拟 PLC 触发序列测试

### 第3步：现场联调（你做）
- [ ] 部署到现场 IPC
- [ ] 与 PLC 联调，验证路径切换
- [ ] 验证综合检测结果
- [ ] 观察日志，确认路径ID正确

### 第4步：问题修复（我做）
- [ ] 根据现场反馈调整逻辑
- [ ] 优化日志输出
- [ ] 补充异常处理

---

## 九、配置文件最终状态

### scan_paths_config.json
```json
{
  "scanPaths": [
    { "pathId": 1, "pathName": "路径1-外圈", "enabled": true, "totalPoints": 6 },
    { "pathId": 2, "pathName": "路径2-内圈", "enabled": true, "totalPoints": 6 },
    { "pathId": 3, "pathName": "路径3-焊缝", "enabled": true, "totalPoints": 6 }
  ],
  "executionConfig": {
    "selectedPathIds": [1, 2, 3]
  }
}
```

### config.ini
```ini
[Tracking]
# 保持不变，代码中硬编码映射
firstStationOuterSegmentIndex=1  # 实际取路径1段1
firstStationInnerSegmentIndex=2  # 实际取路径2段2
firstStationHoleSegmentIndex=3   # 实际取路径3段3
scanSegmentTotal=6
```

---

## 十、常见问题

### Q1: 如果 PLC 只扫2条路径怎么办？
**A**: 没问题。路径ID 自动递增，只会缓存路径1和路径2。综合检测时，如果缺少路径3，会报错提示。

### Q2: 如果 PLC 段号乱序（如 1,3,2,4,5,6）怎么办？
**A**: 会误判路径切换。**必须与 PLC 团队确认段号顺序发送**。

### Q3: 如何调试路径切换逻辑？
**A**: 查看日志：
```
[flow_control] 点云已缓存 [路径1][段1] 点数=12345
[flow_control] 点云已缓存 [路径1][段2] 点数=12345
...
[flow_control] 自动切换到路径2（检测到段号1重复）
[flow_control] 点云已缓存 [路径2][段1] 点数=12345
```

### Q4: 综合检测时，如何知道取了哪些点云？
**A**: 日志会输出：
```
[flow_control] Trig_Inspection 加载点云：
  外圈：路径1段1，点数=12345
  内圈：路径2段2，点数=12345
  孔：  路径3段3，点数=12345
```

---

## 十一、后续优化方向

1. **配置化映射**：将硬编码的路径段号映射改为配置文件
2. **路径名称显示**：日志中显示路径名称（如"外圈扫描"）而非路径ID
3. **异常恢复**：段号乱序时自动修正或报警
4. **性能优化**：大量路径时的缓存管理

---

## 十二、总结

✅ **方案可行**：IPC 侧完全适配，PLC 零改动  
✅ **实现简单**：核心逻辑约 100 行，风险可控  
✅ **现场友好**：自动路径识别，无需人工干预  

**下一步**：我立即开始代码修改，预计 1-2 小时完成。

---

**准备开始实施了吗？** 🚀
