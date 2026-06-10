# 第二工位 / 双工位改造文档

本目录存放 **兰铀第二工位** 与 **双工位同代码改造** 相关的规划、流程与 Agent 交接文档。原位于 `D:\work\兰铀\第二工位\文档`，已迁入仓库便于版本管理与协作。

## 阅读顺序（建议）

| 顺序 | 文档 | 说明 |
|------|------|------|
| 1 | [双工位改造进度与续接指南](./双工位改造进度与续接指南.md) | **入口**：当前阶段、已完成项、硬件清单、续接 Prompt |
| 2 | [工作流程-AI-Agent参考](./工作流程-AI-Agent参考.md) | 第二工位三模式流程、Modbus 数据流、决策节点（源自 PDF） |
| 3 | [第一工位重构计划](./第一工位重构计划.md) | 六阶段重构总规划与代码锚点 |
| 4 | [Agent提示词-阶段1-双工位架构落地](./Agent提示词-阶段1-双工位架构落地.md) | 阶段 1 执行 Prompt（已完成，归档参考） |

## 关联文档（仓库内）

| 主题 | 路径 |
|------|------|
| 第二工位时序图（2.1~2.5） | [`../sequence_diagrams/station2/`](../sequence_diagrams/station2/) |
| 第一工位时序图（1.1~1.11） | [`../sequence_diagrams/station1/`](../sequence_diagrams/station1/) |
| 双工位时序图总索引 | [`../sequence_diagrams/README.md`](../sequence_diagrams/README.md) |
| 第一工位 IPC 待办 | [`../station1/项目未完成事项清单_v1.0.md`](../station1/项目未完成事项清单_v1.0.md) |
| 第一工位现场联调 | [`../station1/现场联调_PLC不改方案.md`](../station1/现场联调_PLC不改方案.md) |
| 工位 Profile 配置 | 仓库根目录 `config/station_profiles/`、`config.ini [Station]` |

## 原始资料

- 流程 PDF：`D:\work\兰铀\第二工位\文档\2 工作流程(动作+数据).pdf`（未纳入 git，仍保留在外部目录）

---

*最后更新：2026-06-10*
