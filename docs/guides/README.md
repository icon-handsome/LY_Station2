# 第二工位开发指南

本目录存放 **IPC_Station2** 的流程说明、配置参考与续接文档。

## 阅读顺序（建议）

| 顺序 | 文档 | 说明 |
|------|------|------|
| 1 | [第二工位骨架删减说明](./第二工位骨架删减说明.md) | **入口**：删减范围、保留模块、默认配置 |
| 2 | [扫描路径配置说明](./扫描路径配置说明.md) | scan_paths JSON、落盘目录、Res 码 |
| 3 | [flow_control架构说明](./flow_control架构说明.md) | StateMachine / PlcTaskHost / Handler 分工 |
| 4 | [双工位改造进度与续接指南](./双工位改造进度与续接指南.md) | 阶段规划、硬件清单、续接 Prompt |
| 5 | [工作流程-AI-Agent参考](./工作流程-AI-Agent参考.md) | 三模式流程、Modbus 数据流、决策节点 |

## 关联文档（仓库内）

| 主题 | 路径 |
|------|------|
| 流程时序图（2.1~2.5） | [`../sequence_diagrams/`](../sequence_diagrams/) |
| HMI TCP 联调 | [`../hmi/HMI开发交接说明.md`](../hmi/HMI开发交接说明.md) |
| PLC / HMI 协议 | [`../protocols/`](../protocols/) |
| 工位 Profile 配置 | `config/station_profiles/`、`config/scan_paths/` |
| 原始流程 PDF | [`../vendor/第二工位_工作流程(动作+数据).pdf`](../vendor/第二工位_工作流程(动作+数据).pdf) |

---

*最后更新：2026-06-16*
