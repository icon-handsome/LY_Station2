# 项目文档索引

本目录集中存放 scan-tracking IPC 项目的协议、联调、算法与双工位规划文档。

## 目录结构

| 目录 | 内容 |
|------|------|
| [station1/](./station1/) | 第一工位 IPC：扫描流程、算法 API、待办、现场联调 |
| [station2/](./station2/) | 第二工位 / 双工位改造：流程参考、进度续接、重构计划 |
| [protocols/](./protocols/) | PLC Modbus、HMI TCP 协议与寄存器表 |
| [hmi/](./hmi/) | HMI TCP 服务端交接与现场联调 |
| [sequence_diagrams/](./sequence_diagrams/) | 第一/第二工位流程时序图（1.x / 2.x） |
| [vendor/](./vendor/) | 第三方设备用户手册（PDF 等） |

## 快速入口

| 我要… | 文档 |
|-------|------|
| 查当前待办 | [station1/项目未完成事项清单_v1.0.md](./station1/项目未完成事项清单_v1.0.md) |
| 理解扫描与落盘 | [station1/多点位扫描与位姿跟踪完整流程.md](./station1/多点位扫描与位姿跟踪完整流程.md) |
| 查算法 / LBN / 缓存 API | [station1/算法使用API.md](./station1/算法使用API.md) |
| PLC Modbus 联调 | [protocols/封头检测工位PLC-IPC Modbus通信协议_v0.1.md](./protocols/封头检测工位PLC-IPC%20Modbus通信协议_v0.1.md) |
| HMI TCP 对接 | [hmi/HMI开发交接说明.md](./hmi/HMI开发交接说明.md) |
| 第二工位续接开发 | [station2/双工位改造进度与续接指南.md](./station2/双工位改造进度与续接指南.md) |
| 看时序图 | [sequence_diagrams/README.md](./sequence_diagrams/README.md) |

---

*最后更新：2026-06-10*
