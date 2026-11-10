# Legged Navigation Runtime

[![CI](https://github.com/zxinyu-robot/legged-navigation-runtime/actions/workflows/ci.yml/badge.svg)](https://github.com/zxinyu-robot/legged-navigation-runtime/actions/workflows/ci.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

> 面向双足和足式机器人的故障恢复定位与多楼层导航运行时。

## 项目状态

| 项目 | 状态 |
| --- | --- |
| 维护状态 | Active |
| 实现范围 | C++20、无 ROS 依赖、可单元测试 |
| 验证方式 | 回放 demo、单元测试、Linux/macOS CI |
| 生产集成 | ROS 2、Nav2 和厂商 SDK 通过适配层接入 |

## 项目简介

本仓库是 Unitree G1/H2 级机器人导航工程的脱敏公开重构，聚焦定位前端失效后的安全恢复链路。状态机只输出可审计的恢复动作，不输出速度、关节力矩或足端指令；底盘或全身控制器负责最终安全约束。

```text
定位失效 → 暂停运动 → Scan Context 候选搜索 → NDT / ICP 精配准
          → Localizer reset → 连续健康帧确认 → 恢复 Nav2 任务
```

## 核心能力

| 能力 | 当前公开实现 |
| --- | --- |
| 故障恢复状态机 | `RelocalizationSupervisor`：Tracking、Searching、Aligning、Verifying、SafeStop |
| 恢复动作 | 暂停运动、候选搜索、精配准、Localizer reset、恢复任务、安全停车 |
| 结果门控 | 位姿跳变量与 alignment fitness 双阈值 |
| 超时保护 | 搜索 / 配准超时后进入 `SafeStop` |
| 恢复确认 | 连续健康帧通过后才恢复导航 |
| 延迟统计 | 固定容量窗口、nearest-rank P50/P95/最大值 |
| 平台配置 | G1/H2 独立配置示例 |

## 快速开始

要求：CMake 3.20+、Make 和支持 C++20 的编译器。

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default --output-on-failure
./build/default/recovery_demo
```

示例输出使用合成延迟序列，只用于验证统计代码，不代表实机性能。

## 恢复状态模型

```text
Tracking → Searching → Aligning → Verifying → Tracking
    │          │           │           │
    └──────────┴───────────┴──────────→ SafeStop
```

- `Tracking`：正常定位和导航；单帧异常先保持运动安全。
- `Searching`：暂停导航并搜索全局描述子候选；超时进入 `SafeStop`。
- `Aligning`：执行 NDT/ICP，只有通过位姿跳变和 fitness 门控才接受。
- `Verifying`：已 reset localizer，但等待连续健康帧后再恢复任务。
- `SafeStop`：自动恢复耗尽或超时，需显式 `reset()` 解除锁存。

## 集成边界

生产适配层需要连接定位健康输入、Scan Context 候选服务、NDT/ICP 结果、localizer reset、Nav2 暂停/恢复 action 和平台 safe-stop service。ROS 2 集成应统一使用 ROS time、拒绝过期响应、将状态机更新放在互斥 callback group，并保持恢复动作与底盘安全层分离。

多楼层规划位于恢复运行时之上：拓扑规划器 → 楼层 / 楼梯转换边 → SCAN-Planner / PCT-Planner 三维可通行段 → 楼梯控制器 → 楼层地图切换。定位 reset 不能隐式改变语义楼层，也不能恢复过期路径。

## 公开实现边界

已包含：恢复状态机、结果门控、超时保护、连续健康帧确认、延迟窗口、可运行回放 demo、单元测试和跨平台 CI。

不包含：FAST-LIO2、Nav2、SCAN-Planner、PCT-Planner 第三方源码，厂商 SDK、实机地图、完整 ROS 2 节点、Scan Context/NDT/ICP 算法实现，以及原项目的多楼层地图、楼梯控制参数和 rosbag。

## 工程结果与限制

以下结果来自对应私有工程经历，不是当前仓库 CI 可复现 benchmark：Scan Context + NDT/ICP 自动重定位约 `1.0–1.2 s`，LOST 场景恢复链路触发约 `0.5 s`，G1/H2 定位前端 P95 分别约 `25.9 ms` / `16.3 ms`。复现实测需要原始硬件、地图、rosbag 和完整 ROS 2 集成，指标口径见 [benchmark methodology](docs/benchmark-methodology.md)。

## 项目结构

```text
include/legged_navigation/  公开头文件
src/                        实现与 recovery demo
tests/                      单元测试
config/                     G1/H2 配置
docs/                       系统设计与 benchmark 口径
```

## 路线图

1. 增加 ROS 2 lifecycle node 和诊断消息适配器。
2. 增加 Scan Context 候选与配准结果录制回放接口。
3. 增加楼层拓扑任务模型和 Nav2 action 恢复适配器。
4. 在公开数据集上发布可复现的失效注入与延迟报告。

## 许可证

[MIT](LICENSE)
