# Legged Terrain State

> 本模块由原仓库 [legged-terrain-state](https://github.com/zxinyu-robot/legged-terrain-state) 并入，原仓库已归档；构建与测试见仓库根目录 README。

面向四足机器人混合地形导航的 C++20 状态契约与安全降级核心。

项目将定位、地形与接触感知统一为带时间戳、坐标系、置信度和故障标志的 `BaseState / TerrainState / ContactState`，使导航与运控能够在感知稀疏、坡度过大或地形粗糙时执行可解释的降速与停车策略。

## 公开实现

- Base/Terrain/Contact 三类状态契约。
- 支撑面高度、法向、坡度、粗糙度与支撑置信度。
- 可通行性评分与 Nominal/Caution/Critical 风险分级。
- `Normal / ReduceSpeed / HoldPosition` 安全模式。
- 稀疏支撑、非法法向、过大坡度和粗糙度故障标志。
- 单元测试、回放 Demo 和 Linux/macOS CI。

## 系统边界

```text
FAST-LIO2 / terrain perception / contact estimator
                       ↓
       BaseState / TerrainState / ContactState
                       ↓
                TerrainAssessor
                       ↓
       navigation constraints / safety mode
                       ↓
         locomotion or whole-body controller
```

本仓库不包含 FAST-LIO2、LIO-SAM、点云分割、局部高程图或厂商控制器源码；它公开的是感知和运控之间可独立测试的状态契约。

## 快速运行

```bash
cmake --preset default
cmake --build --preset default
ctest --preset default --output-on-failure
./build/default/terrain_demo
```

## 工程案例背景

该公开重构对应四足机器人混合地形感知导航经历：

- 以 FAST-LIO2 为主、LIO-SAM 为对照，处理机体抖动和足端冲击下的点云/IMU 链路。
- 构建局部高程地图与可通行性评估，向导航/运控输出目标高度、风险等级和可落脚区域约束。
- 使用 ROS 2 DDS QoS、rosbag2、RViz2、Foxglove、evo 和 PlotJuggler 评估轨迹、地图与端到端延迟。
- 平地和坡面参与实机联调；楼梯场景以仿真验证为主。

这些经历不是本仓库 CI 可复现的真机结果。详细边界见 [架构说明](docs/architecture.md)。

## License

[MIT](LICENSE)
