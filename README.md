# ros2_nav

基于 ROS 2 Humble 的导航扩展功能包集合，在 Nav2 框架之上提供全向机器人仿真、地图仿真激光雷达、自定义代价地图导航服务端、BT 节点及辅助工具。

## 包结构

| 包名 | 说明 |
|------|------|
| `navflex_bringup` | 整合启动包，提供本地仿真启动、完整导航栈启动，并集中管理地图、参数和 RViz 配置 |
| `navflex_costmap_nav` | 核心导航服务端，封装 Nav2 Planner/Controller/Behavior action server，提供完整的代价地图导航能力 |
| `navflex_bt_navigator` | BT Navigator 启动封装与 Navflex 行为树 XML |
| `navflex_bt_nodes` | 自定义 BehaviorTree 节点（GetPath、ExePath、Recovery），供 BT Navigator 调用；ExePath 支持运行中接收重规划路径更新 |
| `navflex_cmdbehavior` | Nav2 恢复行为插件，实现 `nav2_core::Behavior` 接口 |
| `navflex_utility` | 公共工具库，提供 `RobotInformation`、`OdometryHelper`、路径/导航异常类型等 |
| `omni_fake_node` | 全向轮虚拟机器人节点，支持 vx/vy/wz 速度指令，用于无实体硬件时的 Nav2 仿真测试 |
| `simulation_lidar` | 基于 `OccupancyGrid` 地图的 2D 仿真激光雷达，发布 `LaserScan` 与 `PointCloud2`，支持噪声、射线投射、未知区域障碍 |
| `rcl_logging_spdlog_rotating` | ROS 2 自定义日志后端，使用 spdlog 异步滚动写入替换默认日志实现 |

## 依赖

- ROS 2 Humble
- Nav2（`nav2_bringup`、`nav2_core`、`nav2_behavior_tree` 等）: 主要扩展nav2_core以及nav2_msgs
- `tf2`、`tf2_ros`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`

### 依赖：ros2_nav 与 navigation2

克隆本仓库及所有依赖：

```bash
cd ~/humble_ws/navflex_ws
git clone https://github.com/Astaxuqichao/ros2_nav.git -b main
git clone https://github.com/Astaxuqichao/navigation2.git -b humble
git clone https://github.com/SteveMacenski/spatio_temporal_voxel_layer.git -b humble
```

## 编译

```bash
cd ~/humble_ws/navflex_ws/ros2_nav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动方式

### 1. sim_local_launch.py — 仿真本地启动

用于本地仿真测试。启动 `omni_fake_node`（仿真底盘）、`simulation_lidar`、RViz、地图服务器及静态 TF，不启动导航栈。

```bash
ros2 launch navflex_bringup sim_local_launch.py
```

可选参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_file` | `navflex_bringup/maps/map1.yaml` | 地图文件路径 |
| `rviz_config_file` | `navflex_bringup/rviz/nav2_default_view.rviz` | RViz 配置文件路径 |
| `use_sim_time` | `true` | 是否使用仿真时钟 |
| `autostart` | `true` | 是否自动激活 `map_server` |

示例（指定地图）：
```bash
ros2 launch navflex_bringup sim_local_launch.py \
  map_file:=/path/to/your/map.yaml
```

---

### 2. navflex_bringup_launch.py — 完整导航启动

启动完整导航栈：`costmap_nav_node`（规划/控制/行为服务器）+ `bt_navigator` + `lifecycle_manager_navflex`，通过 `navflex_bringup/params/nav2_params.yaml` 和 BT 参数文件配置各插件。

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py
```

可选参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `params_file` | `navflex_bringup/params/nav2_params.yaml` | 参数文件路径 |
| `bt_params_file` | `navflex_bt_navigator/params/navflex_bt_navigator.yaml` | `bt_navigator` 参数文件路径 |
| `default_nav_to_pose_bt_xml` | `navflex_bt_navigator/behavior_trees/test_bt_navigator.xml` | NavigateToPose 使用的行为树 XML |
| `use_sim_time` | `false` | 是否使用仿真时钟 |
| `autostart` | `true` | 是否自动激活生命周期节点 |
| `log_level` | `info` | 日志级别（debug/info/warn/error） |
| `use_respawn` | `False` | 节点崩溃后是否自动重启 |
| `namespace` | `` | 命名空间 |
| `use_route_server` | `False` | 是否同时启动 nav2_route 路网服务器 |
| `graph_filepath` | `nav2_route/graphs/sample_graph.geojson` | 路网图文件路径 |

示例（启用路网服务器，使用仿真时钟）：
```bash
ros2 launch navflex_bringup navflex_bringup_launch.py \
  use_route_server:=True \
  graph_filepath:=/path/to/your_graph.geojson \
  use_sim_time:=true
```

#### 生命周期节点管理

- 默认管理 `navflex_costmap_nav` 与 `bt_navigator`
- 激活顺序为 `navflex_costmap_nav` -> `route_server`（可选）-> `bt_navigator`
- 启用 `use_route_server:=True` 时，`route_server` 会加入**同一个** `lifecycle_manager_navflex` 统一管理

#### 行为树路径更新

- 默认行为树位于 `navflex_bt_navigator/behavior_trees/test_bt_navigator.xml`
- `RateController` 会周期性重新规划并更新 blackboard 中的 `{path}`
- `NavflexExePathAction` 在 FollowPath 运行中检测 `{path}`、`controller_id`、`goal_checker_id` 变化，并重新发送 FollowPath goal
- `navflex_costmap_nav` 的 controller action server 对同 controller、同 goal checker 的新 FollowPath goal 执行原地 plan update，避免完整重启控制器

---

## 快速启动

### 第一步：启动本地仿真环境

启动全向虚拟机器人（`omni_fake_node`）、仿真激光雷达（`simulation_lidar`）、地图服务器及 RViz：

```bash
ros2 launch navflex_bringup sim_local_launch.py
```

### 第二步：启动导航栈

```bash
ros2 launch navflex_bringup navflex_bringup_launch.py use_sim_time:=true
```

### 第三步：启动导航测试客户端

监听 `/goal_pose` 话题，自动调用规划与跟踪 action server：

```bash
ros2 run navflex_costmap_nav nav_test.py
```

在 RViz 中使用 **2D Goal Pose** 工具发布目标点，客户端将依次调用 `ComputePathToPose` 和 `FollowPath`。

也可以直接使用 `bt_navigator` 的 NavigateToPose action，由 `navflex_bt_navigator` 的行为树完成规划、控制和恢复逻辑。

## TODO
1. 根据实际场景继续完善全局重规划、恢复策略和路网导航行为树
