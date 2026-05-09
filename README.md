# ros2_nav

基于 ROS 2 Humble 的导航扩展功能包集合，在 Nav2 框架之上提供全向机器人仿真、地图仿真激光雷达、自定义代价地图导航服务端、BT 节点及辅助工具。

## 包结构

| 包名 | 说明 |
|------|------|
| `navflex_costmap_nav` | 核心导航服务端，封装 Nav2 Planner/Controller/Behavior action server，提供完整的代价地图导航能力；包含多套 launch 文件 |
| `navflex_bt_nodes` | 自定义 BehaviorTree 节点（GetPath、ExePath、Recovery），供 BT Navigator 调用 |
| `navflex_cmdbehavior` | Nav2 恢复行为插件，实现 `nav2_core::Behavior` 接口 |
| `navflex_utility` | 公共工具库，提供 `RobotInformation`、`OdometryHelper`、路径/导航异常类型等 |
| `omni_fake_node` | 全向轮虚拟机器人节点，支持 vx/vy/wz 速度指令，用于无实体硬件时的 Nav2 仿真测试 |
| `simulation_lidar` | 基于 `OccupancyGrid` 地图的 2D 仿真激光雷达，发布 `LaserScan` 与 `PointCloud2`，支持噪声、射线投射、未知区域障碍 |
| `rcl_logging_spdlog_rotating` | ROS 2 自定义日志后端，使用 spdlog 异步滚动写入替换默认日志实现 |

## 依赖

- ROS 2 Humble
- Nav2（`nav2_bringup`、`nav2_core`、`nav2_behavior_tree` 等）: 主要扩展nav2_core以及nav2_msgs
- `tf2`、`tf2_ros`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`

## 编译

```bash
cd ~/humble_ws/navflex_ws/ros2_nav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 快速启动

### 第一步：启动仿真机器人与激光雷达

启动全向虚拟机器人（`omni_fake_node`）、仿真激光雷达（`simulation_lidar`）及 RViz：

```bash
ros2 launch navflex_costmap_nav sim_local_launch.py
```

### 第二步：启动导航栈

```bash
ros2 launch navflex_costmap_nav navigation_launch_test.launch.py
```

### 第三步：启动导航测试客户端

监听 `/goal_pose` 话题，自动调用规划与跟踪 action server：

```bash
ros2 run navflex_costmap_nav nav_test.py
```

在 RViz 中使用 **2D Goal Pose** 工具发布目标点，客户端将依次调用 `ComputePathToPose` 和 `FollowPath`。

## TODO
1. 行为树适配nav2_bt_navigator，加入