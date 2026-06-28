# navflex

Navflex 是基于 ROS 2 Humble 和 Nav2 的导航扩展包集合。它把 costmap、规划、控制、BT 行为树、自定义恢复行为、本地仿真和 TB3 manipulation Gazebo 仿真整理成一套可直接启动的导航工作区。

更详细的启动参数和排查命令见 [navflex_bringup/README.md](navflex_bringup/README.md)。

## 包功能

| 包名 | 功能 |
| --- | --- |
| `navflex_bringup` | 统一启动包，维护 launch、地图、参数、RViz 配置；提供本地仿真、TB3 manipulation Gazebo 仿真和 Navflex 导航栈启动入口 |
| `navflex_costmap_nav` | 核心导航服务端；在一个 lifecycle 节点内管理 global/local costmap，并提供 `/compute_path_to_pose`、`/follow_path`、`/behavior_action` |
| `navflex_bt_navigator` | BT Navigator 封装包，提供 Navflex 默认行为树 XML 和 BT Navigator 参数 |
| `navflex_bt_nodes` | 自定义 BT 节点，包括 GetPath、ExePath、Recovery；`NavflexExePathAction` 支持运行中路径更新和 FollowPath 容差传递 |
| `navflex_cmdbehavior` | Nav2 behavior 插件，提供 `rotate`、`linear`、`wait` 等简单恢复/动作命令 |
| `navflex_exclusion_zone` | Nav2 costmap 电子禁区层插件，可通过话题动态标记禁行区域 |
| `navflex_instruction_server` | 文本指令、语义地图、任务服务和 VLN/VLM 桥接入口 |
| `navflex_utility` | 公共工具库，提供机器人状态、TF/odom 查询和导航辅助函数 |
| `omni_fake_node` | 本地全向虚拟机器人，订阅 `/cmd_vel`，发布 `/odom`、TF 和本地仿真 `/clock` |
| `simulation_lidar` | 基于 OccupancyGrid 地图射线投射的 2D 仿真激光雷达，发布 `/scan` 和 `/scan_cloud` |
| `rcl_logging_spdlog_rotating` | 自定义 ROS 2 spdlog 滚动日志后端 |

## 主要依赖

- ROS 2 Humble
- Nav2 源码包 `navigation2`
- `spatio_temporal_voxel_layer`
- TurtleBot3 / TurtleBot3 Manipulation / TurtleBot3 Simulations
- Gazebo Classic 11
- `ros2_control`、`controller_manager`、`xacro`

从新环境准备依赖时，建议在工作区根目录执行：

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## 编译

```bash
cd <workspace>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果改过 `src/navigation2/nav2_msgs/action/FollowPath.action`，需要重编依赖该 action 的包，至少包括：

```bash
colcon build --packages-select \
  nav2_msgs nav2_behavior_tree nav2_bt_navigator \
  navflex_costmap_nav navflex_bt_nodes navflex_bt_navigator \
  navflex_bringup
source install/setup.bash
```

## 本地全向仿真快速启动

终端 1，启动本地仿真环境：

```bash
ros2 launch navflex_bringup sim_local_launch.py
```

终端 2，启动 Navflex 导航栈：

```bash
source install/setup.bash
ros2 launch navflex_bringup navflex_bringup_launch.py
```

默认导航参数为 `chassis_model:=omni`，适配 `omni_fake_node`。

## TB3 Manipulation 移动操作仿真

TB3 manipulation 是差速底盘，不是全向底盘。启动导航时必须使用 `chassis_model:=diff`。

终端 1，启动 TB3 manipulation Gazebo 仿真：

```bash
ros2 launch navflex_bringup tb3_manipulation_sim_launch.py
```

默认是无头 Gazebo server，并启动 RViz。需要 Gazebo GUI 时有两种方式：

```bash
ros2 launch navflex_bringup tb3_manipulation_sim_launch.py gui:=true
```

或者在无头仿真已经启动后，另开终端连接：

```bash
gzclient
```

终端 2，启动 Navflex 导航栈并选择 TB3 差速参数：

```bash
source install/setup.bash
ros2 launch navflex_bringup navflex_bringup_launch.py chassis_model:=diff
```

启动后可在 RViz 使用 `2D Goal Pose` 发送导航目标。

常用检查：

```bash
ros2 topic echo /clock
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /local_costmap/costmap
ros2 topic hz /global_costmap/costmap
```

## FollowPath 到点容差

Navflex 扩展了 `nav2_msgs/action/FollowPath`：

- `xy_goal_tolerance`
- `yaw_goal_tolerance`

两个值都为 `0.0` 时，`navflex_costmap_nav` 使用参数文件中的默认到点精度。控制器内部仍通过 `nav2_core::GoalChecker` 判断是否到点；每个新的 FollowPath goal 会按 action 容差生成对应的 goal checker。

## 文档入口

- Bringup、依赖、TB3 仿真和排查：[navflex_bringup/README.md](navflex_bringup/README.md)
- 核心导航服务端：[navflex_costmap_nav/README.md](navflex_costmap_nav/README.md)
- 文本/语义任务入口：[navflex_instruction_server/README.md](navflex_instruction_server/README.md)
- 本地全向假机器人：[omni_fake_node/README.md](omni_fake_node/README.md)

## License

Apache-2.0
