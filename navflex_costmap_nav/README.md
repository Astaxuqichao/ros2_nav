# navflex_costmap_nav

NavFlex 导航框架核心包，基于 Nav2 插件体系，提供规划（Planner）、控制（Controller）和行为（Behavior）三个 action server。

## 构建

```bash
cd ~/humble_ws/navflex_ws/ros2_nav
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 启动方式

### 1. sim_local_launch.py — 仿真本地启动

用于本地仿真测试。启动 omni_fake_node（仿真底盘）、RViz、地图服务器及静态 TF，不依赖外部导航栈。

```bash
ros2 launch navflex_costmap_nav sim_local_launch.py
```

可选参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `map_file` | `nav2_bringup/maps/map1.yaml` | 地图文件路径 |
| `rviz_config_file` | `nav2_bringup/rviz/nav2_default_view.rviz` | RViz 配置文件路径 |

示例（指定地图）：
```bash
ros2 launch navflex_costmap_nav sim_local_launch.py \
  map_file:=/path/to/your/map.yaml
```

---

### 2. navigation_launch_test.launch.py — 完整导航启动

启动完整导航栈：`costmap_nav_node`（规划/控制/行为服务器）+ `lifecycle_manager`，通过参数文件配置各插件。

```bash
ros2 launch navflex_costmap_nav navigation_launch_test.launch.py
```

可选参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `params_file` | `params/nav2_params.yaml` | 参数文件路径 |
| `use_sim_time` | `false` | 是否使用仿真时钟 |
| `autostart` | `true` | 是否自动激活生命周期节点 |
| `log_level` | `info` | 日志级别（debug/info/warn/error） |
| `use_respawn` | `False` | 节点崩溃后是否自动重启 |
| `namespace` | `` | 命名空间 |

示例（使用仿真时钟，debug 日志）：
```bash
ros2 launch navflex_costmap_nav navigation_launch_test.launch.py \
  use_sim_time:=true \
  log_level:=debug
```

---

## 测试 action

导航栈启动并激活后，可通过命令行发送 action 目标进行测试。

### Behavior action（行为）

```bash
# 直线行驶 1.0 m
ros2 action send_goal /behavior_action nav2_msgs/action/DummyBehavior \
  "{behavior: 'cmd_behavior', command: {data: 'linear 1.0'}}"

# 旋转 -1.0 rad（约 -57°）
ros2 action send_goal /behavior_action nav2_msgs/action/DummyBehavior \
  "{behavior: 'cmd_behavior', command: {data: 'rotate -1.0'}}"

# 原地等待 2.0 秒
ros2 action send_goal /behavior_action nav2_msgs/action/DummyBehavior \
  "{behavior: 'cmd_behavior', command: {data: 'wait 2.0'}}"
```

### FollowPath action（路径跟踪）

```bash
ros2 action send_goal /follow_path nav2_msgs/action/FollowPath \
  "{path: {header: {frame_id: 'map'}, poses: []}, controller_id: '', goal_checker_id: ''}"
```

### ComputePathToPose action（全局规划）

```bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}, planner_id: ''}"
```

---

## 包结构

```
navflex_costmap_nav/
├── include/navflex_costmap_nav/    # 头文件
├── src/
│   ├── navflex_costmap_nav/        # 主节点和三个服务器实现
│   │   ├── navflex_costmap_nav.cpp
│   │   ├── behavior_action_costmap_server.cpp
│   │   ├── controller_action_costmap_server.cpp
│   │   └── planner_action_costmap_server.cpp
│   ├── navflex_base/               # Action/Execution 基类实现
│   └── main.cpp
├── launch/                         # 启动文件
└── params/                         # 参数文件
```
