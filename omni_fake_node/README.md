# OmniFakeNode

一个支持全向运动的 ROS2 C++ 假机器人节点，用于测试 Nav2。功能对标 `turtlebot3_fake_node`，但支持全向运动（vx, vy, wz）。

## 功能特性

- **订阅** `/cmd_vel` (geometry_msgs/Twist) - 接收来自 Nav2 或其他控制节点的速度命令
- **发布** `/odom` (nav_msgs/Odometry) - 发布里程表信息
- **发布** TF transforms (odom → base_link) - 发布坐标系转换
- **全向运动支持**:
  - `vx`: 前后运动 (m/s)
  - `vy`: 左右运动 (m/s)
  - `wz`: 旋转运动 (rad/s)
- **简单积分模型**: 无动力学模型，直接根据速度命令积分更新位置

## 编译

在工作区根目录运行：

```bash
colcon build --packages-select omni_fake_node
```

## 运行

### 方式 1: 使用 Launch 文件

```bash
ros2 launch omni_fake_node omni_fake_node.launch.py
```

### 方式 2: 直接运行节点

```bash
ros2 run omni_fake_node omni_fake_node
```

## 参数

节点支持以下参数：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `update_rate` | double | 50.0 | 更新频率 (Hz) |
| `cmd_vel_timeout` | double | 0.5 | 无速度命令超时时间 (s)，超时后停止运动 |
| `odom_frame` | string | "odom" | 里程表坐标系名称 |
| `base_link_frame` | string | "base_link" | 机器人基链坐标系名称 |

### 自定义参数运行

```bash
ros2 run omni_fake_node omni_fake_node --ros-args -p update_rate:=100.0 -p cmd_vel_timeout:=1.0 -p odom_frame:=odom -p base_link_frame:=base_link
```

### 功能说明：cmd_vel_timeout

- 当订阅到 `/cmd_vel` 话题时，更新最后接收时间
- 每次定时器触发时，检查距离上次接收 `/cmd_vel` 的时间
- 如果超过 `cmd_vel_timeout` 秒，自动将所有速度命令（vx、vy、wz）置为 0
- 这确保了当外部控制节点异常退出或通信中断时，机器人能够安全停止

## 与 Nav2 集成

1. 在 Nav2 的配置中，确保使用本节点作为假机器人
2. 正确配置 TF 树：确保 `map` → `odom` → `base_link` 的转换链路
3. 节点会自动发布 `odom` → `base_link` 的转换

## 坐标系说明

- **odom**: 世界坐标系（里程表）
- **base_link**: 机器人基链（中心）

速度在里程表中积分，根据当前姿态进行坐标转换。

## 实现细节

- 使用简单的积分模型：位置通过累积速度命令计算
- 速度命令在机器人坐标系中接收，转换到世界坐标系后积分
- 发布频率由 `update_rate` 参数控制
- 姿态使用四元数表示，便于 TF 处理

## 测试

### 订阅里程表
```bash
ros2 topic echo /odom
```

### 查看 TF 树
```bash
ros2 run tf2_tools view_frames.py
```

### 发送速度命令
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.5, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}'
```

## 注意事项

- 本节点假设 `/cmd_vel` 中的 `vx` 和 `vy` 为机器人坐标系中的速度
- 里程表仅基于速度积分，没有考虑实际的动力学或轮子滑动
- 适合仿真和测试环境，不适合真实机器人

## 许可证

Apache-2.0
