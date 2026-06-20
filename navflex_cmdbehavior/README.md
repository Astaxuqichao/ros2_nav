# navflex_cmdbehavior

`navflex_cmdbehavior` 是一个 Nav2 recovery behavior 插件包，用于通过运行时命令字符串驱动机器人运动。

## 功能

- 提供一个 `nav2_core::Behavior` 插件：`navflex_cmdbehavior::CmdBehavior`
- 支持通过命令控制机器人执行：
  - `linear <distance_m>`：前进/后退
  - `rotate <angle_deg>`：原地旋转
  - `wait <duration_s>`：等待指定时间
- 主要用于与行为树/恢复动作结合，通过 `behavior_action` 发送命令

## 插件描述

插件描述文件：`navflex_cmdbehavior_plugin.xml`

- 插件类型：`nav2_core::Behavior`
- 插件名称：`navflex_cmdbehavior/CmdBehavior`

## 命令格式

- `linear 0.5`：向前移动 0.5 米
- `linear -0.3`：向后移动 0.3 米
- `rotate 1.57`：左转 1.57 弧度（约 90 度）
- `rotate -1.57`：右转 1.57 弧度
- `wait 5`：等待 5 秒

> 命令字符串通过 `behavior` 字段选择该行为，例如 `behavior="cmd_behavior"`。

## 参数

可通过 ROS2 参数配置，参数命名为 `CmdBehavior` 插件名称前缀，例如 `cmd_behavior.linear_vel`。

- `linear_vel` (double, 默认 `0.2`)：线速度，单位 m/s
- `angular_vel` (double, 默认 `0.5`)：角速度，单位 rad/s
- `xy_tolerance` (double, 默认 `0.05`)：线性位置容差，单位 m
- `yaw_tolerance` (double, 默认 `0.017`)：角度容差，单位 rad
- `timeout` (double, 默认 `30.0`)：安全超时时间，单位 s
- `control_frequency` (double, 默认 `10.0`)：控制循环频率，单位 Hz

## 使用示例

在 Behavior Tree 中通过 `NavflexRecoveryAction` 或 `DummyBehavior` 发送命令：

```xml
<Action ID="NavflexRecoveryAction"
        behavior="cmd_behavior"
        command="rotate 1.57"
        server_name="behavior_action"
        server_timeout="10.0"/>

<Action ID="NavflexRecoveryAction"
        behavior="cmd_behavior"
        command="linear -0.3"
        server_name="behavior_action"
        server_timeout="10.0"/>

<Action ID="NavflexRecoveryAction"
        behavior="cmd_behavior"
        command="wait 5"
        server_name="behavior_action"
        server_timeout="10.0"/>
```

## 构建

在工作区根目录下运行：

```bash
colcon build --packages-select navflex_cmdbehavior
```

## 安装

`CMakeLists.txt` 已将库和插件描述文件安装到 ROS2 包共享目录中。

## 注意

- 该行为主要依赖 `global_costmap` 获取当前机器人位姿，以便更准确地判断执行是否完成。
- 如果 global costmap 不可用，行为会退回到基于时间的终止策略。
