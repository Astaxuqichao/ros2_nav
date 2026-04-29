# rcl_logging_spdlog_rotating

ROS 2 Humble 的自定义日志后端，基于 [spdlog](https://github.com/gabime/spdlog) 实现**异步滚动文件日志**。

## 特性

- 异步写入（`spdlog::async_factory` + 后台线程池），不阻塞业务线程
- 按文件大小自动滚动（rotating），可配置保留文件数
- 日志格式简洁：去除原始 ROS2 时间戳和重复的 logger 名
- 通过 `.conf` 配置文件控制所有参数，无需重新编译
- 支持路径中的 `~` 展开

## 输出格式示例

```
[2026-04-17 11:48:48.839] [info] [map_server]: Creating bond to lifecycle manager.
[2026-04-17 11:48:49.001] [warning] [controller_server]: Costmap is not clear.
```

---

## 编译

```bash
cd ~/humble_ws/navflex_ws
colcon build --packages-select rcl_logging_spdlog_rotating
source install/setup.bash
```

---

## 配置文件

参考 `config/ros2_spdlog_rotating.conf`：

```ini
# 日志输出路径（支持 ~ 展开）
log_path=~/logs/ros2_test.log

# 单个文件最大字节数（默认 10MB）
max_size=10485760

# 最多保留文件数（滚动后保留 N 个旧文件）
max_files=5

# 日志级别：debug / info / warn / error / fatal
level=info

# spdlog pattern 格式（%Y-%m-%d 日期，%H:%M:%S.%e 时间，%l 级别，%v 消息）
pattern=[%Y-%m-%d %H:%M:%S.%e] [%l] %v

# 异步队列大小（条数）
queue_size=8192

# 后台写入线程数
thread_count=1
```

---

## 使用方法

### 原因说明

在 ROS 2 Humble 中，`librcl.so` 在编译时已经**硬链接**到 `librcl_logging_spdlog.so`，
`RCL_LOGGING_IMPLEMENTATION` 环境变量在 Humble 中**不生效**。

必须通过 `LD_PRELOAD` 劫持官方 spdlog 库，替换为本库。

### 启动前设置环境变量

```bash
# 必须：指定配置文件（使用绝对路径）
export RCUTILS_LOGGING_CONFIG_FILE=/path/to/ros2_spdlog_rotating.conf

# 必须：通过 LD_PRELOAD 替换官方日志库
export LD_PRELOAD=/path/to/install/rcl_logging_spdlog_rotating/lib/librcl_logging_spdlog_rotating.so
```

### 直接启动节点

```bash
export RCUTILS_LOGGING_CONFIG_FILE=~/humble_ws/navflex_ws/ros2_spdlog_rotating.conf
export LD_PRELOAD=~/humble_ws/navflex_ws/install/rcl_logging_spdlog_rotating/lib/librcl_logging_spdlog_rotating.so

ros2 run <package> <node>
```

### 在 launch 文件中设置

```python
import os
from launch import LaunchDescription
from launch_ros.actions import Node

ws = os.path.expanduser('~/humble_ws/navflex_ws')

env = {
    'RCUTILS_LOGGING_CONFIG_FILE': os.path.join(ws, 'ros2_spdlog_rotating.conf'),
    'LD_PRELOAD': os.path.join(ws, 'install/rcl_logging_spdlog_rotating/lib/librcl_logging_spdlog_rotating.so'),
}

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package',
            executable='your_node',
            additional_env=env,
        ),
    ])
```

### 写入脚本统一管理

```bash
#!/bin/bash
source ~/humble_ws/navflex_ws/install/setup.bash

export RCUTILS_LOGGING_CONFIG_FILE=~/humble_ws/navflex_ws/ros2_spdlog_rotating.conf
export LD_PRELOAD=~/humble_ws/navflex_ws/install/rcl_logging_spdlog_rotating/lib/librcl_logging_spdlog_rotating.so

exec "$@"
```

用法：`./run_with_log.sh ros2 launch nav2_bringup navigation2.launch.py`

---

## 注意事项

- 初始化成功时会向 stderr 输出 `🔥 rcl_logging_spdlog_rotating initialized`，可用于确认库已加载
- 若配置文件不存在或解析失败，日志将写入默认路径 `/tmp/ros2.log`
- `flush_on(info)` 默认对 info 及以上级别立即 flush，保证日志实时性；如要提升写入性能可改为定时 flush
- 进程结束时调用 `rcl_logging_external_shutdown()` 会自动 flush 并关闭文件，确保日志完整

---

## 依赖

| 依赖 | 说明 |
|------|------|
| `rcl_logging_interface` | ROS 2 日志后端接口 |
| `rcutils` | ROS 2 工具库（日志级别常量） |
| `spdlog_vendor` / `libspdlog-dev` | spdlog 库 |
