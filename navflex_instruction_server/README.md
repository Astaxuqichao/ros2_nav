# navflex_instruction_server

`navflex_instruction_server` 是一个独立 ROS2 功能包，用文本指令服务把上层抽象语言调用转换成现有 Navflex 导航能力。它不重新实现规划、控制或底层运动，而是复用：

- `/compute_path_to_pose` (`nav2_msgs/action/ComputePathToPose`)
- `/follow_path` (`nav2_msgs/action/FollowPath`)
- `/behavior_action` (`nav2_msgs/action/DummyBehavior` + `cmd_behavior`)

同时提供网页控制台，用直接文本指令或对话方式调用这些能力。

## 服务接口

### `/navflex_instruction/capabilities`

类型：`navflex_instruction_server/srv/GetCapabilities`

用于能力发现，类似 MCP 工具的 `list tools`。返回支持的指令类型、示例和服务 schema。

```bash
ros2 service call /navflex_instruction/capabilities \
  navflex_instruction_server/srv/GetCapabilities {}
```

### `/navflex_instruction/execute`

类型：`navflex_instruction_server/srv/ExecuteInstruction`

输入一段抽象文本指令，节点解析后调用 Navflex action。

```bash
ros2 service call /navflex_instruction/execute \
  navflex_instruction_server/srv/ExecuteInstruction \
  "{instruction: 'go to 1.0 2.0 90deg'}"
```

响应字段：

- `success`：是否执行成功
- `command_type`：解析后的类型，`navigate` / `linear` / `rotate` / `wait`
- `normalized_command`：标准化后的内部命令
- `message`：执行结果说明
- `steps`：后端记录的导航过程，包括解析、调用规划插件、控制插件或行为插件、完成/失败原因
- `elapsed_time`：总耗时

## 实时状态

执行过程中，节点会把步骤和 action feedback 发布到：

```text
/navflex_instruction/events
```

类型：`std_msgs/msg/String`，内容是 JSON 字符串。

事件类型包括：

- `step`：指令解析、检查 action server、调用插件、完成/失败等过程步骤
- `feedback`：来自 action feedback 的实时状态，例如 `follow_path` 的 `distance_to_goal`、`speed`、`current_pose`、`last_cmd_vel`
- `completed` / `failed`：最终执行状态

网页通过 `GET /api/events` 使用 Server-Sent Events 订阅这些实时状态。

## 支持指令

坐标导航：

```text
go to 1.0 2.0
go to 1.0 2.0 90deg
去 1.0 2.0
去1.0 2.0
```

命名地点导航：

```text
goto kitchen
去 充电桩
```

命名地点推荐通过参数文件配置：

```yaml
navflex_instruction_server:
  ros__parameters:
    named_locations:
      - "kitchen:1.0,2.0,1.57"
      - "充电桩:0.0,0.0,0.0"
```

```bash
ros2 run navflex_instruction_server navflex_instruction_server_node.py --ros-args \
  --params-file /path/to/navflex_instruction.yaml
```

相对运动：

```text
forward 0.5m
前进0.5米
backward 0.3
后退 0.3
rotate 90deg
左转90度
右转 45deg
原地旋转 -1.57rad
wait 2.0
等待2秒
```

相对运动会转换为 `cmd_behavior` 支持的命令：

- `linear <distance_m>`
- `rotate <angle_rad>`
- `wait <duration_s>`

## 启动

```bash
ros2 launch navflex_instruction_server instruction_server.launch.py
```

带命名地点参数文件：

```bash
ros2 launch navflex_instruction_server instruction_server.launch.py \
  params_file:=/path/to/navflex_instruction.yaml
```

## 网页交互

启动文本指令服务和网页：

```bash
ros2 launch navflex_instruction_server instruction_web.launch.py
```

浏览器打开：

```text
http://localhost:8080
```

页面提供：

- 对话调用：可输入“帮我去厨房”“往前走半米”“右转90度”等自然语言。
- 指令输入框：输入 `去 1.0 2.0`、`前进0.5米`、`左转90度` 等文本。
- 示例按钮：点击后自动填入指令。
- 实时过程时间线：显示后端步骤和 action feedback。
- 结果面板：显示 `success`、`command_type`、`normalized_command`、`message`、`steps` 和耗时。

只启动网页桥接，不自动启动 `/navflex_instruction/*` 服务：

```bash
ros2 launch navflex_instruction_server instruction_web.launch.py \
  start_instruction_server:=false
```

修改监听地址和端口：

```bash
ros2 launch navflex_instruction_server instruction_web.launch.py \
  host:=0.0.0.0 port:=8080
```

网页后端 HTTP API：

- `GET /api/capabilities`：调用 `/navflex_instruction/capabilities`
- `GET /api/llm_status`：查看对话模型状态和降级原因
- `GET /api/events`：订阅 `/navflex_instruction/events` 实时状态
- `POST /api/execute`：调用 `/navflex_instruction/execute`
- `POST /api/chat`：对话输入，解析为基础导航指令并按配置执行

```bash
curl -X POST http://localhost:8080/api/execute \
  -H 'Content-Type: application/json' \
  -d '{"instruction":"前进0.5米"}'
```

```bash
curl -X POST http://localhost:8080/api/chat \
  -H 'Content-Type: application/json' \
  -d '{"message":"帮我往前走半米"}'
```

## 大语言模型对话

网页节点支持可选 LLM 适配。对话会先由模型解析成单条 Navflex 指令，再调用 `/navflex_instruction/execute`。

### OpenAI Platform 后端

配置 `OPENAI_API_KEY` 后，使用 OpenAI Responses API：

```bash
export OPENAI_API_KEY=sk-...
ros2 launch navflex_instruction_server instruction_web.launch.py \
  llm_enabled:=true llm_backend:=openai llm_model:=gpt-5.5 auto_execute_chat:=true
```

### Codex CLI 临时后端

如果暂时没有 OpenAI Platform API key，可以使用本机已登录的 Codex CLI 作为解析后端：

```bash
ros2 launch navflex_instruction_server instruction_web.launch.py \
  llm_enabled:=true llm_backend:=codex_cli auto_execute_chat:=true
```

这个后端会调用：

```bash
codex exec --skip-git-repo-check -
```

要求本机 `codex` 命令可以非交互运行。如果 `/api/llm_status` 返回 `codex_cli_error` 或 `fallback`，并且 `reason` / `last_error` 里有 `SyntaxError: Unexpected reserved word`，通常是系统 Node.js 版本太旧，需要升级到 Codex CLI 支持的 Node 版本，或者把 `codex_command` 指向可运行的 Codex 启动脚本。

没有可用后端或模型请求失败时，会自动退回本地规则解析。为了控制安全边界，模型只负责输出 `execute` / `clarify` / `answer` 意图；真正的导航执行仍然走本包的 ROS service 和既有 Navflex action。

检查当前是否真的走到了模型：

```bash
curl http://localhost:8080/api/llm_status
```

常见状态：

- `ready`：网页节点已读到 key，下一次对话会优先走 OpenAI。
- `local`：显式使用本地规则解析。
- `missing_codex_cli`：`llm_backend:=codex_cli`，但找不到 `codex` 命令。
- `codex_cli_error`：找到了 `codex` 命令，但 CLI 自身无法启动，常见原因是 Node.js 版本太旧。
- `missing_key`：`OPENAI_API_KEY` 没有进入 `ros2 launch` 进程环境。需要在同一个 shell 里先 `export OPENAI_API_KEY=...`，再启动 launch。
- `disabled`：`llm_enabled:=false`。
- `fallback`：模型请求失败，`last_error` 会包含 HTTP 错误、模型不可用、网络错误或 JSON 解析错误。

如果 `last_error` 是 `Network is unreachable`，说明 key 已经读到了，但 ROS 进程所在环境无法访问 `https://api.openai.com`。需要先恢复主机/容器出站网络，或配置代理：

```bash
export HTTPS_PROXY=http://127.0.0.1:7890
export HTTP_PROXY=http://127.0.0.1:7890
ros2 launch navflex_instruction_server instruction_web.launch.py llm_enabled:=true
```

也可以显式传给网页节点：

```bash
ros2 launch navflex_instruction_server instruction_web.launch.py \
  llm_enabled:=true openai_proxy:=http://127.0.0.1:7890
```

再次检查：

```bash
curl http://localhost:8080/api/llm_status
```

其中 `proxy_present` 应为 `true`。

如果你的启动方式不会继承环境变量，可以临时用参数指定变量名或 key：

```bash
ros2 launch navflex_instruction_server instruction_web.launch.py \
  openai_api_key_env:=OPENAI_API_KEY
```

也支持 `openai_api_key:=...`，但不建议长期这样用，因为命令行参数可能被 `ps` 看到。

如果只想让对话解析但不自动执行：

```bash
ros2 launch navflex_instruction_server instruction_web.launch.py \
  auto_execute_chat:=false
```

可配置参数：

- `global_frame`，默认 `map`
- `planner_id`，默认 `GridBased`
- `controller_id`，默认 `FollowPath`
- `goal_checker_id`，默认 `general_goal_checker`
- `behavior_id`，默认 `cmd_behavior`
- `action_timeout`，默认 `60.0`
- `feedback_publish_period`，默认 `0.5`，实时 feedback 发布节流周期，单位秒
- `named_locations`，字符串数组，格式 `name:x,y[,yaw_rad]`
- 网页节点参数：`llm_enabled`、`llm_backend`、`llm_model`、`codex_command`、`codex_args`、`openai_api_url`、`openai_api_key_env`、`openai_api_key`、`openai_proxy_env`、`openai_proxy`、`llm_timeout`、`auto_execute_chat`

## 构建

```bash
colcon build --symlink-install --packages-select navflex_instruction_server
source install/setup.bash
```

如果在普通构建和 `--symlink-install` 之间切换，可能出现
`ament_cmake_python_symlink_navflex_instruction_server` 无法创建符号链接的问题。固定使用一种构建模式；需要切换时先清理该包的旧产物：

```bash
rm -rf build/navflex_instruction_server install/navflex_instruction_server log/latest_build/navflex_instruction_server*
colcon build --symlink-install --packages-select navflex_instruction_server
```

## 设计边界

这个包是“指令路由层”：负责抽象文本解析、能力发现和调用编排。真正的导航行为仍由 `navflex_costmap_nav`、`navflex_cmdbehavior` 和 Nav2 插件完成。
