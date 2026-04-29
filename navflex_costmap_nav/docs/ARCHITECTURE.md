# navflex_costmap_nav — 架构文档

## 顶层结构

```
CostmapNavNode  (nav2_util::LifecycleNode)
│
├── Costmap2DROS "global_costmap"  ─── NodeThread
├── Costmap2DROS "local_costmap"   ─── NodeThread
│
├── PlannerCostmapServer           ─── NodeThread
│     └── PlannerAction ──► PlannerExecution ──► nav2_core::GlobalPlanner
│
├── ControllerCostmapServer        ─── NodeThread
│     └── ControllerAction ──► ControllerExecution ──► nav2_core::Controller
│
└── BehaviorCostmapServer          ─── NodeThread
      └── BehaviorAction ──► BehaviorExecution ──► nav2_core::Behavior
```

`CostmapNavNode` 自身作为整体的生命周期入口，通过 `lifecycle_manager` 驱动
configure → activate → deactivate → cleanup 各阶段，统一协调各子组件。

---

## 线程架构

```
进程主线程 (MultiThreadedExecutor, 4 threads)
  │
  ├─ global_costmap NodeThread   — 定时更新全局代价地图
  ├─ local_costmap NodeThread    — 定时更新局部代价地图
  ├─ planner_server NodeThread   — spin navflex_planner_server 节点
  ├─ controller_server NodeThread— spin navflex_controller_server 节点
  └─ behavior_server NodeThread  — spin navflex_behavior_server 节点
       │
       └─ 每个 Server 内部：
            action_executor_thread_ (SingleThreadedExecutor)
              └─ 仅包含 action_cb_group_（MutuallyExclusive, false）
                   负责 execute 回调，与主节点 executor 完全隔离
```

### 为何使用独立 action executor

`rclcpp_action` 的 goal/cancel 服务响应 和 execute 回调 若共享同一 `MutuallyExclusive`
callback group，execute 长跑时会阻塞 goal response，导致
`"Failed to send goal response (timeout)"` 告警。

仿照 Nav2 `SimpleActionServer(spin_thread=true)` 方案：
- `action_cb_group_` 以 `false`（不自动加入节点 executor）创建
- 单独的 `SingleThreadedExecutor` + `NodeThread` 专门 spin 该 cb group
- goal/cancel 服务由节点主 executor 处理，execute 回调由专属 executor 处理

---

## 核心类说明

### NavflexActionBase\<Action, Execution\>

位于 `include/navflex_base/navflex_action_base.hpp`。

模板基类，管理最多 256 个并发执行槽（`ConcurrencySlot`，slot ID 0–255）。

```
NavflexActionBase<Action, Execution>
  ├─ start(goal_handle, execution_ptr)
  │     1. 检查 goal 是否已在 canceling 状态 → 直接 canceled()
  │     2. 获取 slot_id（从 goal.concurrency_slot 或默认 0）
  │     3. cleanupSlot()：取消旧 execution，join 旧线程（阻塞）
  │     4. setupAndStartSlot()：启动新线程执行 run()
  │
  ├─ cancel(goal_handle)        非阻塞，设置 execution.cancel_ 标志
  ├─ cancelAll()                阻塞，等待所有槽位线程退出
  │
  └─ run() [在 execution 线程中]
        ├─ preRun()
        ├─ runImpl(goal_handle, execution)   ← 纯虚，子类实现
        └─ postRun()
```

**ConcurrencySlot 成员：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `execution` | `Execution::Ptr` | 执行对象（含插件引用） |
| `thread_ptr` | `unique_ptr<thread>` | 执行线程 |
| `goal_handle` | `GoalHandlePtr` | 与客户端通信的 handle |
| `in_use` | `bool` | 槽位占用标志 |

**阻塞特性：**

| 操作 | 阻塞 |
|------|------|
| `start()`（替换旧槽位时）| 是，join 旧线程 |
| `cancel()` | 否 |
| `cancelAll()` / 析构 | 是 |

---

### NavflexExecutionBase

位于 `include/navflex_base/navflex_execution_base.h`。

所有 Execution 类的基类，提供：
- `start()` / `stop()` / `join()` — 线程生命周期
- `cancel_` 原子标志 — 供 `run()` 循环检查
- `outcome_` / `message_` — 执行结果
- `condition_` + `mutex_` — 状态变更通知（`waitForStateUpdate()`）

---

### 三对 Action / Execution

| Server | Action | Execution | 使用插件接口 |
|--------|--------|-----------|--------------|
| PlannerCostmapServer | PlannerAction | PlannerExecution | `nav2_core::GlobalPlanner` |
| ControllerCostmapServer | ControllerAction | ControllerExecution | `nav2_core::Controller` |
| BehaviorCostmapServer | BehaviorAction | BehaviorExecution | `nav2_core::Behavior` |

**BehaviorExecution 状态机：**

```
INITIALIZED → STARTED → RECOVERING → RECOVERY_DONE
                                   ↘ CANCELED
                          STOPPED / INTERNAL_ERROR（异常路径）
```

---

## action server 完整调用链（以 Behavior 为例）

```
ros2 action send_goal /behavior_action DummyBehavior ...
  │
  ▼
BehaviorCostmapServer::handleGoalDummyBehavior()   — ACCEPT_AND_EXECUTE
  │
  ▼  (action_executor_thread_ 内)
BehaviorCostmapServer::callActionDummyBehavior()
  ├─ 查找 behaviors_ map，找不到 → abort(INVALID_PLUGIN)
  ├─ newBehaviorExecution(behavior_name)
  ├─ execution->setCommand(goal->command.data)
  └─ behavior_action_->start(goal_handle, execution)
       │
       ▼  NavflexActionBase::start()
       cleanupSlot(0) → setupAndStartSlot(0, ...)
         └─ new thread → BehaviorAction::run()
              └─ BehaviorAction::runImpl(goal_handle, execution)
                   ├─ execution->start()  → BehaviorExecution::run()
                   │     └─ plugin->runBehavior(command) [在 execution 线程]
                   ├─ waitForStateUpdate() 轮询状态
                   └─ goal_handle->succeed/abort/canceled(result)
```

---

## 结果码（DummyBehavior）

来自 `nav2_msgs::action::DummyBehavior::Result`：

| 常量 | 值 | 含义 |
|------|----|------|
| `SUCCESS` | 0 | 成功 |
| `FAILURE` | 150 | 插件返回失败 |
| `CANCELED` | 151 | 正常取消 |
| `PAT_EXCEEDED` | 152 | 超时 |
| `TF_ERROR` | 153 | 坐标变换失败 |
| `NOT_INITIALIZED` | 154 | 未初始化 |
| `INVALID_PLUGIN` | 155 | 插件名不存在 |
| `INTERNAL_ERROR` | 156 | 内部异常 |
| `STOPPED` | 157 | 外部停止 |

---

## 生命周期回调

各 Server 在生命周期各阶段的职责：

| 回调 | 操作 |
|------|------|
| `on_configure` | 加载插件、创建 action executor + NodeThread、创建 action server |
| `on_activate` | 激活插件、激活 publisher |
| `on_deactivate` | `cancelAll()`、停止 publisher |
| `on_cleanup` | `executor_thread_.reset()` → `executor_.reset()` → 清理插件 |

> cleanup 中必须先 reset executor_thread_（停止 spin），再 reset executor_，
> 最后 reset action_server_，否则 action server 析构时可能访问已释放的 executor。

