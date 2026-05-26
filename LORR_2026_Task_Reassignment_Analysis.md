# LORR 2026 MAPF 竞赛任务重分配策略深度分析

## 1. LORR 2026 竞赛实际场景特征

### 1.1 持续规划 (Continuous Planning) 架构
```
时间线:
┌──────────────────────────────────────────────────────────────────┐
│ Planner Thread (后台运行)                                         │
│   ├─ compute(): 规划路径 + 调度任务                               │
│   └─ 每次规划覆盖 min_comm_time 步 (lookahead 窗口)                │
├──────────────────────────────────────────────────────────────────┤
│ Executor Thread (前台运行)                                        │
│   ├─ move(): 执行规划的动作                                       │
│   ├─ update_tasks(): 检查任务完成 + 揭示新任务                    │
│   └─ 每次移动 min_comm_time 步后同步                              │
└──────────────────────────────────────────────────────────────────┘
```

**关键特性**:
- Planner 永远在后台运行，规划未来 min_comm_time 步
- 当 planner 返回时，executor 同步新计划，然后启动新规划
- 这意味着 **任何时刻 agent 都有对未来任务的"承诺"**

### 1.2 任务揭示机制
```cpp
// TaskManager::reveal_tasks
while (ongoing_tasks.size() < num_tasks_reveal) {
    // 持续补充任务池
}
// num_tasks_reveal = numTasksReveal * num_of_agents
```

**典型配置 (Paris_200)**:
- teamSize = 200 agents
- numTasksReveal = 1
- num_tasks_reveal = 1 × 200 = 200 个任务始终在池中

**结果**: 任务池大小 ≈ 车辆数量 → **完全占用状态**

### 1.3 任务状态转换
```
reveal_tasks()                    check_finished_tasks()
   │                                     │
   ▼                                     ▼
[new_tasks] ──────────────────► [ongoing_tasks] ──► 完成
                                   │
                                   ├─ idx_next_loc == 0 (新任务)
                                   ├─ idx_next_loc > 0 && < size (执行中)
                                   └─ idx_next_loc == size (完成)
```

**关键状态字段**:
- `task.agent_assigned`: 当前执行任务的 agent (-1 表示无)
- `task.idx_next_loc`: 下一个要访问的位置索引
- `task.locations`: 任务路径点序列 [start, loc1, loc2, ..., goal]

---

## 2. 当前策略失效根本原因分析

### 2.1 问题诊断图

```
┌─────────────────────────────────────────────────────────────────────┐
│                    失效机制链条                                      │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  num_tasks_reveal = numTasksReveal * num_of_agents                  │
│           │                                                         │
│           ▼                                                         │
│  ┌─────────────────────┐                                           │
│  │   任务池满载状态      │  (200 agents, 200 tasks)                   │
│  └─────────────────────┘                                           │
│           │                                                         │
│           ▼                                                         │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ free_agents 永远是空的 ────────────────────────────────────▶│    │
│  │   └─ H27 (效率切换) 失效                                    │    │
│  └─────────────────────────────────────────────────────────────┘    │
│           │                                                         │
│           ▼                                                         │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │ free_tasks 永远是空的 ────────────────────────────────────▶│    │
│  │   └─ H23 (年龄惩罚) 失效                                    │    │
│  │   └─ H31 (等待切换) 失效 (无法找到替换任务)                 │    │
│  │   └─ H32 (互抑制) 失效 (无法找到替换任务)                   │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 各策略具体失效分析

#### H23 年龄惩罚 (Age Penalty)
```cpp
// scheduler.cpp:262
for (int free_task_id : free_tasks) {  // ← free_tasks 为空，循环不执行
    dist += task_age * penalty;
}
```
**问题**: 年龄惩罚只在任务位于 `free_tasks` 时生效。但 `free_tasks` 在 lookahead + 持续规划模式下**永远为空**。

#### H27 效率切换 (Efficiency Switching)
```cpp
// scheduler.cpp:297
if (do_reassess && !free_agents.empty() && !agent_assigned_task.empty()) {
    // 只有 free_agents 不为空时才执行
}
```
**问题**: 当 `num_tasks_reveal = num_of_agents` 时，所有 agent 都有任务，`free_agents` 永远为空。

#### H31 等待切换 (Wait-time Switching)
```cpp
// scheduler.cpp:260-276
for (int free_task_id : free_tasks) {  // ← free_tasks 为空
    // 找不到替换任务，best_free_task 永远是 -1
}
```
**问题**: 即使检测到 agent 等待 (consecutive_wait > 5)，也无法找到替代任务。

#### H32 互抑制 (Mutual Inhibition)
```cpp
// scheduler.cpp:391-404
for (int free_task_id : free_tasks) {  // ← free_tasks 为空
    // 无法找到非冲突的替换任务
}
```
**问题**: 70% 路径重叠阈值 + `free_tasks` 为空 = 双重失效。

### 2.3 核心矛盾

```
┌────────────────────────────────────────────────────────────────┐
│                      根本矛盾                                   │
├────────────────────────────────────────────────────────────────┤
│                                                                 │
│  传统重分配思路:                                                │
│    "发现问题任务" → "放入 free_tasks" → "重新分配"              │
│                                                                 │
│  实际系统行为:                                                  │
│    "问题任务" → "仍然是 assigned" → "无法重新分配"              │
│                                                                 │
│  原因:                                                          │
│    - 任务池始终满载 (num_tasks_reveal ≈ num_of_agents)          │
│    - lookahead 机制让 planner 提前分配未来任务                   │
│    - 任务一旦 assigned，scheduler 无法主动回收                  │
│                                                                 │
└────────────────────────────────────────────────────────────────┘
```

---

## 3. 全新任务重分配框架

### 3.1 核心思想转变

**传统思路 (被动式)**:
```
等待问题发生 → 放入池中 → 重新分配
```

**新思路 (主动式)**:
```
持续监控 → 预测性评估 → 主动交换/调整
```

### 3.2 新框架: PACD (Predictive Assessment & Cooperative Decision)

```
┌─────────────────────────────────────────────────────────────────────┐
│                    PACD 框架                                         │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐          │
│  │  PREDICTIVE  │───▶│  ASSESSMENT  │───▶│  DECISION    │          │
│  │  PHASE       │    │  PHASE       │    │  PHASE       │          │
│  │              │    │              │    │              │          │
│  │ 预测未来效率 │    │ 计算效率/    │    │ 决定是否交换 │          │
│  │ 预测冲突概率 │    │ 冲突/距离    │    │ /调整任务    │          │
│  └──────────────┘    └──────────────┘    └──────────────┘          │
│          │                  │                  │                    │
│          ▼                  ▼                  ▼                    │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │                    ACTION POOL                              │   │
│  │  1. 任务交换 (Task Swap)                                    │   │
│  │  2. 任务转移 (Task Transfer)                                 │   │
│  │  3. 路径重规划 (Replan Only)                                 │   │
│  │  4. 放弃任务 (Task Abandonment)                              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 3.3 关键机制

#### 3.3.1 任务交换 (Task Swap)
```
状态: Agent A → Task T1, Agent B → Task T2

交换条件:
  1. T1 和 T2 都处于执行早期 (idx_next_loc < locations.size() / 3)
  2. A 离 T2 的起点比离 T1 的起点更近
  3. B 离 T1 的起点比离 T2 的起点更近
  4. 交换后总体效率提升

交换后:
  Agent A → Task T2
  Agent B → Task T1
```

#### 3.3.2 任务转移 (Task Transfer)
```
状态: Agent A → Task T1 (效率低, 距离远)

转移条件:
  1. A 执行 T1 的效率 < 阈值
  2. 存在 Agent B 有更好的 T1 匹配度
  3. B 当前任务的效率 > 阈值 (确保 B 有余力)

转移后:
  Agent A → 接受 B 的任务或新任务
  Agent B → 接管 T1
  T1 的 idx_next_loc 保持不变 (继续从当前位置执行)
```

#### 3.3.3 预测性重分配 (Predictive Reallocation)
```
不是等任务失败后重分配，而是:

1. 预测任务完成时间
2. 预测未来某个时刻的"空闲车辆"
3. 在任务完成前预先分配下一个任务
4. 但保留一定的灵活性用于调整

关键: 在规划时考虑"软重分配"的可能性
```

---

## 4. 具体可落地算法设计

### 4.1 算法 1: 效率引导的任务交换 (EGTS)

```cpp
// 效率引导的任务交换算法
// 位置: scheduler.cpp (新函数)

void efficient_task_swap(SharedEnvironment* env, vector<int>& proposed_schedule, int current_time) {
    const double EFFICIENCY_THRESHOLD = 0.3;      // 低效率阈值
    const double SWAP_GAIN_THRESHOLD = 0.2;      // 交换收益阈值
    const int TASK_EARLY_THRESHOLD = 3;           // 任务早期判断
    const int SWAP_COOLDOWN = 50;                 // 交换冷却期
    
    static unordered_map<int, int> last_swap_time;
    
    // 1. 找出所有低效率的 agent-task 对
    vector<pair<int, int>> low_efficiency_pairs;
    for (auto& at : agent_assigned_task) {
        int a = at.first;
        int t = at.second;
        if (t < 0) continue;
        
        // 检查冷却期
        auto swap_it = last_swap_time.find(a);
        if (swap_it != last_swap_time.end() && 
            current_time - swap_it->second < SWAP_COOLDOWN) continue;
        
        double eff = get_agent_efficiency(a, t, env, current_time);
        if (eff < EFFICIENCY_THRESHOLD) {
            low_efficiency_pairs.push_back({a, t});
        }
    }
    
    // 2. 尝试配对交换
    for (size_t i = 0; i < low_efficiency_pairs.size(); i++) {
        int a1 = low_efficiency_pairs[i].first;
        int t1 = low_efficiency_pairs[i].second;
        
        for (size_t j = i + 1; j < low_efficiency_pairs.size(); j++) {
            int a2 = low_efficiency_pairs[j].first;
            int t2 = low_efficiency_pairs[j].second;
            
            // 检查双方是否都在冷却期
            auto swap_it1 = last_swap_time.find(a1);
            auto swap_it2 = last_swap_time.find(a2);
            if ((swap_it1 != last_swap_time.end() && 
                 current_time - swap_it1->second < SWAP_COOLDOWN) ||
                (swap_it2 != last_swap_time.end() && 
                 current_time - swap_it2->second < SWAP_COOLDOWN)) continue;
            
            // 3. 检查任务是否处于早期
            Task& task1 = env->task_pool[t1];
            Task& task2 = env->task_pool[t2];
            if (task1.idx_next_loc > TASK_EARLY_THRESHOLD || 
                task2.idx_next_loc > TASK_EARLY_THRESHOLD) continue;
            
            // 4. 计算交换收益
            int a1_loc = env->curr_states[a1].location;
            int a2_loc = env->curr_states[a2].location;
            
            int t1_start = task1.locations[0];
            int t1_goal = task1.locations.back();
            int t2_start = task2.locations[0];
            int t2_goal = task2.locations.back();
            
            // A1 到 T2 的距离 vs A1 到 T1 的距离
            int a1_to_t1 = DefaultPlanner::get_h(env, a1_loc, t1_start);
            int a1_to_t2 = DefaultPlanner::get_h(env, a1_loc, t2_start);
            int a2_to_t1 = DefaultPlanner::get_h(env, a2_loc, t1_start);
            int a2_to_t2 = DefaultPlanner::get_h(env, a2_loc, t2_start);
            
            // 交换后的总距离变化
            int old_total = a1_to_t1 + a2_to_t2;
            int new_total = a1_to_t2 + a2_to_t1;
            double swap_gain = (old_total - new_total) / (double)old_total;
            
            if (swap_gain > SWAP_GAIN_THRESHOLD) {
                // 执行交换
                proposed_schedule[a1] = t2;
                proposed_schedule[a2] = t1;
                
                // 更新状态
                agent_assigned_task[a1] = t2;
                agent_assigned_task[a2] = t1;
                task_start_time[t2] = task_start_time[t1];
                task_start_time[t1] = current_time;
                
                last_swap_time[a1] = current_time;
                last_swap_time[a2] = current_time;
                
                // 注意: 不需要修改 idx_next_loc，因为任务继续从当前位置执行
            }
        }
    }
}
```

### 4.2 算法 2: 预测性任务完成 (Predictive Completion)

```cpp
// 预测性任务完成 + 预分配
// 位置: scheduler.cpp (新函数)

void predictive_task_planning(SharedEnvironment* env, vector<int>& proposed_schedule, 
                             int current_time, int lookahead_horizon) {
    // lookahead_horizon: 预测的时间范围 (通常是 min_comm_time)
    
    struct PredictedCompletion {
        int agent_id;
        int task_id;
        int completion_time;
        int remaining_distance;
    };
    
    vector<PredictedCompletion> predictions;
    
    // 1. 预测每个 agent 的任务完成时间
    for (auto& at : agent_assigned_task) {
        int a = at.first;
        int t = at.second;
        if (t < 0) continue;
        
        Task& task = env->task_pool[t];
        int agent_loc = env->curr_states[a].location;
        int goal_loc = task.locations.back();
        
        int remaining = DefaultPlanner::get_h(env, agent_loc, goal_loc);
        int completion_time = current_time + remaining;
        
        if (completion_time < current_time + lookahead_horizon) {
            predictions.push_back({a, t, completion_time, remaining});
        }
    }
    
    // 2. 按完成时间排序
    sort(predictions.begin(), predictions.end(), 
         [](const PredictedCompletion& p1, const PredictedCompletion& p2) {
             return p1.completion_time < p2.completion_time;
         });
    
    // 3. 对于即将完成的 agent，预分配下一个任务
    // 但不锁定，允许后续调整
    for (auto& pred : predictions) {
        int a = pred.agent_id;
        int curr_task = pred.task_id;
        
        // 跳过已经在预分配冷却期的 agent
        auto switch_it = agent_last_switch_time.find(a);
        if (switch_it != agent_last_switch_time.end() && 
            current_time - switch_it->second < TASK_SWITCH_COOLDOWN) continue;
        
        // 计算当前任务的效率
        double curr_eff = get_agent_efficiency(a, curr_task, env, current_time);
        
        // 如果当前效率很低，考虑预切换
        if (curr_eff < 0.3) {
            // 寻找更好的预分配任务
            int best_task = -1;
            double best_score = -1.0;
            
            // 从所有任务中找 (不仅仅是 free_tasks)
            for (auto& tp : env->task_pool) {
                int t = tp.first;
                if (t == curr_task) continue;  // 跳过当前任务
                
                // 检查任务是否已被分配
                bool already_assigned = false;
                for (auto& at : agent_assigned_task) {
                    if (at.second == t) {
                        already_assigned = true;
                        break;
                    }
                }
                if (already_assigned) continue;
                
                // 计算评分
                int a_loc = env->curr_states[a].location;
                int t_start = tp.second.locations[0];
                int t_goal = tp.second.locations.back();
                int dist = DefaultPlanner::get_h(env, a_loc, t_start) + 
                           DefaultPlanner::get_h(env, t_start, t_goal);
                
                double score = 1000.0 / (dist + 1);
                
                // 考虑任务年龄
                auto age_it = task_age_map.find(t);
                if (age_it != task_age_map.end()) {
                    int age = current_time - age_it->second;
                    if (age > 50) score *= 1.2;  // 老任务优先
                }
                
                if (score > best_score) {
                    best_score = score;
                    best_task = t;
                }
            }
            
            if (best_task != -1 && best_score > (1000.0 / (pred.remaining_distance + 1)) * 1.5) {
                // 预分配新任务
                proposed_schedule[a] = best_task;
                agent_assigned_task[a] = best_task;
                task_start_time[best_task] = current_time;
                agent_last_switch_time[a] = current_time;
                
                // 注意: 这不是强制重分配，而是预规划
                // 实际执行时，如果 planner 发现路径冲突，会自然拒绝
            }
        }
    }
}
```

### 4.3 算法 3: 协作式任务转移 (Collaborative Transfer)

```cpp
// 协作式任务转移
// 当一个 agent 效率低且距离远时，让最近的空闲 agent 接管

void collaborative_task_transfer(SharedEnvironment* env, vector<int>& proposed_schedule,
                                 int current_time) {
    const double LOW_EFFICIENCY_THRESHOLD = 0.25;
    const int HIGH_DISTANCE_THRESHOLD = 100;
    const int TRANSFER_COOLDOWN = 80;
    
    static unordered_map<int, int> last_transfer_time;
    
    // 1. 找出"困境"任务 (低效率 + 远距离)
    vector<tuple<int, int, double, int>> distressed_tasks;  // (agent, task, eff, dist)
    
    for (auto& at : agent_assigned_task) {
        int a = at.first;
        int t = at.second;
        if (t < 0) continue;
        
        double eff = get_agent_efficiency(a, t, env, current_time);
        if (eff > LOW_EFFICIENCY_THRESHOLD) continue;
        
        int agent_loc = env->curr_states[a].location;
        int goal_loc = env->task_pool[t].locations.back();
        int dist = DefaultPlanner::get_h(env, agent_loc, goal_loc);
        
        if (dist > HIGH_DISTANCE_THRESHOLD) {
            distressed_tasks.push_back({a, t, eff, dist});
        }
    }
    
    // 2. 按效率排序，最低的优先处理
    sort(distressed_tasks.begin(), distressed_tasks.end(),
         [](auto& a, auto& b) { return get<2>(a) < get<2>(b); });
    
    // 3. 尝试转移给最近的 agent
    for (auto& dt : distressed_tasks) {
        int src_agent = get<0>(dt);
        int task_id = get<1>(dt);
        
        // 检查冷却期
        auto lt_it = last_transfer_time.find(src_agent);
        if (lt_it != last_transfer_time.end() && 
            current_time - lt_it->second < TRANSFER_COOLDOWN) continue;
        
        // 找到最近的"有余裕"的 agent
        int best_agent = -1;
        int best_distance = INT_MAX;
        
        int task_curr_loc = env->task_pool[task_id].locations[0]; // 任务起点
        
        for (int a = 0; a < env->num_of_agents; a++) {
            if (a == src_agent) continue;
            
            // 检查是否是 assigned
            if (agent_assigned_task.find(a) == agent_assigned_task.end()) continue;
            
            int a_task = agent_assigned_task[a];
            if (a_task < 0) continue;
            
            // 检查目标 agent 的效率
            double a_eff = get_agent_efficiency(a, a_task, env, current_time);
            if (a_eff < 0.4) continue;  // 目标 agent 自己也效率不高
            
            // 计算距离
            int a_loc = env->curr_states[a].location;
            int dist = DefaultPlanner::get_h(env, a_loc, task_curr_loc);
            
            if (dist < best_distance) {
                best_distance = dist;
                best_agent = a;
            }
        }
        
        if (best_agent != -1 && best_distance < HIGH_DISTANCE_THRESHOLD) {
            // 执行转移
            // 原 agent 放弃任务 (设为 -1，稍后分配新任务)
            proposed_schedule[src_agent] = -1;
            agent_assigned_task.erase(src_agent);
            agent_consecutive_wait[src_agent] = 0;
            
            // 目标 agent 接管任务
            // 注意: 任务继续从当前位置执行，不需要改变 idx_next_loc
            proposed_schedule[best_agent] = task_id;
            agent_assigned_task[best_agent] = task_id;
            // task_start_time 不变，任务继续
            
            last_transfer_time[src_agent] = current_time;
            agent_last_switch_time[best_agent] = current_time;
        }
    }
}
```

### 4.4 算法 4: 软强制重分配 (Soft Forced Reallocation)

```cpp
// 软强制重分配 - 针对长期低效率任务
// 不同于 H23 的 age 惩罚，这是在重分配触发后才强制

void soft_forced_reallocation(SharedEnvironment* env, vector<int>& proposed_schedule,
                              int current_time) {
    const int LONG_TERM_THRESHOLD = 150;  // 长期低效率阈值
    const double LONG_TERM_EFFICIENCY = 0.2;
    const int COOLDOWN = 100;
    
    static unordered_map<int, int> last_force_time;
    
    for (auto& at : agent_assigned_task) {
        int a = at.first;
        int t = at.second;
        if (t < 0) continue;
        
        // 检查冷却期
        auto lf_it = last_force_time.find(a);
        if (lf_it != last_force_time.end() && 
            current_time - lf_it->second < COOLDOWN) continue;
        
        // 计算效率和持续时间
        double eff = get_agent_efficiency(a, t, env, current_time);
        
        auto start_it = task_start_time.find(t);
        if (start_it == task_start_time.end()) continue;
        int duration = current_time - start_it->second;
        
        // 条件: 效率低 + 持续时间长
        if (eff < LONG_TERM_EFFICIENCY && duration > LONG_TERM_THRESHOLD) {
            // 强制重分配
            proposed_schedule[a] = -1;
            agent_assigned_task.erase(a);
            task_start_time.erase(t);
            
            // 将任务标记为"可重新分配"
            // 注意: 任务仍在 ongoing_tasks 中，只是没有 agent 执行
            last_force_time[a] = current_time;
        }
    }
    
    // 在正常分配阶段，这些被放弃的任务会回到 free_tasks
    // 然后被正常分配
}
```

---

## 5. 各策略适用条件与局限性分析

### 5.1 策略总览表

| 策略 | 触发条件 | 作用范围 | 局限性 |
|------|----------|----------|--------|
| EGTS (任务交换) | 两任务效率都低 + 交换收益>20% | 配对交换 | 需要双方都在早期 |
| Predictive Planning | 任务即将完成 + 当前效率低 | 预分配 | 预测不准确时效果差 |
| Collaborative Transfer | 一任务困境 + 附近有余裕agent | 点对点转移 | 需要余力agent存在 |
| Soft Forced Realloc | 长期低效率 (>150步, <0.2效率) | 单点重置 | 可能打乱规划 |
| 年龄惩罚 (H23) | 任务在free_tasks超过阈值 | free_tasks | 在持续规划下失效 |
| 效率切换 (H27) | free_agents存在 + 效率低 | free_agents | 在满载场景失效 |
| 等待切换 (H31) | 连续等待>5步 | assigned agents | 需要有替代任务 |
| 互抑制 (H32) | 路径重叠>70% + 效率低 | 配对调整 | 阈值难以达到 |

### 5.2 详细分析

#### EGTS (效率引导任务交换)
**适用条件**:
- 任务刚开始执行 (idx_next_loc < 3)
- 两个 agent 的任务可以互相优化
- 交换后总体距离减少 > 20%

**局限性**:
- 任务已经开始执行后期时，交换收益降低
- 计算交换收益需要启发式估计，可能不准确
- 交换可能与 planner 的路径规划产生冲突

**效果预期**:
- 在多车近距离场景效果好 (如 warehouse)
- 在稀疏场景效果一般 (如 Paris)

#### Predictive Planning (预测性规划)
**适用条件**:
- lookahead 窗口较大 (>20 步)
- 任务完成时间可准确预测
- 系统支持"软分配"

**局限性**:
- 预测基于启发式，可能误差较大
- 预分配可能与实际 planner 冲突
- 计算开销增加

**效果预期**:
- 在规整地图效果好 ( Manhattan 距离准确)
- 在有障碍物地图效果一般

#### Collaborative Transfer (协作转移)
**适用条件**:
- 存在"余裕"agent (效率 > 0.4)
- 任务可以快速转移 (距离 < 阈值)
- 转移后总体效率提升

**局限性**:
- 高密度场景下很少有余裕 agent
- 转移可能造成目标 agent 也陷入困境
- 需要额外的同步机制

**效果预期**:
- 在任务负载不均场景效果好
- 在所有 agent 都满载时失效

#### Soft Forced Reallocation (软强制重分配)
**适用条件**:
- 任务长期低效率 (>150 步)
- 系统允许任务暂时"无人认领"
- 有足够的自由任务可以重新分配

**局限性**:
- 可能打乱正在执行的规划
- 长期低效率判断阈值难以设定
- 可能造成任务饥饿

**效果预期**:
- 作为"兜底"策略，在其他策略失效时使用
- 效果取决于系统负载情况

### 5.3 组合策略建议

```
优先级顺序:

1. Predictive Planning (预测性规划)
   - 主动预防，而非被动应对
   - 在规划阶段就考虑重分配可能性

2. EGTS (任务交换)
   - 处理刚分配就发现的效率问题
   - 适用于任务早期

3. Collaborative Transfer (协作转移)
   - 处理单点严重困境
   - 需要有余力 agent

4. Soft Forced Reallocation (软强制)
   - 最后兜底
   - 处理长期无法解决的低效任务
```

---

## 6. 实现建议

### 6.1 关键代码位置

```cpp
// scheduler.cpp 新增
void efficient_task_swap(SharedEnvironment* env, vector<int>& proposed_schedule, int current_time);
void predictive_task_planning(SharedEnvironment* env, vector<int>& proposed_schedule, int current_time, int lookahead);
void collaborative_task_transfer(SharedEnvironment* env, vector<int>& proposed_schedule, int current_time);
void soft_forced_reallocation(SharedEnvironment* env, vector<int>& proposed_schedule, int current_time);

// 在 schedule_plan() 中调用
// 顺序很重要: 预测 -> 交换 -> 转移 -> 强制
```

### 6.2 关键参数调优建议

| 参数 | 当前值 | 建议范围 | 调整依据 |
|------|--------|----------|----------|
| EFFICIENCY_THRESHOLD | 0.3 | 0.2-0.4 | 地图密度 |
| SWAP_GAIN_THRESHOLD | 0.2 | 0.15-0.3 | 任务长度 |
| TASK_EARLY_THRESHOLD | 3 | 2-5 | 任务复杂度 |
| SWAP_COOLDOWN | 50 | 40-80 | 系统响应速度 |
| LONG_TERM_THRESHOLD | 150 | 100-200 | 任务完成时间 |
| TRANSFER_COOLDOWN | 80 | 60-120 | agent 数量 |

### 6.3 测试建议

1. **单元测试**: 每个算法独立验证
2. **集成测试**: 验证算法组合效果
3. **压力测试**: 高密度场景 (numTasksReveal = 2-3)
4. **对比测试**: 新策略 vs H23/H27/H31/H32

---

## 7. 结论

LORR 2026 的持续规划 + lookahead 机制造成了"任务池满载"这一独特场景，导致传统重分配策略 (基于 free_tasks/free_agents) 全部失效。

**核心解决思路**:
1. 从"被动检测-重新分配"转向"主动预测-预防性调整"
2. 不依赖 free_tasks 池，而是直接对 assigned 任务进行交换/转移
3. 利用 PACD 框架: Predictive Assessment → Cooperative Decision

**推荐实现顺序**:
1. 先实现 EGTS (最简单，风险低)
2. 再实现 Predictive Planning (最主动)
3. 最后加 Collaborative Transfer 和 Soft Forced 作为补充
