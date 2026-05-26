# EGTS Waypoint-Based Progress Tracking: Algorithm Design Document

## Project
MAPF-Research `/mnt/f/MAPF/MAPF-Research`

## Files Analyzed
- `default_planner/scheduler.cpp` (602 lines)
- `inc/Tasks.h` (Task struct)
- `inc/States.h` (State struct)

---

## Section 1: Problem Analysis

### 1.1 Current `get_agent_efficiency()` Logic (lines 114-124)

```cpp
double get_agent_efficiency(int a, int curr_task_id, SharedEnvironment* env, int current_time) {
    if (curr_task_id < 0) return 0.0;
    auto start_it = task_start_time.find(curr_task_id);
    int time_elapsed = (start_it != task_start_time.end()) ? (current_time - start_it->second) : 1;
    int agent_loc = env->curr_states[a].location;
    int goal_loc = env->task_pool[curr_task_id].locations.back();  // <-- PROBLEM
    int remaining = DefaultPlanner::get_h(env, agent_loc, goal_loc);
    int initial_dist = DefaultPlanner::get_h(env, env->task_pool[curr_task_id].locations[0], goal_loc);
    int progress = initial_dist - remaining;
    return (time_elapsed > 0) ? ((double)progress / (double)time_elapsed) : 0.0;
}
```

**What's Wrong:**
- Uses `goal_loc = locations.back()` (final destination) for progress measurement
- When agent is navigating toward intermediate waypoints, `remaining` may not decrease even though the agent IS making progress
- Example: Task path `[p0, p1, p2, p3, p4]` with segment costs `[2, 2, 20, 2]`
  - Agent at p2, heading to p3 (which is 20 steps away due to obstacle)
  - Agent reaches p2 (intermediate waypoint) → `remaining` to goal = 20
  - Agent continues toward p3 → `remaining` = 20 (unchanged, still at p2 area)
  - Agent finally reaches p3 → `remaining` = 2
  - **Problem**: Agent made 2 steps of progress but `remaining` didn't decrease until reaching p3

### 1.2 Current `consecutive_wait` Logic (lines 331-351)

```cpp
for (int a = 0; a < env->num_of_agents; a++) {
    int task_id = env->curr_task_schedule[a];
    if (task_id < 0) { agent_consecutive_wait[a] = 0; continue; }
    int agent_loc = env->curr_states[a].location;
    int goal_loc = env->task_pool[task_id].locations.back();  // <-- PROBLEM
    int remaining = DefaultPlanner::get_h(env, agent_loc, goal_loc);
    auto prev_it = agent_prev_remaining.find(a);
    if (prev_it != agent_prev_remaining.end()) {
        if (remaining >= prev_it->second)
            agent_consecutive_wait[a]++;  // "no progress" detected
        else
            agent_consecutive_wait[a] = 0;
    }
    agent_prev_remaining[a] = remaining;
}
```

**What's Wrong:**
- Uses distance-to-goal to detect "no progress"
- When navigating multi-waypoint tasks, reaching intermediate waypoints doesn't reduce distance-to-goal
- False positive: Agent reaching intermediate waypoint but not yet closer to goal → counted as "waiting"
- False negative: Agent detours around obstacle but still moving toward goal (distance increases temporarily) → counted as "waiting"

### 1.3 Concrete Example of Failure

**Scenario**: Task revealed at t=0, path `[p0, p1, p2, p3]` with segment costs `[5, 5, 5, 5]`

```
Timeline (current_time in scheduler_call_count):
t=0:  Agent at p0, idx_next_loc=0, remaining_to_goal=15
t=1:  Agent moves to p1, idx_next_loc=1, remaining_to_goal=10 ✓ Progress made
t=2:  Agent moves to p2, idx_next_loc=2, remaining_to_goal=5  ✓ Progress made
t=3:  Agent stuck at p2 (traffic), idx_next_loc=2, remaining=5 (waiting)
t=4:  Agent still stuck, remaining=5 (waiting)
t=5:  Agent moves to p3, idx_next_loc=3, remaining=0 ✓ Done
```

**Current consecutive_wait tracking:**
- t=1: remaining went from 15→10, so counter = 0 (correct)
- t=2: remaining went from 10→5, so counter = 0 (correct)
- t=3: remaining stayed at 5, so counter = 1 (WRONG: agent reached intermediate waypoint!)
- t=4: remaining stayed at 5, so counter = 2 (WRONG)
- t=5: remaining went from 5→0, so counter = 0 (correct)

**Problem**: The agent was detected as "waiting" for 2 timesteps even though it successfully traversed from p1 to p2. The issue is that reaching p2 didn't reduce remaining distance to goal (5 remained the same).

### 1.4 Impact on EGTS (lines 126-213)

The `efficient_task_swap()` function uses `get_agent_efficiency()` to decide which agents to consider for swapping. With the flawed efficiency metric:
- Agents making real progress may be flagged as "low efficiency"
- The swap decision may target the wrong agents
- The EGTS thresholds (0.3 efficiency, -0.5 negative progress) become unreliable

---

## Section 2: Algorithm Design

### 2.1 Data Flow Overview

```
Task t_revealed=0, locations=[p0,p1,p2,p3,p4], segment_costs=[5,5,5,5,5]
Current time = 25 (simulation timestep)

compute_ideal_waypoint():
  time_elapsed = 25 - 0 = 25
  cumulative_costs = [0, 5, 10, 15, 20, 25]
  ideal_waypoint_idx = binary_search_first_gte(cumulative_costs, time_elapsed) = 5 (AT_END)
  → Agent should have completed the task by now

actual_waypoint = agent.idx_next_loc (e.g., 2)
deviation = 2 - 5 = -3 (agent is 3 waypoints BEHIND schedule)

If deviation < -THRESHOLD → ABNORMAL_DEVIATION (task assignment error or severe congestion)
If deviation >= -THRESHOLD → NORMAL_PROGRESS (traffic congestion is acceptable)
```

### 2.2 Core Functions

#### 2.2.1 `compute_segment_costs(Task& task, SharedEnvironment* env)`

Computes the cumulative cost along each segment of the task path.

```cpp
// Returns cumulative distance from start to each waypoint
// segment_costs[k] = distance from locations[k-1] to locations[k]
// cumulative_costs[0] = 0 (at start)
// cumulative_costs[1] = segment_costs[1] (cost to reach locations[1])
// cumulative_costs[2] = segment_costs[1] + segment_costs[2], etc.

std::vector<int> compute_segment_costs(const Task& task, SharedEnvironment* env) {
    std::vector<int> cumulative_costs;
    cumulative_costs.push_back(0);  // At locations[0], cost is 0
    
    int running_sum = 0;
    for (size_t k = 1; k < task.locations.size(); k++) {
        int segment_dist = DefaultPlanner::get_h(env, 
            task.locations[k-1], task.locations[k]);
        running_sum += segment_dist;
        cumulative_costs.push_back(running_sum);
    }
    return cumulative_costs;  // Size = locations.size(), cumulative_costs[0]=0
}
```

**Example**: Task with locations `[p0, p1, p2, p3]` and segment costs `[5, 5, 5, 5]`
```
cumulative_costs = [0, 5, 10, 15]
                  ↑  ↑  ↑  ↑
                p0  p1  p2  p3
```

#### 2.2.2 `compute_actual_progress_cost()` — 关键修正

**核心问题**：`idx_next_loc` 只是 agent 的"下一个目标"，agent 的实际位置可能在 `locations[idx_next_loc-1]` 到 `locations[idx_next_loc]` 之间的任何位置。

所以 `actual_progress_cost` 必须包含"当前段已行进距离"，否则会严重低估 progress：

```cpp
// agent 在去 idx_next_loc 的路上，还没到
// actual_progress 必须考虑这段走了多远
int compute_actual_progress_cost(int agent_id, const Task& task, SharedEnvironment* env) {
    int idx = task.idx_next_loc;
    int agent_loc = env->curr_states[agent_id].location;
    int next_loc = task.locations[idx];
    int prev_loc = (idx > 0) ? task.locations[idx - 1] : -1;
    
    // 已完成的所有完整段（从 0 到 idx-1）
    int completed_cost = 0;
    for (int k = 1; k < idx; k++) {
        completed_cost += DefaultPlanner::get_h(env, task.locations[k-1], task.locations[k]);
    }
    
    // 当前段（idx-1 → idx）已行进多少
    int segment_cost = 0;
    if (prev_loc >= 0) {
        segment_cost = DefaultPlanner::get_h(env, prev_loc, next_loc);
    }
    int dist_to_next = DefaultPlanner::get_h(env, agent_loc, next_loc);
    int traveled_in_segment = segment_cost - dist_to_next;  // 已经走了多远（可能是负数，如果 agent 还在 prev_loc 那边）
    
    // clamp：确保不会超过整个路径总长
    int total_cost = 0;
    for (size_t k = 1; k < task.locations.size(); k++) {
        total_cost += DefaultPlanner::get_h(env, task.locations[k-1], task.locations[k]);
    }
    
    int actual_progress = completed_cost + traveled_in_segment;
    if (actual_progress < 0) actual_progress = 0;
    if (actual_progress > total_cost) actual_progress = total_cost;
    return actual_progress;
}
```

**示例**：
```
locations=[A,B,C,D,E], segment_costs=[5,5,5,5,5], cumulative=[0,5,10,15,20,25]
total_cost = 25

Case: agent 从 A→B，已经走了 3 步（B 在 5 步外），idx_next_loc = 1
  completed_cost = 0 (还没有完整段)
  segment_cost = 5, dist_to_next = 2, traveled_in_segment = 3
  actual_progress = 0 + 3 = 3
  
Case: agent 刚到达 B，idx_next_loc = 2，agent_loc = B 位置
  completed_cost = 5 (A→B 完整)
  dist_to_next = get_h(B, C) = 5
  traveled_in_segment = 5 - 5 = 0
  actual_progress = 5 + 0 = 5
  // 下次 scheduler 调用时，executor 前进，idx_next_loc 会变成 2，然后 actual_progress 就会跳到 10
```

**注意**：这个函数在 scheduler 每次调用时计算。scheduler 调用间隔 ≈ window_size 个 simulation timestep，所以：
- Agent 在两次调用之间可能前进 1 到 window_size 步
- `idx_next_loc` 可能还没更新（executor 更新 idx_next_loc 的频率决定了这个延迟）
- 所以 **actual_progress 一定要加当前段已行进距离**，才能准确反映真实进度

### 2.2.3 `compute_ideal_waypoint_index()`

Given a task's reveal time and current time, compute which waypoint index the agent should ideally have reached.

```cpp
int compute_ideal_waypoint_index(const Task& task, int t_revealed, 
                                  int current_time, SharedEnvironment* env) {
    if (current_time <= t_revealed) return 0;  // Not enough time to move
    
    int time_elapsed = current_time - t_revealed;
    auto cumulative_costs = compute_segment_costs(task, env);
    
    // Binary search for first cumulative_cost >= time_elapsed
    int lo = 0, hi = cumulative_costs.size() - 1;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (cumulative_costs[mid] < time_elapsed) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}
```

**Example**:
- Task: `t_revealed=10`, `locations=[p0,p1,p2,p3,p4]`, `segment_costs=[2,2,2,2,2]`
- `cumulative_costs = [0, 2, 4, 6, 8, 10]`
- `current_time = 17`, `time_elapsed = 7`
- Binary search: first cumulative >= 7 is index 4 (cumulative[4]=8)
- **Result**: ideal = 4, but actual `idx_next_loc` might be 2
- **Deviation**: 2 - 4 = -2 (agent is 2 waypoints behind)

#### 2.2.4 `measure_progress_deviation()` — 使用 actual_progress_cost

Measures how far the agent is ahead or behind the ideal schedule.

```cpp
struct ProgressDeviation {
    int ideal_waypoint_idx;     // What index agent should have reached
    int actual_waypoint_idx;    // What index agent actually has reached
    int deviation_waypoints;    // actual - ideal (negative = behind)
    double deviation_time;      // time-based deviation in timesteps
    bool is_abnormal;           // True if deviation exceeds threshold
    enum DeviationType { AHEAD, ON_SCHEDULE, BEHIND, ABNORMAL } type;
};

// Key constants
const double ABNORMAL_DEVIATION_THRESHOLD_WAYPOINTS = 2.0;  // 2 waypoints behind = abnormal
const double ABNORMAL_DEVIATION_THRESHOLD_TIME = 10.0;      // 10 timesteps behind = abnormal

ProgressDeviation measure_progress_deviation(int agent_id, const Task& task,
                                             int current_time, SharedEnvironment* env) {
    ProgressDeviation result;

    // Get t_revealed from task (NOT task_start_time which is when agent was assigned)
    int task_revealed = task.t_revealed;

    // Compute ideal waypoint based on when task was revealed
    result.ideal_waypoint_idx = compute_ideal_waypoint_index(task, task_revealed, current_time, env);

    // Get actual progress cost (includes partial segment progress)
    int actual_progress_cost = compute_actual_progress_cost(agent_id, task, env);
    auto cumulative_costs = compute_segment_costs(task, env);
    int total_cost = cumulative_costs.back();

    // Convert actual_progress_cost back to waypoint index approximation
    // actual_waypoint_idx = how many waypoints' worth of progress has been made
    result.actual_waypoint_idx = 0;
    for (int k = 0; k < cumulative_costs.size(); k++) {
        if (cumulative_costs[k] <= actual_progress_cost) {
            result.actual_waypoint_idx = k;
        }
    }

    // Compute deviation in waypoint terms
    result.deviation_waypoints = result.actual_waypoint_idx - result.ideal_waypoint_idx;

    // Compute time-based deviation
    int ideal_progress_cost = std::min(total_cost, current_time - task_revealed);
    result.deviation_time = (double)(actual_progress_cost - ideal_progress_cost);
    
    // Classify deviation type
    if (result.deviation_waypoints < -ABNORMAL_DEVIATION_THRESHOLD_WAYPOINTS ||
        result.deviation_time < -ABNORMAL_DEVIATION_THRESHOLD_TIME) {
        result.type = ProgressDeviation::ABNORMAL;
        result.is_abnormal = true;
    } else if (result.deviation_waypoints < -1) {
        result.type = ProgressDeviation::BEHIND;
        result.is_abnormal = false;
    } else if (result.deviation_waypoints <= 1) {
        result.type = ProgressDeviation::ON_SCHEDULE;
        result.is_abnormal = false;
    } else {
        result.type = ProgressDeviation::AHEAD;
        result.is_abnormal = false;
    }
    
    return result;
}
```

**Concrete Example**:
```
Task: t_revealed=100, locations=[A,B,C,D,E], segment_costs=[5,5,5,5,5]
      cumulative_costs = [0,5,10,15,20,25]
      
Current_time = 140 (40 timesteps elapsed since reveal)
time_elapsed = 40

ideal_waypoint_idx = binary_search([0,5,10,15,20,25], >=40) = 6 (AT_END, task should be done)
actual_waypoint_idx = task.idx_next_loc = 3 (agent still at C, heading to D)

deviation_waypoints = 3 - 6 = -3 (3 waypoints behind)
deviation_time = cumulative[3] - cumulative[6] = 15 - 25 = -10 (10 timesteps behind schedule)

Result: is_abnormal = TRUE (deviation exceeds threshold)
```

**Interpretation**: At current_time=140 (40 steps into task), the agent should have completed the task (all 25 path cost worth). But agent is only at waypoint 3 (cost 15), which is 10 timesteps worth of path behind schedule. This is abnormal - likely a task assignment error or severe system failure.

#### 2.2.5 `update_consecutive_wait_with_next_loc()`

Replaces the current consecutive_wait logic that uses goal-based distance. Uses waypoint-based progress instead.

```cpp
// Track consecutive waypoint progress failures
// If agent's idx_next_loc doesn't advance when it should, count as "waiting"

struct WaypointWaitState {
    int last_waypoint_idx;       // idx_next_loc at last check
    int last_check_time;        // current_time at last check
    int consecutive_no_progress; // count of consecutive timesteps with no waypoint progress
};

std::unordered_map<int, WaypointWaitState> agent_waypoint_wait_state;

void update_consecutive_wait_with_next_loc(int agent_id, int task_id, 
                                           SharedEnvironment* env, int current_time) {
    if (task_id < 0) {
        agent_waypoint_wait_state.erase(agent_id);
        agent_consecutive_wait[agent_id] = 0;  // Keep using old name for compatibility
        return;
    }
    
    const Task& task = env->task_pool[task_id];
    
    // Get ideal waypoint for this time
    int ideal_idx = compute_ideal_waypoint_index(task, task.t_revealed, current_time, env);
    
    // Get actual waypoint
    int actual_idx = task.idx_next_loc;
    
    // Look up previous state
    auto it = agent_waypoint_wait_state.find(agent_id);
    if (it == agent_waypoint_wait_state.end()) {
        // First time seeing this agent
        WaypointWaitState state;
        state.last_waypoint_idx = actual_idx;
        state.last_check_time = current_time;
        state.consecutive_no_progress = 0;
        agent_waypoint_wait_state[agent_id] = state;
        agent_consecutive_wait[agent_id] = 0;
        return;
    }
    
    WaypointWaitState& state = it->second;
    
    // Determine if agent made progress since last check
    // Progress = actual_idx increased OR agent reached a waypoint it should have reached
    
    bool made_progress = false;
    
    // Check 1: Did idx_next_loc advance?
    if (actual_idx > state.last_waypoint_idx) {
        made_progress = true;
    }
    
    // Check 2: Is agent at or ahead of ideal (even if idx didn't change)?
    // This handles the case where agent is making progress but we haven't updated idx yet
    if (actual_idx >= ideal_idx) {
        made_progress = true;
    }
    
    // Check 3: Time-based check - if significant time passed without reaching ideal
    int time_since_last_check = current_time - state.last_check_time;
    int expected_progress = time_since_last_check;  // Should progress at ~1 waypoint per cost
    
    // Update wait counter
    if (made_progress) {
        agent_consecutive_wait[agent_id] = 0;
    } else {
        // Also check if agent is significantly behind ideal
        // If agent is at waypoint X but should be at waypoint Y (Y > X), no progress
        if (actual_idx < ideal_idx - 1) {
            // Agent is falling behind - increment counter more aggressively
            agent_consecutive_wait[agent_id] += 2;
        } else {
            agent_consecutive_wait[agent_id]++;
        }
    }
    
    // Update state
    state.last_waypoint_idx = actual_idx;
    state.last_check_time = current_time;
}
```

**Key Difference from Old Logic**:
- Old: measures distance-to-goal, counts as "no progress" if remaining distance doesn't decrease
- New: measures waypoint index, counts as "no progress" if idx_next_loc doesn't advance when it should

**Concrete Example**:
```
Task: t_revealed=0, locations=[p0,p1,p2,p3], costs=[5,5,5,5], cumulative=[0,5,10,15,20]

Scenario A: Normal traffic (agent making progress but slowly)
t=0:  ideal_idx=0, actual_idx=0, wait=0
t=5:  ideal_idx=1, actual_idx=0 (agent still at p0), wait=1 (no progress but only 1 behind)
t=10: ideal_idx=2, actual_idx=1 (agent reached p1), wait=0 (progress made!)
t=15: ideal_idx=3, actual_idx=2 (agent reached p2), wait=0 (progress made!)
Result: Agent detected as making progress despite traffic delays

Scenario B: Agent stuck (abnormal)
t=0:  ideal_idx=0, actual_idx=0, wait=0
t=5:  ideal_idx=1, actual_idx=0, wait=1
t=10: ideal_idx=2, actual_idx=0, wait=2 (still at p0, falling behind)
t=15: ideal_idx=3, actual_idx=0, wait=4 (2 behind → increment by 2)
Result: Agent detected as abnormally behind schedule
```

#### 2.2.6 `get_agent_efficiency_v2()` — 使用 actual_progress_cost

Improved efficiency calculation using waypoint-based progress.

```cpp
// v2 uses waypoint index progress instead of distance-to-goal

double get_agent_efficiency_v2(int agent_id, int task_id, SharedEnvironment* env, int current_time) {
    if (task_id < 0) return 0.0;

    const Task& task = env->task_pool[task_id];

    // Use t_revealed as the start time (when task became available)
    int time_elapsed = current_time - task.t_revealed;
    if (time_elapsed <= 0) return 0.0;

    // Get cumulative costs along path
    auto cumulative_costs = compute_segment_costs(task, env);
    int total_cost = cumulative_costs.back();

    // Compute ideal progress (what should have been accomplished)
    int ideal_progress_cost = std::min(total_cost, time_elapsed);

    // Compute actual progress cost (includes partial segment progress!)
    int actual_progress_cost = compute_actual_progress_cost(agent_id, task, env);

    // Efficiency = actual_progress / ideal_progress
    double efficiency = 0.0;
    if (ideal_progress_cost > 0) {
        efficiency = (double)actual_progress_cost / (double)ideal_progress_cost;
    }

    // If behind schedule, apply penalty factor
    int ideal_idx = compute_ideal_waypoint_index(task, task.t_revealed, current_time, env);
    if (actual_progress_cost < ideal_progress_cost) {
        // Agent is behind schedule — find how many waypoints behind
        int behind_cost = ideal_progress_cost - actual_progress_cost;
        // Apply exponential penalty: each unit of cost behind multiplies efficiency by a factor
        // E.g., behind by 10 steps → efficiency *= 0.5
        double penalty = std::pow(0.95, behind_cost);  // ~0.95^10 ≈ 0.60
        efficiency *= penalty;
    }

    return efficiency;
}
```

**Concrete Example**:
```
Task: t_revealed=0, locations=[p0,p1,p2,p3,p4], costs=[5,5,5,5,5], cumulative=[0,5,10,15,20,25]

Scenario A: Agent on schedule (at p2 at t=10)
time_elapsed = 10
ideal_progress_cost = min(25, 10) = 10
actual_waypoint_idx = 2, actual_progress_cost = cumulative[2] = 10
efficiency = 10/10 = 1.0 (100% efficient)

Scenario B: Agent behind schedule (at p1 at t=15)
time_elapsed = 15
ideal_progress_cost = min(25, 15) = 15
actual_waypoint_idx = 1, actual_progress_cost = cumulative[1] = 5
efficiency = 5/15 = 0.33

Penalty: ideal_idx = 3, actual_idx = 1, behind_by = 2
efficiency = 0.33 * 0.8^2 = 0.33 * 0.64 = 0.21

Scenario C: Agent ahead of schedule (at p3 at t=10)
time_elapsed = 10
ideal_progress_cost = min(25, 10) = 10
actual_waypoint_idx = 3, actual_progress_cost = cumulative[3] = 15
efficiency = 15/10 = 1.5 (150% - bonus for being ahead)
```

---

## Section 3: Implementation Points

### 3.1 Functions to Add/Modify

|| Function | Action | Notes ||
|----------|---------|--------|-------|
| `compute_segment_costs()` | Add | Helper: cumulative path costs ||
| `compute_actual_progress_cost()` | Add | **NEW**: actual progress incl. partial segment ||
| `compute_ideal_waypoint_index()` | Add | Core: ideal waypoint via binary search ||
| `measure_progress_deviation()` | Add | Returns detailed deviation using actual_progress_cost ||
| `get_agent_efficiency` | Modify → v2 | Replace logic to use actual_progress_cost ||
| `update_consecutive_wait_with_next_loc()` | Add | Replaces lines 331-351 ||
| `efficient_task_swap()` | Modify | Use new efficiency function ||

### 3.2 Data Structures to Add

```cpp
// New state tracking for waypoint-based wait
struct WaypointWaitState {
    int last_waypoint_idx;
    int last_check_time;
    int consecutive_no_progress;
};

// Map from agent_id to wait state
std::unordered_map<int, WaypointWaitState> agent_waypoint_wait_state;

// Progress deviation result structure
struct ProgressDeviation {
    int ideal_waypoint_idx;
    int actual_waypoint_idx;
    int deviation_waypoints;
    double deviation_time;
    bool is_abnormal;
    enum DeviationType { AHEAD, ON_SCHEDULE, BEHIND, ABNORMAL } type;
};
```

### 3.3 Key Constants to Add

```cpp
// Thresholds for abnormal deviation detection
const double ABNORMAL_WAYPOINT_THRESHOLD = 2.0;   // 2 waypoints behind = abnormal
const double ABNORMAL_TIME_THRESHOLD = 10.0;      // 10 timesteps behind = abnormal
const double EFFICIENCY_PENALTY_FACTOR = 0.8;     // Multiplier per waypoint behind
const double EGTS_EFFICIENCY_THRESHOLD_V2 = 0.25; // Tighter threshold for v2
```

### 3.4 Key Formulas

**Formula 1: Cumulative Path Cost**
```
cumulative_costs[0] = 0
cumulative_costs[k] = Σ(i=1 to k) get_h(env, locations[i-1], locations[i])
```

**Formula 2: Ideal Waypoint Index (Binary Search)**
```
time_elapsed = current_time - t_revealed
ideal_idx = smallest k where cumulative_costs[k] >= time_elapsed
           = lower_bound(cumulative_costs, time_elapsed)
```

**Formula 3: Progress Deviation**
```
deviation_waypoints = actual_idx_next_loc - ideal_idx
deviation_time = cumulative_costs[actual_idx] - time_elapsed
is_abnormal = (deviation_waypoints < -2) OR (deviation_time < -10)
```

**Formula 4: Efficiency v2**
```
ideal_progress = min(cumulative_costs.back(), time_elapsed)
actual_progress = cumulative_costs[idx_next_loc]
base_efficiency = actual_progress / ideal_progress
if (idx_next_loc < ideal_idx):
    efficiency = base_efficiency * (0.8 ^ (ideal_idx - idx_next_loc))
else:
    efficiency = base_efficiency
```

---

## Section 4: Call Sites

### 4.1 `get_agent_efficiency` Call Sites

| Location | Line | Usage | Change Needed |
|----------|------|-------|---------------|
| `efficient_task_swap` | 154, 174 | Check if agent efficiency < threshold for swap candidate | Use `get_agent_efficiency_v2` instead |
| H32 mutual inhibition | 499, 500 | Check if both agents are low efficiency for mutual swap | Use `get_agent_efficiency_v2` instead |

**Change in `efficient_task_swap` (lines 154-156)**:
```cpp
// OLD
double eff1 = get_agent_efficiency(a1, t1, env, current_time);
if (eff1 >= EGTS_EFFICIENCY_THRESHOLD) continue;

// NEW
double eff1 = get_agent_efficiency_v2(a1, t1, env, current_time);
if (eff1 >= EGTS_EFFICIENCY_THRESHOLD_V2) continue;
```

### 4.2 `consecutive_wait` Update Call Sites

| Location | Line | Usage | Change Needed |
|----------|------|-------|---------------|
| H31 wait tracking loop | 331-351 | Count consecutive steps with no progress | Replace entire loop with new function |
| H31 initialization | 290-293 | Initialize wait state for new agent | Use new init logic |
| H40 task switching | 407-410 | Reset wait counter after task switch | Keep same, call new update |

**Change in H31 loop (lines 331-351)**:
```cpp
// OLD
for (int a = 0; a < env->num_of_agents; a++) {
    int task_id = env->curr_task_schedule[a];
    if (task_id < 0) { agent_consecutive_wait[a] = 0; continue; }
    int agent_loc = env->curr_states[a].location;
    int goal_loc = env->task_pool[task_id].locations.back();
    int remaining = DefaultPlanner::get_h(env, agent_loc, goal_loc);
    auto prev_it = agent_prev_remaining.find(a);
    if (prev_it != agent_prev_remaining.end()) {
        if (remaining >= prev_it->second)
            agent_consecutive_wait[a]++;
        else
            agent_consecutive_wait[a] = 0;
    }
    agent_prev_remaining[a] = remaining;
}

// NEW
for (int a = 0; a < env->num_of_agents; a++) {
    int task_id = env->curr_task_schedule[a];
    update_consecutive_wait_with_next_loc(a, task_id, env, current_time);
}
```

### 4.3 `task_start_time` vs `t_revealed` Clarification

- `task_start_time[task_id]` = when agent was assigned to task (set at assignment time)
- `task.t_revealed` = when task was originally revealed to the system

For efficiency calculation, using `t_revealed` gives a consistent baseline regardless of when assignment happened. This is important because two agents assigned the same task at different times should still be measured against the same schedule.

However, for `consecutive_wait` tracking, we want to know if the specific agent is making progress since assignment. So `task_start_time` is appropriate for wait tracking, while `t_revealed` is appropriate for schedule deviation.

### 4.4 Additional Call Sites to Review

| Location | Line | Usage | Change Needed |
|----------|------|-------|---------------|
| H26 reassess | 317-325 | Check if task is stale | Use new deviation check |
| H27 efficiency switch | 432-436 | Compute efficiency for switch decision | Use v2 |
| H40 task switching | 381-382 | Check wait count threshold | No change needed |
| H32 mutual inhibition | 496 | Check efficiency threshold | Use v2 |

---

## Appendix A: Example Calculation with Full Numbers

### Scenario: Normal Detour vs Abnormal Deviation

**Setup**:
- Task revealed at `t_revealed = 100`
- Path: `locations = [A, B, C, D, E]`
- Segment costs: `get_h(A,B)=5, get_h(B,C)=5, get_h(C,D)=5, get_h(D,E)=5`
- Cumulative: `[0, 5, 10, 15, 20, 25]`

**Case 1: Normal Traffic Delay (Agent making progress)**
```
current_time = 120 (20 timesteps elapsed)
time_elapsed = 20

ideal_waypoint_idx = lower_bound([0,5,10,15,20,25], >=20) = 5 (AT_END, task complete)
Agent actually at idx_next_loc = 3 (at D, heading to E)

deviation_waypoints = 3 - 5 = -2 (2 behind, but within threshold)
deviation_time = 15 - 25 = -10 (but this is near end of task)

Classification: BEHIND but NOT ABNORMAL (within 2 waypoint threshold)
→ Agent is likely in normal traffic, no task swap needed
→ consecutive_wait should be low (agent is making waypoint progress)
```

**Case 2: Abnormal Deviation (Agent stuck)**
```
current_time = 130 (30 timesteps elapsed)
time_elapsed = 30

ideal_waypoint_idx = lower_bound([0,5,10,15,20,25], >=30) = 6 (AT_END, way past complete)
Agent still at idx_next_loc = 1 (at B, should be done!)

deviation_waypoints = 1 - 6 = -5 (5 behind, exceeds threshold of -2)
deviation_time = 5 - 25 = -20 (20 timesteps behind schedule)

Classification: ABNORMAL (deviation exceeds thresholds)
→ Agent likely has wrong task assignment or severe system issue
→ consecutive_wait should be high
→ EGTS should consider swapping this agent
```

**Case 3: Agent Making Slow Progress (within tolerance)**
```
current_time = 125 (25 timesteps elapsed)
time_elapsed = 25

ideal_waypoint_idx = lower_bound([0,5,10,15,20,25], >=25) = 5 (AT_END)
Agent at idx_next_loc = 3 (at D)

deviation_waypoints = 3 - 5 = -2 (exactly at threshold)
deviation_time = 15 - 25 = -10

Classification: BEHIND but NOT ABNORMAL
→ consecutive_wait low, agent making progress
→ Efficiency = actual_progress / ideal_progress = 15 / 25 = 0.6 (60%)
→ No penalty applied (at threshold boundary)
→ Agent not flagged for swap
```

---

## Appendix B: Compatibility Notes

1. **Keep `agent_consecutive_wait` map** - used by H31, H40, H49 for thresholds. Just change how it's updated.

2. **Keep `agent_prev_remaining` map** - only used by old consecutive_wait logic. Can deprecate after migration.

3. **Keep `task_start_time` map** - still needed for some time-based calculations. The distinction is:
   - `t_revealed`: task visibility start (for schedule deviation)
   - `task_start_time`: agent assignment time (for wait tracking)

4. **Add new maps gradually** - don't remove old maps until new logic is fully validated.

5. **Maintain EGTS cooldown semantics** - the swap logic still needs time-based cooldowns even if efficiency metric changes.