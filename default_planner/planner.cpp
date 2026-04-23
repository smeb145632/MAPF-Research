#include "planner.h"
#include "heuristics.h"
#include "SharedEnv.h"
#include "pibt.h"
#include "flow.h"
#include "const.h"
#include <iostream>


namespace DefaultPlanner{

    // ============================================================
    // 辅助函数：将 Action 枚举转为字符串（用于调试输出）
    // ============================================================
    // Action::FW  = Forward (前进)
    // Action::CR  = Clockwise Rotate (右转90度)
    // Action::CCR = Counter-Clockwise Rotate (左转90度)
    // Action::W   = Wait (等待)
    // Action::NA  = Not Available (不可用)
    static const char* debug_action_to_string(Action action)
    {
        switch (action)
        {
            case Action::FW: return "FW";
            case Action::CR: return "CR";
            case Action::CCR: return "CCR";
            case Action::W: return "W";
            case Action::NA: return "NA";
            default: return "UNKNOWN";
        }
    }


    // ============================================================
    // 默认规划器的全局数据
    // 这些变量在规划过程中维护状态，跨多次调用保持
    // ============================================================

    std::vector<int> decision;      // decision[位置] = 占用该位置的 agent ID（-1 表示空闲）
    std::vector<int> prev_decision; // 上一帧的 decision（用于碰撞检测）
    std::vector<double> p;          // 每个 agent 的优先级
    std::vector<State> prev_states; // agent 上一帧的状态
    std::vector<State> next_states; // agent 当前帧的状态（规划过程中更新）
    std::vector<int> ids;           // agent ID 列表 [0,1,2,...,n-1]
    std::vector<double> p_copy;     // 优先级的备份（用于恢复）
    std::vector<bool> occupied;     // 位置是否被占用的标记
    std::vector<DCR> decided;       // 每个 agent 的"已决定动作"状态
    std::vector<bool> checked;      // 移动检查标记
    std::vector<bool> require_guide_path;  // 是否需要重新计算引导路径
    std::vector<int> dummy_goals;   // 虚拟目标（当 agent 没有任务时使用）
    TrajLNS trajLNS;                // Trajectory LNS 数据结构
    std::mt19937 mt1;               // 随机数生成器


    // ============================================================
    // rollout_next_state — 根据动作计算下一个状态
    // ============================================================
    // 作用：根据当前状态和动作，推算出 agent 的下一个状态
    // 注意：这只是"仿真"计算，不会真正移动 agent
    //
    // @param state 当前状态
    // @param action 要执行的动作
    // @param env 共享环境
    // @return 执行动作后的新状态
    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
    {
        State next = state;
        next.timestep = state.timestep + 1;  // 时间步 +1

        // ----------------------------------------
        // 处理旋转动作（不改变位置，只改变朝向）
        // ----------------------------------------
        if (action == Action::CR)
        {
            // 右转：(当前朝向 + 1) % 4
            // 0=北, 1=东, 2=南, 3=西
            next.orientation = (state.orientation + 1) % 4;
            return next;
        }
        if (action == Action::CCR)
        {
            // 左转：(当前朝向 + 3) % 4
            next.orientation = (state.orientation + 3) % 4;
            return next;
        }
        if (action != Action::FW)
        {
            // 非前进动作（等待等），位置不变
            return next;
        }

        // ----------------------------------------
        // 处理前进动作
        // ----------------------------------------
        // 计算朝向对应的位置增量
        //   0(北): delta = +1 (行+1，向下)
        //   1(东): delta = +cols (列+1，向右)
        //   2(南): delta = -1 (行-1，向上)
        //   3(西): delta = -cols (列-1，向左)
        int delta = 0;
        if (state.orientation == 0) delta = 1;
        if (state.orientation == 1) delta = env->cols;
        if (state.orientation == 2) delta = -1;
        if (state.orientation == 3) delta = -env->cols;

        // 计算下一个位置
        int next_loc = state.location + delta;

        // 检查移动是否有效（边界内且不是障碍物）
        if (next_loc >= 0 && next_loc < env->map.size() && validateMove(state.location, next_loc, env))
        {
            next.location = next_loc;  // 位置更新
        }
        // 如果无效，位置保持不变（agent 撞墙了）
        return next;
    }


    // ============================================================
    // initialize_dummy_goals — 初始化虚拟目标
    // ============================================================
    // 作用：当 agent 没有分配到任务时，使用当前位置作为虚拟目标
    // 只在时间步 0 时调用一次
    static void initialize_dummy_goals_if_needed(SharedEnvironment* env)
    {
        // 只在仿真开始时初始化
        if (env->curr_timestep != 0)
            return;

        dummy_goals.resize(env->num_of_agents);
        for(int i = 0; i < env->num_of_agents; i++)
        {
            // 虚拟目标 = agent 的起始位置
            dummy_goals.at(i) = env->curr_states.at(i).location;
        }
    }


    // ============================================================
    // setup_multistep_episode_state — 设置多步规划场景状态
    // ============================================================
    // 作用：
    //   1. 初始化启发式表
    //   2. 为每个 agent 设置任务/目标
    //   3. 检测引导路径是否需要更新
    //   4. 更新 agent 的优先级
    static void setup_multistep_episode_state(SharedEnvironment* env, TimePoint flow_end_time,
	                                              std::vector<double>& local_priority)
    {
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);  // 所有位置初始为空闲

        // ----------------------------------------
        // 遍历每个 agent
        // ----------------------------------------
        for(int i = 0; i < env->num_of_agents; i++)
        {
            // 1. 如果时间还够，初始化目标位置的启发式表
            if (std::chrono::steady_clock::now() < flow_end_time)
            {
                for(int j = 0; j < env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                    if (trajLNS.heuristics.at(goal_loc).empty())
                    {
                        // 为这个目标位置初始化启发式距离表
                        init_heuristic(trajLNS.heuristics[goal_loc], env, goal_loc);
                    }
                }
            }

            // 2. 设置 agent 的任务/目标
            if (env->goal_locations[i].empty())
            {
                // 没有任务 → 使用虚拟目标（当前位置）
                trajLNS.tasks[i] = dummy_goals.at(i);
                local_priority[i] = p_copy[i];  // 优先级恢复默认值
            }
            else
            {
                // 有任务 → 使用任务的下一个 errand 作为目标
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }

            // 3. 检测引导路径是否需要重新计算
            // 如果目标改变了，或者没有引导路径，就需要重新计算
            require_guide_path[i] = false;
            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
                require_guide_path[i] = true;

            // 4. 更新 agent 的规划状态
            assert(env->curr_states[i].location >= 0);
            prev_states[i] = env->curr_states[i];  // 记录上一帧状态
            next_states[i] = State();              // 重置下一帧状态

            // 标记当前位置被此 agent 占用
            prev_decision[env->curr_states[i].location] = i;

            // 5. 处理"已决定动作"状态
            if (decided[i].loc == -1)
            {
                decided[i].loc = env->curr_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            if (prev_states[i].location == decided[i].loc)
            {
                decided[i].state = DONE::DONE;  // agent 到达了之前决定的位置
            }
            if (decided[i].state == DONE::NOT_DONE)
            {
                // agent 还在执行之前的决定
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc, -1, -1);
            }

            // 6. 跨 episode 更新优先级
            if (require_guide_path[i])
                local_priority[i] = p_copy[i];  // 需要重新规划，恢复基础优先级
            else if (!env->goal_locations[i].empty())
                local_priority[i] = local_priority[i] + 1;  // 已有引导路径，优先级+1

            // 7. 如果 agent 处于死角（只有一个邻居），优先级大幅提升
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1)
            {
                local_priority[i] = local_priority[i] + 10;
            }
        }
    }


    // ============================================================
    // update_guide_paths_once_for_multistep — 更新引导路径（一次性）
    // ============================================================
    // 作用：
    //   1. 为目标改变的 agent 更新引导路径
    //   2. 运行 Frank-Wolfe 优化流量分配
    //
    // 引导路径 = 从当前位置到目标的预计算最短路径
    static void update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time)
    {
        // ----------------------------------------
        // 1. 为需要重新规划的 agent 更新引导路径
        // ----------------------------------------
        for (int i = 0; i < env->num_of_agents; i++)
        {
            // 如果超时，停止
            if (std::chrono::steady_clock::now() > flow_end_time)
                break;

            // 只为需要更新的 agent 更新
            if (require_guide_path[i])
            {
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);  // 移除旧轨迹
                update_traj(trajLNS, i);      // 计算新轨迹
            }
        }

        // ----------------------------------------
        // 2. Frank-Wolfe 优化（流量分配阶段的一次性优化）
        // ----------------------------------------
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated, flow_end_time);
    }


    // ============================================================
    // refresh_multistep_step_state — 刷新多步状态（每步调用）
    // ============================================================
    // 作用：在每个规划步骤开始时刷新状态
    // 与 setup_multistep_episode_state 类似，但更轻量
    static void refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority)
    {
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);

        for(int i = 0; i < env->num_of_agents; i++)
        {
            // 设置任务/目标
            if (env->goal_locations[i].empty())
            {
                trajLNS.tasks[i] = dummy_goals.at(i);
                local_priority[i] = p_copy[i];
            }
            else
            {
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
                local_priority[i] = local_priority[i] + 1;
            }

            assert(env->curr_states[i].location >= 0);
            prev_states[i] = env->curr_states[i];
            next_states[i] = State();
            prev_decision[env->curr_states[i].location] = i;

            if (prev_states[i].location == decided[i].loc)
            {
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE)
            {
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc, -1, -1);
            }

            // 死角检测
            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->curr_states[i].location].size() == 1)
            {
                local_priority[i] = local_priority[i] + 10;
            }
        }
    }


    // ============================================================
    // run_multistep_pibt_once — 运行一次 PIBT 算法
    // ============================================================
    // 作用：为所有 agent 决定下一步的动作
    //
    // PIBT = Priority Inheritance with Backtracking
    //        优先级继承 + 回溯的多 agent 路径规划算法
    //
    // @param one_step_actions 输出：每个 agent 的下一步动作
    static void run_multistep_pibt_once(SharedEnvironment* env, std::vector<double>& local_priority,
                                        std::vector<Action>& one_step_actions)
    {
        // ----------------------------------------
        // 1. 按优先级排序 agent（高优先级先规划）
        // ----------------------------------------
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return local_priority.at(a) > local_priority.at(b);
            }
        );

        // 清空占用标记
        std::fill(occupied.begin(), occupied.end(), false);

        // ----------------------------------------
        // 2. 运行 causalPIBT（因果 PIBT）
        // ----------------------------------------
        for (int i : ids)
        {
            // 跳过已完成当前决定的 agent
            if (decided[i].state == DONE::NOT_DONE)
            {
                continue;
            }
            // 执行 PIBT
            if (next_states[i].location == -1)
            {
                assert(prev_states[i].location >= 0 && prev_states[i].location < env->map.size());
                causalPIBT(i, -1, prev_states, next_states,
                    prev_decision, decision,
                    occupied, trajLNS);
            }
        }

        // ----------------------------------------
        // 3. 生成最终动作
        // ----------------------------------------
        one_step_actions.resize(env->num_of_agents);
        for (int id : ids)
        {
            // 释放下一状态占用的位置
            if (next_states.at(id).location != -1)
                decision.at(next_states.at(id).location) = -1;

            // 更新 decided 状态
            if (next_states.at(id).location >= 0)
            {
                decided.at(id) = DCR({next_states.at(id).location, DONE::NOT_DONE});
            }

            // 根据当前状态和目标位置，决定实际动作
            one_step_actions.at(id) = getAction(prev_states.at(id), decided.at(id).loc, env);
            checked.at(id) = false;
        }

        // ----------------------------------------
        // 4. 移动检查（确保 FW 动作不会导致碰撞）
        // ----------------------------------------
        for (int id = 0; id < env->num_of_agents; id++)
        {
            if (!checked.at(id) && one_step_actions.at(id) == Action::FW)
            {
                moveCheck(id, checked, decided, one_step_actions, prev_decision);
            }
        }

        // 保存当前状态供下一帧使用
        prev_states = next_states;
    }


    // ============================================================
    // append_actions_and_rollout_states — 保存动作并更新状态
    // ============================================================
    // 作用：
    //   1. 将本步动作追加到动作序列
    //   2. 仿真更新 agent 状态（用于内部 rollout）
    //
    // 注意：这只是"仿真"状态，不会影响实际的 SharedEnvironment
    static void append_actions_and_rollout_states(SharedEnvironment* env,
                                                   std::vector<std::vector<Action>> & actions,
                                                   const std::vector<Action>& one_step_actions)
    {
        // ----------------------------------------
        // 1. 追加动作到每个 agent 的动作序列
        // ----------------------------------------
        for (int aid = 0; aid < env->num_of_agents; aid++)
        {
            actions[aid].push_back(one_step_actions[aid]);
        }

        // ----------------------------------------
        // 2. 仿真更新状态（rollout）
        // ----------------------------------------
        std::vector<State> rolled_states(env->num_of_agents);
        for (int aid = 0; aid < env->num_of_agents; aid++)
        {
            // 根据动作计算下一状态
            rolled_states[aid] = rollout_next_state(env->curr_states[aid], one_step_actions[aid], env);
        }

        // 保存 rollout 后的状态
        env->curr_states = rolled_states;
        env->curr_timestep += 1;
    }


    // ============================================================
    // initialize — 规划器预处理初始化
    // ============================================================
    // 作用：在仿真开始前初始化规划器的数据结构和启发式表
    //
    // @param preprocess_time_limit 预处理时间限制（毫秒）
    // @param env 共享环境
    void initialize(int preprocess_time_limit, SharedEnvironment* env)
    {
        // ----------------------------------------
        // 1. 初始化所有数据结构
        // ----------------------------------------
        assert(env->num_of_agents != 0);

        p.resize(env->num_of_agents);                    // 优先级
        decision.resize(env->map.size(), -1);           // 位置占用表
        prev_states.resize(env->num_of_agents);          // 上一帧状态
        next_states.resize(env->num_of_agents);           // 当前帧状态
        decided.resize(env->num_of_agents, DCR({-1, DONE::DONE}));  // 已决定状态
        occupied.resize(env->map.size(), false);         // 位置占用标记
        checked.resize(env->num_of_agents, false);       // 检查标记
        ids.resize(env->num_of_agents);                  // agent ID 列表
        require_guide_path.resize(env->num_of_agents, false);  // 引导路径需求

        // ids = [0, 1, 2, ..., num_of_agents-1]
        for (int i = 0; i < ids.size(); i++)
        {
            ids[i] = i;
        }

        // ----------------------------------------
        // 2. 初始化启发式距离表
        // ----------------------------------------
        init_heuristics(env);

        // 设置随机种子（保证结果可复现）
        mt1.seed(0);
        srand(0);

        // ----------------------------------------
        // 3. 初始化 TrajLNS 数据结构
        // ----------------------------------------
        new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
        trajLNS.init_mem();

        // ----------------------------------------
        // 4. 分配初始优先级
        // ----------------------------------------
        std::shuffle(ids.begin(), ids.end(), mt1);  // 随机打乱顺序
        for (int i = 0; i < ids.size(); i++)
        {
            // 优先级 = (n - i) / (n + 1)
            // 打乱后，排在前面的 agent 优先级更高
            p[ids[i]] = ((double)(ids.size() - i)) / ((double)(ids.size() + 1));
        }
        p_copy = p;  // 保存优先级副本

        return;
    };


    // ============================================================
    // plan — 默认规划器的主函数
    // ============================================================
    // 作用：为所有 agent 规划接下来 num_steps 步的动作
    //
    // 算法流程：
    //   1. 检查任务/目标变化，执行必要的更新
    //   2. 计算并优化流量分配的引导路径
    //   3. 使用 PIBT 算法为每个 agent 决定动作
    //
    // @param time_limit 规划时间限制（毫秒）
    // @param actions 输出：每个 agent 的动作序列
    // @param env 共享环境
    // @param num_steps 要规划的步数
    //
    // 注意：默认规划器忽略转向动作的成本，转向动作会被处理为额外的延迟
    void plan(int time_limit, std::vector<std::vector<Action>> & actions,
                         SharedEnvironment* env, int num_steps)
    {
        actions.clear();

        // 检查参数有效性
        if (env == nullptr || env->num_of_agents <= 0)
        {
            return;
        }

        actions.resize(env->num_of_agents);
        if (num_steps <= 0)
        {
            return;
        }

        // ----------------------------------------
        // 计算时间预算
        // ----------------------------------------
        const auto episode_start = std::chrono::steady_clock::now();

        // PIBT 时间预算 = (每100个agent的时间) * (agent数量/100)
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
        if (pibt_time <= 0)
        {
            pibt_time = 1;
        }

        // Flow 优化时间预算 = 总时间 - PIBT时间 - 容差
        const int flow_budget_ms = std::max(0, time_limit - pibt_time * num_steps - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
        TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

        std::vector<double> local_priority = p;

        // 保存原始状态（用于内部 rollout 后恢复）
        const std::vector<State> original_states = env->curr_states;
        const int original_timestep = env->curr_timestep;
        const std::vector<DCR> original_decided = decided;

        // ----------------------------------------
        // 每个多步规划 episode 的初始化
        // 注意：跨 episode 不保留未完成的转向补偿动作
        // ----------------------------------------
        decided.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

        // --- 一次性设置 ---
        initialize_dummy_goals_if_needed(env);                           // 初始化虚拟目标
        setup_multistep_episode_state(env, flow_end_time, local_priority);  // 设置场景状态
        // 更新引导路径, traffic-flow流程、加上轨迹偏离地图搜索后、通过偏移排序重新规划一遍,知道time-end;
        update_guide_paths_once_for_multistep(env, flow_end_time);

        // 记录设置阶段耗时
        const auto after_setup = std::chrono::steady_clock::now();
        const auto setup_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(after_setup - episode_start).count();
        std::cout << "[DefaultPlanner::plan] timing: time_limit=" << time_limit
              << "ms, pibt_time_hint=" << pibt_time
              << "ms, flow_budget=" << flow_budget_ms
              << "ms, setup_elapsed=" << setup_elapsed_ms << "ms" << std::endl;

        // 提交跨 episode 的优先级更新
        p = local_priority;

        // ----------------------------------------
        // 多步规划循环
        // ----------------------------------------
        for (int step = 0; step < num_steps; step++)
        {
            std::vector<Action> one_step_actions;

            // 从 step 1 开始，只需要 PIBT + rollout（不需要重新计算引导路径和 FW）
            if (step > 0)
            {
                refresh_multistep_step_state(env, local_priority);
            }

            // 运行一次 PIBT
            run_multistep_pibt_once(env, local_priority, one_step_actions);

            // 保存动作并更新状态
            append_actions_and_rollout_states(env, actions, one_step_actions);
        }

        // ----------------------------------------
        // 调试输出（仅 agent 47）
        // ----------------------------------------
        std::cout << "[DefaultPlanner::plan] computed actions for "
                  << env->num_of_agents << " agents over " << num_steps << " steps" << std::endl;
        for (int aid = 0; aid < env->num_of_agents; aid++)
        {
            if (aid != 47) continue;  // 只输出 agent 47 的信息
            int curr_loc = env->curr_states[aid].location;
            int goal_loc = -1;
            if (!env->goal_locations[aid].empty())
            {
                goal_loc = env->goal_locations[aid].front().first;
            }

            // 获取分配的任务信息
            int assigned_task_id = -1;
            std::string task_details = "N/A";
            if (aid < env->curr_task_schedule.size())
            {
                assigned_task_id = env->curr_task_schedule[aid];
                auto it = env->task_pool.find(assigned_task_id);
                if (it != env->task_pool.end())
                {
                    const auto& task = it->second;
                    std::ostringstream oss;
                    oss << "task_id=" << task.task_id
                        << ", t_revealed=" << task.t_revealed
                        << ", t_completed=" << task.t_completed
                        << ", agent_assigned=" << task.agent_assigned
                        << ", idx_next_loc=" << task.idx_next_loc
                        << ", locations=[";
                    for (size_t i = 0; i < task.locations.size(); ++i)
                    {
                        oss << task.locations[i];
                        if (i + 1 < task.locations.size()) oss << ",";
                    }
                    oss << "]";
                    const bool task_finished = (task.idx_next_loc >= static_cast<int>(task.locations.size()));
                    if (!task_finished)
                    {
                        oss << ", next_loc=" << task.locations[task.idx_next_loc];
                    }
                    task_details = oss.str();
                }
            }
            std::cout << "  agent " << aid
                      << ": loc=" << curr_loc
                      << ", goal=" << goal_loc
                      << ", assigned_task_id=" << assigned_task_id
                      << ", task_details=[" << task_details << "]:";
            for (int step = 0; step < static_cast<int>(actions[aid].size()); step++)
            {
                std::cout << (step == 0 ? " " : ", ") << debug_action_to_string(actions[aid][step]);
            }
            std::cout << std::endl;
        }

        // ----------------------------------------
        // 恢复原始状态
        // 注意：规划过程中进行了内部仿真（rollout），这里要恢复环境状态
        // 真正的状态更新会在 Executor 处理 plan 时发生
        // ----------------------------------------
        env->curr_states = original_states;
        env->curr_timestep = original_timestep;
        decided = original_decided;
    }
}
