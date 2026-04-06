#include <cmath>
#include "CompetitionSystem.h"
#include <boost/tokenizer.hpp>
#include "SharedEnv.h"
#include "nlohmann/json.hpp"
#include <functional>
#include <Logger.h>

using json = nlohmann::ordered_json;


// ============================================================
// 共享环境同步函数
// ============================================================
// 作用：将 Simulator 和 TaskManager 的最新状态同步到 SharedEnvironment (env)
//      供 Entry::compute() 中的规划器和调度器读取
//
// 时机：
//   - 规划器空闲时（!started）才同步，否则跳过（避免数据竞争）
//   - 正在规划时不同步，因为规划器在读取旧数据
void BaseSystem::sync_shared_env()
{
    // 规划器空闲，可以安全同步
    if (!started)
    {
        // 重新调整目标位置向量大小（应对动态添加 agent 的情况）
        env->goal_locations.resize(num_of_agents);

        // 同步任务管理器的状态到共享环境
        // 包括：task_pool（任务池）、curr_task_schedule（当前分配）、new_tasks（新任务）等
        task_manager.sync_shared_env(env);

        // 同步仿真器的状态到共享环境
        // 包括：agent 当前位置、动作执行进度等
        simulator.sync_shared_env(env);
    }
    else
    {
        // 规划器正在运行，只更新时间戳（不更新完整状态）
        // 防止规划器读取到不一致的中间状态
        env->curr_timestep = simulator.get_curr_timestep();
    }
}


// ============================================================
// 规划器包装函数（运行在独立线程中）
// ============================================================
// 作用：在线程中调用 Entry::compute()，让规划和执行可以并行
//
// 注意：这是一个线程入口函数，由 std::thread 调用
//       真正的计算在 planner->compute() 中执行
bool BaseSystem::planner_wrapper()
{
    planner->compute(plan_time_limit, proposed_plan, proposed_schedule);
    return true;
}


// ============================================================
// 预处理阶段初始化（在仿真开始前调用）
// ============================================================
// 作用：在独立线程中运行 Entry::initialize()，为仿真做准备
//       包括：规划器预处理（如构建启发式函数）、调度器初始化等
//
// 超时处理：如果超过 preprocess_time_limit，线程会被 detach，主线程继续执行
bool BaseSystem::planner_initialize()
{
    using namespace std::placeholders;

    // 创建一个可异步执行的任务：调用 Entry::initialize(preprocess_time_limit)
    // std::bind 将成员函数和 this 指针绑定
    std::packaged_task<void(int)> init_task(std::bind(&Entry::initialize, planner, _1));
    auto init_future = init_task.get_future();  // 获取未来结果

    // 记录规划开始时间（用于计算预处理耗时）
    env->plan_start_time = std::chrono::steady_clock::now();

    // 启动独立线程执行初始化任务
    auto init_td = std::thread(std::move(init_task), preprocess_time_limit);

    // 等待预处理完成（或超时）
    if (init_future.wait_for(std::chrono::milliseconds(preprocess_time_limit)) == std::future_status::ready)
    {
        // 预处理在时间限制内完成，正常 join 线程
        init_td.join();
        return true;
    }

    // 预处理超时，detach 线程（让它在后台继续运行，但我们不等它）
    init_td.detach();
    return false;
}


// ============================================================
// 记录预处理结果
// ============================================================
void BaseSystem::log_preprocessing(bool succ)
{
    if (logger == nullptr)
        return;

    if (succ)
    {
        // 预处理成功
        logger->log_info("Preprocessing success", simulator.get_curr_timestep());
    }
    else
    {
        // 预处理超时失败（严重错误）
        logger->log_fatal("Preprocessing timeout", simulator.get_curr_timestep());
    }
    logger->flush();
}


// ============================================================
// 核心仿真主循环（双速率架构）
// ============================================================
// 参数：
//   simulation_time - 仿真总时长（毫秒）
//   chunk_size - 每次规划的时间窗口（固定100ms），即 planCommTimeLimit
//
// 核心思想：双速率控制
//   - 慢速循环（规划）：每 chunk_size ms 调用一次 Entry::compute()
//   - 快速循环（执行）：每个 simulator_time_limit ms 执行一次动作
//
// 架构图：
//   仿真开始
//      ↓
//   [预处理阶段] → Entry::initialize()
//      ↓
//   [首次规划] → Entry::compute() (可能超时等待)
//      ↓
//   ┌─────────────────────────────────────────────┐
//   │         主仿真循环 (while timestep < simulation_time) │
//   │                                               │
//   │   ┌─ 检查规划器是否完成 ─┐                   │
//   │   │                      ↓                   │
//   │   │  规划完成？ → 处理新计划 → 启动新规划   │
//   │   │                      ↑                   │
//   │   │  规划未完成？ → 直接执行动作            │
//   │   └──────────────────────┘                   │
//   │              ↓                               │
//   │   ┌─ 执行一个 tick 的动作 ─┐               │
//   │   │  Executor → 碰撞检测 → 移动           │
//   │   └──────────────────────┘                   │
//   │              ↓                               │
//   │   更新任务状态 → 返回循环开始                │
//   └─────────────────────────────────────────────┘
//      ↓
//   保存结果
//
void BaseSystem::simulate(int simulation_time, int chunk_size)
{
    // 设置仿真器的时间分块参数
    simulator.set_chunk(chunk_size, simulation_time);

    // ----------------------------------------
    // 阶段1: 初始化（预处理）
    // ----------------------------------------
    initialize();

    this->simulation_time = simulation_time;

    // 获取当前仿真器状态
    vector<State> curr_states = simulator.get_current_state();
    int timestep = simulator.get_curr_timestep();

    // ----------------------------------------
    // 阶段2: 首次规划（Initial Planning）
    // ----------------------------------------
    // 在仿真开始前，调用一次完整的规划（规划所有 agent 的初始路径）
    plan_time_limit = initial_plan_time_limit;  // 首次规划时间限制

    // 创建异步任务：调用 planner_wrapper()
    std::packaged_task<bool()> task(std::bind(&BaseSystem::planner_wrapper, this));
    future = task.get_future();  // 获取未来结果

    // 记录规划开始时间
    env->plan_start_time = std::chrono::steady_clock::now();

    // 启动独立线程运行规划器
    task_td = std::thread(std::move(task));
    started = true;  // 标记：规划器正在运行

    // 清空新 agent/新任务的记录（初始化时）
    task_manager.clear_new_agents_tasks();

    // ----------------------------------------
    // 等待首次规划完成（或超时）
    // ----------------------------------------
    if (future.wait_for(std::chrono::milliseconds(initial_plan_time_limit)) == std::future_status::ready)
    {
        // 规划器在限制时间内完成
        task_td.join();      // 等待线程结束
        started = false;     // 标记：规划器空闲
        auto res = future.get();  // 获取规划结果
        logger->log_info("planner returns", timestep);
    }
    else
    {
        // 规划器超时，继续执行（使用已有的部分结果或空计划）
        logger->log_info("planner timeout", timestep);
    }

    // ----------------------------------------
    // 首次规划超时处理循环
    // 如果首次规划超时（运行时间超过 initial_plan_time_limit），
    // 主线程会一直等待直到规划器完成
    // 同时每 sim_time_limit ms 执行一次"等待"动作
    // ----------------------------------------
    while (started)
    {
        logger->log_info("planner cannot run because the previous run is still running", timestep);

        // 设置截止时间
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(simulator_time_limit);

        // 主线程执行动作（等待动作，即 agent 不移动）
        simulator.move(simulator_time_limit);

        auto move_end = std::chrono::steady_clock::now();

        // 如果动作执行时间超过限制，延长截止时间
        while (deadline < move_end)
        {
            deadline += std::chrono::milliseconds(simulator_time_limit);
        }

        // 等待直到截止时间或规划器完成
        const auto st = future.wait_until(deadline);

        if (st == std::future_status::ready)
        {
            // 规划器完成了
            task_td.join();
            started = false;
            auto res = future.get();
            logger->log_info("planner returns", timestep);
        }
        else
        {
            // 规划器仍然在运行
            logger->log_info("planner timeout", timestep);
        }
    }

    // ----------------------------------------
    // 阶段3: 主仿真循环（规划与执行并行）
    // ----------------------------------------
    std::chrono::steady_clock::time_point plan_start = std::chrono::steady_clock::now();
    int remain_communication_time = 0;  // 距离下次规划的时间

    // 主循环：直到达到仿真总时长
    while (simulator.get_curr_timestep() < simulation_time)
    {
        timestep = simulator.get_curr_timestep();

        // ----------------------------------------
        // 3.1: 检查规划器是否完成
        // ----------------------------------------
        if (remain_communication_time <= 0 && started)
        {
            // 距离下次规划的时间已到，检查规划器是否完成
            auto deadline = plan_start + std::chrono::milliseconds(min_comm_time - remain_communication_time);
            const auto st = future.wait_until(deadline);

            if (st == std::future_status::ready)
            {
                // 规划器完成了
                task_td.join();
                started = false;
                auto res = future.get();
                logger->log_info("planner returns", timestep);
            }
            else
            {
                // 规划器仍在运行（未超时）
                logger->log_info("planner timeout", timestep);
            }
        }

        // ----------------------------------------
        // 3.2: 规划器完成且通信间隔满足，启动新规划
        // ----------------------------------------
        if (!started && remain_communication_time <= 0)
        {
            // 处理新计划：
            // - 将新的多步计划转换为 staged_actions
            // - 与未完成的动作进行合并
            // - 考虑 commitment 约束（多tick动作不能中断）
            simulator.process_new_plan(process_new_plan_time_limit, simulator_time_limit, proposed_plan);

            // 同步共享环境（让新规划能读取最新状态）
            sync_shared_env();

            // 设置下次规划的时间限制
            plan_time_limit = min_comm_time;

            // 创建新的异步规划任务
            std::packaged_task<bool()> task(std::bind(&BaseSystem::planner_wrapper, this));
            future = task.get_future();

            // 记录新规划的开始时间
            env->plan_start_time = std::chrono::steady_clock::now();
            task_td = std::thread(std::move(task));

            started = true;  // 标记：规划器正在运行

            plan_start = std::chrono::steady_clock::now();  // 重置规划开始时间

            // 清空新 agent/新任务记录
            task_manager.clear_new_agents_tasks();

            // 重置通信间隔计时器
            remain_communication_time = min_comm_time;
        }

        // ----------------------------------------
        // 3.3: 规划器运行时，执行器执行动作
        // ----------------------------------------
        // 同步共享环境（给执行器看当前状态）
        simulator.sync_shared_env(env);

        // 记录执行开始时间
        auto move_start = std::chrono::steady_clock::now();

        // 执行一个时间步的动作
        // - Executor 决定 GO/STOP
        // - 应用延迟
        // - 碰撞检测
        // - 更新 agent 位置
        curr_states = simulator.move(simulator_time_limit);

        auto move_end = std::chrono::steady_clock::now();

        // 计算经过了多少个 tick
        // 公式：向上取整（确保至少1个tick）
        int elapsed_tick = std::max(1,
            ((int)std::chrono::duration_cast<std::chrono::milliseconds>(move_end - move_start).count()
                + simulator_time_limit - 1) / simulator_time_limit);

        // 减少剩余通信时间
        remain_communication_time -= elapsed_tick * simulator_time_limit;

        // ----------------------------------------
        // 3.4: 更新任务状态
        // ----------------------------------------
        // - 检查是否有任务完成
        // - 释放完成的 agent
        // - 揭示新任务（保持任务池满）
        task_manager.update_tasks(curr_states, proposed_schedule, simulator.get_curr_timestep());
    }
}


// ============================================================
// 仿真器初始化函数（预处理阶段调用）
// ============================================================
// 作用：
//   1. 设置共享环境的基本信息（地图、agent数量等）
//   2. 在独立线程中运行 Entry::initialize() 预处理
//   3. 初始化任务管理器
//   4. 揭示初始任务
//
// 超时处理：如果预处理超过 preprocess_time_limit，程序退出（exit code 124）
void BaseSystem::initialize()
{
    // ----------------------------------------
    // 设置共享环境基本信息
    // ----------------------------------------
    env->num_of_agents = num_of_agents;
    env->rows = map.rows;
    env->cols = map.cols;
    env->map = map.map;

    // 设置时间参数
    env->min_planner_communication_time = min_comm_time;  // 规划通信间隔
    env->action_time = simulator_time_limit;               // 每个 action 的执行时间
    env->max_counter = simulator.get_max_counter();         // action 的最大 tick 数

    int timestep = simulator.get_curr_timestep();

    // ----------------------------------------
    // 在独立线程中运行 Entry::initialize()
    // ----------------------------------------
    std::packaged_task<void(int)> init_task(std::bind(&Entry::initialize, planner, std::placeholders::_1));
    auto init_future = init_task.get_future();

    auto init_start_time = std::chrono::steady_clock::now();
    env->plan_start_time = init_start_time;

    // 预处理截止时间
    auto init_deadline = init_start_time + std::chrono::milliseconds(preprocess_time_limit);

    // 启动初始化线程
    std::thread init_td(std::move(init_task), preprocess_time_limit);

    // 同时初始化执行器（也在预处理时间限制内）
    simulator.initialise_executor(preprocess_time_limit);

    auto init_end_time = std::chrono::steady_clock::now();
    int diff = (int)std::chrono::duration_cast<std::chrono::milliseconds>(init_end_time - init_start_time).count();

    // ----------------------------------------
    // 等待预处理完成（或超时）
    // ----------------------------------------
    if (init_future.wait_until(init_deadline) == std::future_status::ready && diff <= preprocess_time_limit)
    {
        // 预处理成功完成
        init_td.join();
        log_preprocessing(true);
    }
    else
    {
        // 预处理超时，标记失败并退出
        init_td.detach();  // 让线程继续运行，但我们不等了
        log_preprocessing(false);
        _exit(124);  // 退出码 124 表示预处理超时
    }

    // ----------------------------------------
    // 初始化任务池
    // 揭示初始任务，使任务池达到目标大小
    // ----------------------------------------
    task_manager.reveal_tasks(timestep);

    // 同步共享环境
    sync_shared_env();

    // ----------------------------------------
    // 初始化空闲 agent 列表
    // 初始时所有 agent 都是空闲的，可以立即接受任务分配
    // ----------------------------------------
    env->new_freeagents.reserve(num_of_agents);
    for (int i = 0; i < num_of_agents; i++)
    {
        env->new_freeagents.push_back(i);
    }

    // ----------------------------------------
    // 初始化解的统计向量
    // ----------------------------------------
    solution_costs.resize(num_of_agents);
    for (int a = 0; a < num_of_agents; a++)
    {
        solution_costs[a] = 0;
    }

    // 初始化调度方案向量（初始时所有 agent 无任务）
    proposed_schedule.resize(num_of_agents, -1);
}


// ============================================================
// 保存仿真结果到 JSON 文件
// ============================================================
// 参数：
//   fileName - 输出文件路径
//   screen - 屏幕输出级别（数值越小输出越多）
//   pretty_print - 是否格式化输出
//
// 输出内容包括：
//   - 仿真统计（任务完成数、makespan、错误数等）
//   - agent 起始位置和实际路径
//   - 任务完成事件
//   - 规划路径和调度方案
//   - 各种错误信息
void BaseSystem::saveResults(const string &fileName, int screen, bool pretty_print) const
{
    json js;

    // ----------------------------------------
    // 基本信息
    // ----------------------------------------
    js["actionModel"] = "MAPF_T";
    js["version"] = "2026 LoRR";
    js["teamSize"] = num_of_agents;

    // 任务统计
    js["numTaskFinished"] = task_manager.num_of_task_finish;

    // 仿真 makespan（总时长）
    js["makespan"] = simulation_time;

    // 错误统计
    js["numPlannerErrors"] = simulator.get_number_errors();
    js["numScheduleErrors"] = task_manager.get_number_errors();
    js["numEntryTimeouts"] = total_timetous;

    // 配置信息
    js["agentMaxCounter"] = simulator.get_max_counter();
    js["outputSegmentSize"] = simulator.get_chunk_size();

    // ----------------------------------------
    // screen <= 2: 输出详细仿真数据
    // ----------------------------------------
    if (screen <= 2)
    {
        // 延迟间隔信息
        js["delayIntervals"] = simulator.delay_intervals_to_json();

        // agent 起始位置
        js["start"] = simulator.starts_to_json();
    }

    // ----------------------------------------
    // screen <= 2: 输出 agent 实际路径
    // ----------------------------------------
    if (screen <= 2)
    {
        js["actualPaths"] = simulator.actual_path_to_json();
    }

    // ----------------------------------------
    // screen <= 2: 输出任务和事件
    // ----------------------------------------
    if (screen <= 2)
    {
        // 任务完成事件
        // 格式: [timestep, agent_id, task_id, seq_id]
        json event = json::array();
        for (auto e : task_manager.events)
        {
            json ev = json::array();
            int timestep, agent_id, task_id, seq_id;
            std::tie(timestep, agent_id, task_id, seq_id) = e;
            ev.push_back(timestep);
            ev.push_back(agent_id);
            ev.push_back(task_id);
            ev.push_back(seq_id);
            event.push_back(ev);
        }
        js["events"] = event;

        // 所有任务详情
        json tasks = task_manager.to_json(map.cols);
        js["tasks"] = tasks;
    }

    // ----------------------------------------
    // screen <= 1: 输出最详细的规划信息
    // ----------------------------------------
    if (screen <= 1)
    {
        // 规划路径
        js["plannerPaths"] = simulator.planned_path_to_json();

        // 规划耗时
        json planning_times = json::array();
        for (double time : planner_times)
            planning_times.push_back(time);
        js["plannerTimes"] = planning_times;

        // 动作错误
        js["errors"] = simulator.action_errors_to_json();

        // ----------------------------------------
        // 实际调度记录
        // 格式: "timestep1:task_id1,timestep2:task_id2,..."
        // ----------------------------------------
        json aschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto &schedule : task_manager.actual_schedule[i])
            {
                if (!first)
                    schedules += ",";
                else
                    first = false;

                schedules += std::to_string(schedule.first);
                schedules += ":";
                int tid = schedule.second;
                schedules += std::to_string(tid);
            }
            aschedules.push_back(schedules);
        }
        js["actualSchedule"] = aschedules;

        // ----------------------------------------
        // 规划时的调度方案（计划调度 vs 实际执行）
        // ----------------------------------------
        json pschedules = json::array();
        for (int i = 0; i < num_of_agents; i++)
        {
            std::string schedules;
            bool first = true;
            for (const auto &schedule : task_manager.planner_schedule[i])
            {
                if (!first)
                    schedules += ",";
                else
                    first = false;

                schedules += std::to_string(schedule.first);
                schedules += ":";
                int tid = schedule.second;
                schedules += std::to_string(tid);
            }
            pschedules.push_back(schedules);
        }
        js["plannerSchedule"] = pschedules;

        // ----------------------------------------
        // 调度错误记录
        // ----------------------------------------
        json schedule_errors = json::array();
        for (auto error : task_manager.schedule_errors)
        {
            std::string error_msg;
            int t_id, agent1, agent2, timestep;
            std::tie(error_msg, t_id, agent1, agent2, timestep) = error;

            json e = json::array();
            e.push_back(t_id);
            e.push_back(agent1);
            e.push_back(agent2);
            e.push_back(timestep);
            e.push_back(error_msg);
            schedule_errors.push_back(e);
        }
        js["scheduleErrors"] = schedule_errors;
    }

    // ----------------------------------------
    // 写入文件
    // ----------------------------------------
    std::ofstream f(fileName, std::ios_base::trunc | std::ios_base::out);
    if (pretty_print)
    {
        // 格式化输出（缩进）
        f << std::setw(4) << js;
    }
    else
    {
        // 紧凑输出
        f << js.dump();
    }
}
