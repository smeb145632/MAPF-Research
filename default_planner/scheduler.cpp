#include "scheduler.h"

namespace DefaultPlanner{

// 随机数生成器（用于调度算法中的随机决策）
std::mt19937 mt;

// 跨时间步跟踪所有空闲 agent
// 空闲 agent：已完成上一个任务、可以接受新任务分配的 agent
std::unordered_set<int> free_agents;

// 跨时间步跟踪所有未分配的任务
// 未分配任务：已揭示但还未被任何 agent 认领的任务
std::unordered_set<int> free_tasks;


/**
 * schedule_initialize — 调度器预处理初始化
 *
 * @param preprocess_time_limit 预处理时间限制（毫秒）
 * @param env 共享环境指针
 *
 * 职责：
 *   - 初始化启发式函数（供后续调度和规划使用）
 *   - 设置随机数种子（保证结果可复现）
 */
void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // 初始化启发式函数（用于计算 agent 到各位置的距离）
    DefaultPlanner::init_heuristics(env);

    // 设置随机数种子
    mt.seed(0);
    return;
}


/**
 * schedule_plan — 默认调度算法（贪心策略）
 *
 * @param time_limit 本次调度的时间预算（毫秒）
 * @param proposed_schedule 输出：任务分配方案，proposed_schedule[i] = agent i 被分配的任务 ID（-1 表示无任务）
 * @param env 共享环境指针
 *
 * 算法思路（贪心）：
 *   1. 对每个空闲 agent，遍历所有未分配任务
 *   2. 计算 agent 完成每个任务的 makespan（预计完成时间）
 *   3. 选择 makespan 最短的任务分配给该 agent
 *   4. 重复直到所有 agent 被分配或时间耗尽
 *
 * 注意：
 *   - 使用最多一半的时间预算做调度，另一半留给路径规划
 *   - makespan = agent 从当前位置出发，按顺序完成 task 所有 errands 所需的距离
 */
void schedule_plan(int time_limit, std::vector<int> & proposed_schedule, SharedEnvironment* env)
{
    cout << "number of new free agents: " << env->new_freeagents.size()
         << " number of new tasks: " << env->new_tasks.size() << endl;

    // 时间预算：最多用一半的时间做调度，其余留给 path planner
    TimePoint endtime = std::chrono::steady_clock::now()
                       + std::chrono::milliseconds(time_limit);

    // ----------------------------------------
    // 将新空闲的 agents 和新揭示的任务加入跟踪集合
    // free_agents 和 free_tasks 会跨时间步累积
    // ----------------------------------------
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // ----------------------------------------
    // 遍历所有空闲 agent，为每个分配一个任务
    // ----------------------------------------
    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        // 检查是否超时，超时则停止分配
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }

        int i = *it;  // agent ID

        // 确保该 agent 当前没有被分配任务
        assert(env->curr_task_schedule[i] == -1);

        min_task_i = -1;           // 最佳任务 ID（初始为无）
        min_task_makespan = INT_MAX;  // 最短 makespan（初始为无穷大）
        count = 0;                 // 已评估的任务数（用于定期超时检查）

        // ----------------------------------------
        // 遍历所有未分配任务，找 makespan 最短的那个
        // ----------------------------------------
        for (int t_id : free_tasks)
        {
            // 每评估 10 个任务检查一次超时
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }

            dist = 0;                         // 累计距离
            c_loc = env->curr_states.at(i).location;  // agent i 当前位置

            // 遍历任务的所有 errands，计算完成该任务的总距离
            // makespan = 从当前位置出发，按顺序访问所有 errand 位置的距离之和
            for (int loc : env->task_pool[t_id].locations)
            {
                // get_h 是启发式距离函数（ Manhattan 距离或自定义）
                dist += DefaultPlanner::get_h(env, c_loc, loc);
                c_loc = loc;  // 更新当前位置为刚访问的 errand 位置
            }

            // 如果这个任务的 makespan 更短，更新最佳选择
            if (dist < min_task_makespan)
            {
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;
        }

        // ----------------------------------------
        // 将最佳任务分配给 agent i
        // ----------------------------------------
        if (min_task_i != -1)
        {
            // 更新调度方案：agent i -> task min_task_i
            proposed_schedule[i] = min_task_i;

            // 从空闲集合中移除（已分配）
            it = free_agents.erase(it);         // erase 返回下一个迭代器
            free_tasks.erase(min_task_i);
        }
        // 没有可用任务分配给该 agent
        else
        {
            proposed_schedule[i] = -1;  // 标记为无任务
            it++;
        }
    }

    return;
}


/* ============================================================
 * 以下为调试代码（NDEBUG 时不启用）
 * ============================================================
// cout << "调度耗时: " << ((float)(clock() - start))/CLOCKS_PER_SEC << endl;
// cout << "新空闲 agents: " << env->new_freeagents.size() << " 新任务: " << env->new_tasks.size() << endl;
// cout << "空闲 agents 总数: " << free_agents.size() << " 空闲任务总数: " << free_tasks.size() << endl;
 */
}
