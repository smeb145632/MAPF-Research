#include "Entry.h"
#include "Tasks.h"
#include "utils.h"
#include "heuristics.h"

// 初始化函数将在预处理阶段被竞赛系统调用。
// 实现规划器和调度器的初始化函数，用于加载或计算辅助数据。
// 注意，该函数运行直到达到 preprocess_time_limit（毫秒）才结束。
// 这是一个离线步骤，完成后评估开始。
void Entry::initialize(int preprocess_time_limit)
{
    scheduler->initialize(preprocess_time_limit);
    planner->initialize(preprocess_time_limit);
}

//The compute function will be called by competition system on each timestep.
//It computes:
//  1. 一个调度方案，指定哪个 agent 完成哪个任务
//  2. 一个下一步动作，指定每个 agent 在下一个时间步应如何移动
//NB: the parameter time_limit is specified in milliseconds.
void Entry::compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule)
{
    //call the task scheduler to assign tasks to agents
    scheduler->plan(time_limit,proposed_schedule);

    //then update the first unfinished errand/location of tasks for planner reference
    update_goal_locations(proposed_schedule);
    
    //call the planner to compute the actions
    planner->plan(time_limit,plan);

}

// 根据提议的调度方案设置每个 agent 的下一个目标位置
void Entry::update_goal_locations(std::vector<int> & proposed_schedule)
{
    // 记录提议的调度方案，以便我们可以告诉规划器
    env->curr_task_schedule = proposed_schedule;

    // 每个任务的第一个未完成的差事/位置是被分配的 agent 的下一个目标
    for (size_t i = 0; i < proposed_schedule.size(); i++)
    {
        env->goal_locations[i].clear();
        int t_id = proposed_schedule[i];
        if (t_id == -1)
            continue;

        int i_loc = env->task_pool[t_id].idx_next_loc;
        env->goal_locations[i].push_back({env->task_pool[t_id].locations.at(i_loc), env->task_pool[t_id].t_revealed});
    }
    return;
}