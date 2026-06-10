#include <cassert>
#include <cmath>
#include <iostream>
#include <list>
#include <vector>

#include "Tasks.h"
#include "SharedEnv.h"
#include "submission_heuristics.h"

#define private public
#include "TaskScheduler.h"
#undef private

namespace {

void require(bool condition, const char* message)
{
    if (!condition)
    {
        std::cerr << message << std::endl;
        std::exit(1);
    }
}

SharedEnvironment make_line_env(int cols, int agents)
{
    SharedEnvironment env;
    env.rows = 1;
    env.cols = cols;
    env.num_of_agents = agents;
    env.map.assign(cols, 0);
    env.curr_states.assign(agents, State());
    env.curr_task_schedule.assign(agents, -1);
    return env;
}

Task make_task(int id, std::initializer_list<int> locations, int revealed)
{
    return Task(id, std::list<int>(locations.begin(), locations.end()), revealed);
}

} // namespace

int main()
{
    {
        SharedEnvironment* env = new SharedEnvironment(make_line_env(30, 1));
        env->curr_states[0] = State(5, 0, 0);
        env->task_pool[7] = make_task(7, {0, 10}, 0);

        TaskScheduler scheduler(env);
        SubmissionPlanner::init_heuristics(env);
        scheduler.task_start_time[7] = 95;

        const double efficiency = scheduler.get_agent_efficiency(0, 7, 100);
        require(std::fabs(efficiency - 1.0) < 1e-9, "efficiency should use task assignment start time");
    }

    {
        SharedEnvironment* env = new SharedEnvironment(make_line_env(30, 2));
        env->curr_states[0] = State(19, 0, 0);
        env->curr_states[1] = State(1, 0, 0);
        env->curr_task_schedule = {1, 2};
        env->task_pool[1] = make_task(1, {0, 5}, 0);
        env->task_pool[2] = make_task(2, {20, 25}, 0);

        TaskScheduler scheduler(env);
        SubmissionPlanner::init_heuristics(env);
        scheduler.agent_assigned_task[0] = 1;
        scheduler.agent_assigned_task[1] = 2;
        scheduler.task_start_time[1] = 90;
        scheduler.task_start_time[2] = 90;
        scheduler.agent_task_start_location[0] = 0;
        scheduler.agent_task_start_location[1] = 20;

        std::vector<int> proposed = {1, 2};
        scheduler.efficient_task_swap(proposed, 100, 1);

        require(proposed[0] == 1, "swap should keep agent 0 on original task when assignment start location has no gain");
        require(proposed[1] == 2, "swap should keep agent 1 on original task when assignment start location has no gain");
    }

    {
        SharedEnvironment* env = new SharedEnvironment(make_line_env(30, 1));
        env->curr_states[0] = State(3, 0, 0);
        env->curr_task_schedule = {1};
        env->task_pool[1] = make_task(1, {0, 5}, 0);
        env->task_pool[1].idx_next_loc = 1;
        env->task_pool[1].agent_assigned = 0;
        env->task_pool[2] = make_task(2, {10, 15}, 0);

        TaskScheduler scheduler(env);
        std::vector<int> proposed = {2};
        scheduler.sanitize_proposed_schedule(proposed);

        require(proposed[0] == 1, "opened current task must be preserved");
    }

    {
        SharedEnvironment* env = new SharedEnvironment(make_line_env(30, 2));
        env->curr_states[0] = State(3, 0, 0);
        env->curr_states[1] = State(8, 0, 0);
        env->curr_task_schedule = {-1, 1};
        env->task_pool[1] = make_task(1, {0, 5}, 0);
        env->task_pool[1].idx_next_loc = 1;
        env->task_pool[1].agent_assigned = 1;

        TaskScheduler scheduler(env);
        std::vector<int> proposed = {1, 1};
        scheduler.sanitize_proposed_schedule(proposed);

        require(proposed[0] == -1, "task opened by another agent must not be assigned");
        require(proposed[1] == 1, "owner of opened task should keep it");
    }

    std::cout << "task_scheduler_test passed" << std::endl;
    return 0;
}
