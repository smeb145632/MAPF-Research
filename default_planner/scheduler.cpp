// Default scheduler baseline implementation.
//
// Keeps persistent sets of free agents and free tasks across timesteps, then
// greedily matches each free agent to the task with minimum heuristic distance
// makespan. This is intentionally simple and serves as a reference scheduler.
//
// H23 Enhancement: Track task age in free_tasks. Old tasks get distance penalty
// to encourage reassignment when initial assignment was suboptimal.

#include "scheduler.h"
#include "heuristics.h"

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_map<int, int> task_age_map;  // task_id -> timestep when task entered free_tasks

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    mt.seed(0);
    free_agents.clear();
    free_tasks.clear();
    task_age_map.clear();
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    int current_time = env->curr_timestep;

    // Add new tasks to free_tasks and track their age
    for (int t_id : env->new_tasks)
    {
        if (free_tasks.find(t_id) == free_tasks.end())
        {
            // New task entering free pool - record its entry timestep
            task_age_map[t_id] = current_time;
        }
    }
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // H16: Travel + task internal distance
            int travel_dist = DefaultPlanner::get_h(env, c_loc, env->task_pool[t_id].locations[0]);
            
            int task_internal = 0;
            int prev_loc = env->task_pool[t_id].locations[0];
            for (size_t k = 1; k < env->task_pool[t_id].locations.size(); k++){
                task_internal += DefaultPlanner::get_h(env, prev_loc, env->task_pool[t_id].locations[k]);
                prev_loc = env->task_pool[t_id].locations[k];
            }
            
            dist = travel_dist + task_internal / 2;

            // H23: Age-based distance penalty
            // Old tasks get penalized so they're more likely to be reassigned
            auto age_it = task_age_map.find(t_id);
            if (age_it != task_age_map.end())
            {
                int task_age = current_time - age_it->second;
                
                // Force-reassign very old tasks (bypass distance, pick them last)
                if (task_age > TASK_FORCE_REASSIGN_THRESHOLD && TASK_FORCE_REASSIGN_THRESHOLD > 0)
                {
                    // Add huge penalty so this task is only chosen if nothing better exists
                    dist += 10000;
                }
                else if (task_age > TASK_REASSIGN_THRESHOLD)
                {
                    // Penalize old tasks to encourage reassignment
                    dist += (task_age - TASK_REASSIGN_THRESHOLD) * REASSIGN_AGE_PENALTY_PER_STEP;
                }
            }

            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;            
        }

        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            task_age_map.erase(min_task_i);  // Task no longer in free pool
        }
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }

    // Clean up task_age_map for tasks that no longer exist
    // (completed tasks that weren't assigned this round - rare but possible)
    // We don't explicitly clean here - entries are erased when tasks are assigned.
    // Tasks that complete while in free_tasks will have stale entries cleaned on next schedule call.
    // Safe approach: only erase entries we just assigned (already done above).
    
    return;
}

}