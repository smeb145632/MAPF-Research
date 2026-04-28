// Default scheduler baseline implementation.
//
// Keeps persistent sets of free agents and free tasks across timesteps, then
// greedily matches each free agent to the task with minimum heuristic distance
// makespan. This is intentionally simple and serves as a reference scheduler.

#include "scheduler.h"

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    // cout<<"schedule initialise limit" << preprocess_time_limit<<endl;
    // DefaultPlanner::init_heuristics(env);
    mt.seed(0);
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    //use at most half of time_limit to compute schedule, -10 for timing error tolerance
    //so that the remainning time are left for path planner
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    // cout<<"schedule plan limit" << time_limit <<endl;

    // the default scheduler keep track of all the free agents and unassigned (=free) tasks across timesteps
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    // iterate over the free agents to decide which task to assign to each of them
    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        //keep assigning until timeout
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        assert(env->curr_task_schedule[i] == -1);
            
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;

        // iterate over all the unassigned tasks to find the one with the minimum makespan for agent i
        for (int t_id : free_tasks)
        {
            //check for timeout every 10 task evaluations
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime)
            {
                break;
            }
            dist = 0;
            c_loc = env->curr_states.at(i).location;

            // H16 improvement: consider BOTH travel distance AND task internal distance
            // This helps on paris/city maps where manhattan heuristic has high error.
            // We want to avoid assigning all long tasks to the same "closest" agent.
            
            // Part 1: Travel distance from agent to first task waypoint
            int travel_dist = DefaultPlanner::get_h(env, c_loc, env->task_pool[t_id].locations[0]);
            
            // Part 2: Task internal distance (path through all waypoints)
            int task_internal = 0;
            int prev_loc = env->task_pool[t_id].locations[0];
            for (size_t k = 1; k < env->task_pool[t_id].locations.size(); k++){
                task_internal += DefaultPlanner::get_h(env, prev_loc, env->task_pool[t_id].locations[k]);
                prev_loc = env->task_pool[t_id].locations[k];
            }
            
            // H16: Combined cost = travel + task internal
            // Using a weighted sum: travel gets full weight, task internal gets smaller weight
            // This prevents long tasks from being unfairly assigned to just "closest" agents
            dist = travel_dist + task_internal / 2;

            // update the new minimum makespan
            if (dist < min_task_makespan){
                min_task_i = t_id;
                min_task_makespan = dist;
            }
            count++;            
        }

        // assign the best free task to the agent i (assuming one exists)
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
        }
        // nothing to assign
        else{
            proposed_schedule[i] = -1;
            it++;
        }
    }
    // #ifndef NDEBUG
    // cout << "Time Usage: " <<  ((float)(clock() - start))/CLOCKS_PER_SEC <<endl;
    // cout << "new free agents: " << env->new_freeagents.size() << " new tasks: "<< env->new_tasks.size() <<  endl;
    // cout << "free agents: " << free_agents.size() << " free tasks: " << free_tasks.size() << endl;
    // #endif
    return;
}
}
