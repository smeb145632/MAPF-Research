// Default scheduler baseline implementation.
//
// Keeps persistent sets of free agents and free tasks across timesteps, then
// greedily matches each free agent to the task with minimum heuristic distance
// makespan. This is intentionally simple and serves as a reference scheduler.
//
// H23 Enhancement: Track task age in free_tasks. Old tasks get distance penalty
// to encourage reassignment when initial assignment was suboptimal.
// H26 Enhancement: Batch assignment + global re-evaluation every 50 steps.
// H27 Enhancement: Efficiency-driven task switching for low-efficiency agents.
// H31 Enhancement: Wait-time based priority switching - trigger on consecutive
//   blocked steps rather than low efficiency ratio (more stable signal).

#include "scheduler.h"
#include "heuristics.h"

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_map<int, int> task_age_map;  // task_id -> timestep when task entered free_tasks

// H26: Batch assignment tracking
std::unordered_map<int, int> agent_assigned_task;  // agent_id -> task_id (currently executing)
std::unordered_map<int, int> task_start_time;     // task_id -> timestep when task was assigned

// H26: Map-adaptive penalty multiplier (0.0 to 1.0)
double REASSIGN_AGE_PENALTY_MULTIPLIER = 1.0;

// H26: Global re-evaluation state (defined here, declared extern in .h)
int last_reassess_time = 0;

// H27: Dynamic task switching (efficiency-driven reassignment)
std::unordered_map<int, int> agent_last_switch_time;
const double EFFICIENCY_SWITCH_THRESHOLD = 1.5;
const int TASK_SWITCH_COOLDOWN = 100;

// H31: Wait-time based switching
// Track consecutive steps an agent has made zero progress toward goal
std::unordered_map<int, int> agent_consecutive_wait;
std::unordered_map<int, int> agent_prev_remaining;  // agent -> previous step remaining distance
const int WAIT_THRESHOLD = 15;
// Minimum steps between task switches (cooldown to prevent thrashing)
const int TASK_SWITCH_COOLDOWN_H31 = 80;


void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    mt.seed(0);
    free_agents.clear();
    free_tasks.clear();
    task_age_map.clear();
    agent_assigned_task.clear();
    task_start_time.clear();
    last_reassess_time = 0;
    agent_last_switch_time.clear();
    agent_consecutive_wait.clear();
    agent_prev_remaining.clear();

    // H26: Map-adaptive feature detection
    if (env->num_of_agents > 0 && !env->new_tasks.empty())
    {
        const int SAMPLE_SIZE = std::min(20, (int)env->new_tasks.size());
        long long total_dist = 0;
        int sample_count = 0;
        std::vector<int> task_ids(env->new_tasks.begin(), env->new_tasks.end());
        std::shuffle(task_ids.begin(), task_ids.end(), mt);
        for (int s = 0; s < SAMPLE_SIZE; s++)
        {
            int t_id = task_ids[s];
            int t_loc = env->task_pool[t_id].locations[0];
            for (int a = 0; a < std::min(5, env->num_of_agents); a++)
            {
                int a_loc = env->curr_states[a].location;
                total_dist += DefaultPlanner::get_h(env, a_loc, t_loc);
                sample_count++;
            }
        }
        double avg_dist = (double)total_dist / (double)sample_count;
        if (avg_dist < 40.0)
            REASSIGN_AGE_PENALTY_MULTIPLIER = 0.4;
        else
            REASSIGN_AGE_PENALTY_MULTIPLIER = 1.0;
    }
    else
    {
        REASSIGN_AGE_PENALTY_MULTIPLIER = 1.0;
    }
    return;
}

void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);
    int current_time = env->curr_timestep;

    // H28/H31: Track agent task assignments and wait state
    for (int a = 0; a < env->num_of_agents; a++)
    {
        int task_id = env->curr_task_schedule[a];
        if (task_id >= 0)
        {
            if (agent_assigned_task.find(a) == agent_assigned_task.end())
            {
                agent_assigned_task[a] = task_id;
                task_start_time[task_id] = current_time;
                // H31: Initialize wait tracking for newly assigned agent
                int goal_loc = env->task_pool[task_id].locations.back();
                int init_rem = DefaultPlanner::get_h(env, env->curr_states[a].location, goal_loc);
                agent_prev_remaining[a] = init_rem;
                agent_consecutive_wait[a] = 0;
            }
        }
        else
        {
            auto it = agent_assigned_task.find(a);
            if (it != agent_assigned_task.end())
                agent_assigned_task.erase(it);
            agent_consecutive_wait[a] = 0;
        }
    }
    
    bool do_reassess = (current_time - last_reassess_time >= REASSESS_INTERVAL);
    
    if (do_reassess && !agent_assigned_task.empty())
    {
        last_reassess_time = current_time;
        for (auto& at : agent_assigned_task)
        {
            int task_id = at.second;
            auto start_it = task_start_time.find(task_id);
            if (start_it != task_start_time.end())
            {
                int time_elapsed = current_time - start_it->second;
                if (time_elapsed > 20)
                {
                    int agent_id = at.first;
                    int goal_loc = env->task_pool[task_id].locations.back();
                    int agent_loc = env->curr_states[agent_id].location;
                    int remaining = DefaultPlanner::get_h(env, agent_loc, goal_loc);
                    int initial_dist = DefaultPlanner::get_h(env, env->task_pool[task_id].locations[0], goal_loc);
                    if (initial_dist > 0 && remaining > initial_dist * 8 / 10)
                        task_age_map[task_id] = current_time - TASK_FORCE_REASSIGN_THRESHOLD - 10;
                }
            }
        }
    }

    // H31: Update wait tracking - count consecutive steps with no progress
    for (int a = 0; a < env->num_of_agents; a++)
    {
        int task_id = env->curr_task_schedule[a];
        if (task_id < 0) {
            agent_consecutive_wait[a] = 0;
            continue;
        }
        int agent_loc = env->curr_states[a].location;
        int goal_loc = env->task_pool[task_id].locations.back();
        int remaining = DefaultPlanner::get_h(env, agent_loc, goal_loc);
        auto prev_it = agent_prev_remaining.find(a);
        if (prev_it != agent_prev_remaining.end())
        {
            if (remaining >= prev_it->second)
                agent_consecutive_wait[a]++;
            else
                agent_consecutive_wait[a] = 0;
        }
        agent_prev_remaining[a] = remaining;
    }

    // H31: Wait-time based task switching
    // Trigger when agent has been blocked for WAIT_THRESHOLD consecutive steps
    if (do_reassess && !free_agents.empty() && !agent_assigned_task.empty())
    {
        for (int a : free_agents)
        {
            auto switch_it = agent_last_switch_time.find(a);
            if (switch_it != agent_last_switch_time.end() && (current_time - switch_it->second) < TASK_SWITCH_COOLDOWN_H31)
                continue;
            int curr_task_id = env->curr_task_schedule[a];
            if (curr_task_id < 0) continue;
            int wait_count = 0;
            auto wait_it = agent_consecutive_wait.find(a);
            if (wait_it != agent_consecutive_wait.end())
                wait_count = wait_it->second;
            if (wait_count < WAIT_THRESHOLD) continue;
            int agent_loc = env->curr_states[a].location;
            int best_free_task = -1;
            double best_score = -1.0;
            for (int free_task_id : free_tasks)
            {
                int t_loc = env->task_pool[free_task_id].locations[0];
                int travel_dist = DefaultPlanner::get_h(env, agent_loc, t_loc);
                int task_goal = env->task_pool[free_task_id].locations.back();
                int task_dist = DefaultPlanner::get_h(env, t_loc, task_goal);
                int new_total_dist = travel_dist + task_dist;
                double score = (new_total_dist > 0) ? (1000.0 / (double)new_total_dist) : 1000.0;
                auto age_it = task_age_map.find(free_task_id);
                if (age_it != task_age_map.end())
                {
                    int task_age = current_time - age_it->second;
                    if (task_age > 50) score *= 0.8;
                }
                if (score > best_score) { best_score = score; best_free_task = free_task_id; }
            }
            if (best_free_task != -1)
            {
                proposed_schedule[a] = best_free_task;
                task_age_map[curr_task_id] = current_time - TASK_REASSIGN_THRESHOLD - 10;
                free_tasks.erase(best_free_task);
                agent_assigned_task[a] = best_free_task;
                task_start_time[best_free_task] = current_time;
                int goal_loc = env->task_pool[best_free_task].locations.back();
                int init_rem = DefaultPlanner::get_h(env, agent_loc, goal_loc);
                agent_prev_remaining[a] = init_rem;
                agent_consecutive_wait[a] = 0;
                task_start_time.erase(curr_task_id);
                agent_last_switch_time[a] = current_time;
                free_agents.erase(a);
            }
        }
    }

    // H27: Efficiency-driven task switching (fallback for non-waiting agents)
    if (do_reassess && !free_agents.empty() && !agent_assigned_task.empty())
    {
        for (int a : free_agents)
        {
            auto switch_it = agent_last_switch_time.find(a);
            if (switch_it != agent_last_switch_time.end() && (current_time - switch_it->second) < TASK_SWITCH_COOLDOWN)
                continue;
            int curr_task_id = env->curr_task_schedule[a];
            if (curr_task_id < 0) continue;
            int agent_loc = env->curr_states[a].location;
            int goal_loc = env->task_pool[curr_task_id].locations.back();
            int remaining = DefaultPlanner::get_h(env, agent_loc, goal_loc);
            auto start_it = task_start_time.find(curr_task_id);
            int time_elapsed = (start_it != task_start_time.end()) ? (current_time - start_it->second) : 1;
            int progress = DefaultPlanner::get_h(env, env->task_pool[curr_task_id].locations[0], goal_loc) - remaining;
            double curr_efficiency = (time_elapsed > 0) ? ((double)progress / (double)time_elapsed) : 0.0;
            if (curr_efficiency < 0.3)
            {
                int best_free_task = -1;
                double best_efficiency = curr_efficiency * EFFICIENCY_SWITCH_THRESHOLD;
                for (int free_task_id : free_tasks)
                {
                    int t_loc = env->task_pool[free_task_id].locations[0];
                    int travel_dist = DefaultPlanner::get_h(env, agent_loc, t_loc);
                    int task_goal = env->task_pool[free_task_id].locations.back();
                    int task_dist = DefaultPlanner::get_h(env, t_loc, task_goal);
                    int new_total_dist = travel_dist + task_dist;
                    double free_efficiency = (new_total_dist > 0) ? (1.0 / (double)new_total_dist) : 1.0;
                    if (free_efficiency > best_efficiency) { best_efficiency = free_efficiency; best_free_task = free_task_id; }
                }
                if (best_free_task != -1 && best_efficiency > curr_efficiency * EFFICIENCY_SWITCH_THRESHOLD)
                {
                    proposed_schedule[a] = best_free_task;
                    task_age_map[curr_task_id] = current_time - TASK_REASSIGN_THRESHOLD - 10;
                    free_tasks.erase(best_free_task);
                    agent_assigned_task[a] = best_free_task;
                    task_start_time[best_free_task] = current_time;
                    task_start_time.erase(curr_task_id);
                    agent_last_switch_time[a] = current_time;
                    free_agents.erase(a);
                }
            }
        }
    }

    for (int t_id : env->new_tasks)
        if (free_tasks.find(t_id) == free_tasks.end())
            task_age_map[t_id] = current_time;
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    int min_task_i, min_task_makespan, dist, c_loc, count;
    clock_t start = clock();

    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime) break;
        int i = *it;
        assert(env->curr_task_schedule[i] == -1);
        min_task_i = -1;
        min_task_makespan = INT_MAX;
        count = 0;
        for (int t_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;
            dist = 0;
            c_loc = env->curr_states.at(i).location;
            int travel_dist = DefaultPlanner::get_h(env, c_loc, env->task_pool[t_id].locations[0]);
            int task_internal = 0;
            int prev_loc = env->task_pool[t_id].locations[0];
            for (size_t k = 1; k < env->task_pool[t_id].locations.size(); k++){
                task_internal += DefaultPlanner::get_h(env, prev_loc, env->task_pool[t_id].locations[k]);
                prev_loc = env->task_pool[t_id].locations[k];
            }
            dist = travel_dist + task_internal / 2;
            auto age_it = task_age_map.find(t_id);
            if (age_it != task_age_map.end())
            {
                int task_age = current_time - age_it->second;
                if (task_age > TASK_FORCE_REASSIGN_THRESHOLD && TASK_FORCE_REASSIGN_THRESHOLD > 0)
                    dist += 10000;
                else if (task_age > TASK_REASSIGN_THRESHOLD)
                {
                    int scaled_penalty = (int)((task_age - TASK_REASSIGN_THRESHOLD) * REASSIGN_AGE_PENALTY_PER_STEP_MAX * REASSIGN_AGE_PENALTY_MULTIPLIER);
                    dist += scaled_penalty;
                }
            }
            if (dist < min_task_makespan){ min_task_i = t_id; min_task_makespan = dist; }
            count++;
        }
        if (min_task_i != -1){
            proposed_schedule[i] = min_task_i;
            it = free_agents.erase(it);
            free_tasks.erase(min_task_i);
            task_age_map.erase(min_task_i);
        } else {
            proposed_schedule[i] = -1;
            it++;
        }
    }
    return;
}

}