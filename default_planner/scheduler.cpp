// Default scheduler baseline implementation.
//
// Keeps persistent sets of free agents and free tasks across timesteps, then
// greedily matches each free agent to the task with minimum heuristic distance
// makespan. This is intentionally simple and serves as a reference scheduler.
//
// H23 Enhancement: Track task age in free_tasks. Old tasks get distance penalty
// to encourage reassignment when initial assignment was suboptimal.
// H26/H27/H31: EGTS (Efficient Guided Task Swap) - global re-evaluation
//   and efficiency-driven task switching for low-efficiency agents.

#include "scheduler.h"
#include "heuristics.h"
#include <cstdint>

namespace DefaultPlanner{

std::mt19937 mt;
std::unordered_set<int> free_agents;
std::unordered_set<int> free_tasks;
std::unordered_map<int, int> task_age_map;  // task_id -> timestep when task entered free_tasks

// H26: Batch assignment tracking
std::unordered_map<int, int> agent_assigned_task;  // agent_id -> task_id

// H26: Global re-evaluation state
int last_reassess_time = 0;
int schedule_reassess_count = 0;

// EGTS constants
const double EGTS_EFFICIENCY_THRESHOLD = 0.3;
const double EGTS_SWAP_GAIN_THRESHOLD = 0.2;
const int EGTS_AGENT_COOLDOWN = 15;   // in simulation timesteps
const int EGTS_TASK_COOLDOWN = 20;    // in simulation timesteps

// EGTS state
std::unordered_map<int, int> egts_last_swap_call_count;
std::unordered_map<int, int> egts_task_last_swap_call_count;
int egts_swap_count = 0;
int egts_call_count = 0;
// int egts_swap_attempts = 0;      // H31: swap attempts that passed gain threshold
// int egts_swap_rejects_opened = 0; // H31: rejected due to already_opened
int schedule_plan_call_count = 0;

// ================================================================
// EGTS helper: compute cumulative segment costs for a task
// ================================================================
std::vector<int> compute_segment_costs(const Task& task, SharedEnvironment* env) {
    std::vector<int> cum;
    cum.push_back(0);
    int sum = 0;
    for (size_t k = 1; k < task.locations.size(); k++) {
        sum += DefaultPlanner::get_h(env, task.locations[k-1], task.locations[k]);
        cum.push_back(sum);
    }
    return cum;
}

// ================================================================
// EGTS helper: compute actual progress cost for (agent, task)
// ================================================================
int compute_actual_progress_cost(int agent_id, const Task& task, SharedEnvironment* env) {
    if (task.locations.empty()) return 0;
    int idx = task.idx_next_loc;
    int agent_loc = env->curr_states[agent_id].location;
    auto cum = compute_segment_costs(task, env);
    int total = cum.back();
    if (idx <= 0) {
        int dist_to_goal = DefaultPlanner::get_h(env, agent_loc, task.locations[0]);
        return std::max(0, total - dist_to_goal);
    }
    if (idx >= (int)task.locations.size()) return cum.back();
    int completed = 0;
    for (int k = 1; k < idx; k++)
        completed += DefaultPlanner::get_h(env, task.locations[k-1], task.locations[k]);
    int next_loc = task.locations[idx];
    int prev_loc = task.locations[idx-1];
    int seg = DefaultPlanner::get_h(env, prev_loc, next_loc);
    int dist_to_next = DefaultPlanner::get_h(env, agent_loc, next_loc);
    int traveled = seg - dist_to_next;
    int actual = completed + traveled;
    if (actual < 0) actual = 0;
    if (actual > total) actual = total;
    return actual;
}

// ================================================================
// EGTS helper: compute agent efficiency for current task
// Efficiency = actual_progress / ideal_progress (with penalty for behind-schedule)
// ================================================================
double get_agent_efficiency(int a, int curr_task_id, SharedEnvironment* env, int current_time) {
    if (curr_task_id < 0) return 0.0;
    const Task& task = env->task_pool[curr_task_id];
    int time_elapsed = current_time - task.t_revealed;
    if (time_elapsed <= 0) return 0.0;
    auto cum = compute_segment_costs(task, env);
    int total = cum.back();
    int ideal_progress = std::min(total, time_elapsed);
    int actual_progress = compute_actual_progress_cost(a, task, env);
    double eff = (ideal_progress > 0) ? ((double)actual_progress / (double)ideal_progress) : 0.0;
    // Penalize if behind schedule
    if (actual_progress < ideal_progress) {
        int behind_cost = ideal_progress - actual_progress;
        eff *= std::pow(0.95, behind_cost);
    }
    return eff;
}

// ================================================================
// EGTS efficient_task_swap - O(n²) swap search with pre-filtering
//
// Key fixes vs 2500-branch version:
// - Reassess interval is now call-based (REASSESS_CALL_INTERVAL), not time-based
// - cooldown in SIMULATION timesteps (not inflated by window_size)
// ================================================================
void efficient_task_swap(std::vector<int>& proposed_schedule, SharedEnvironment* env,
                          int current_time) {
    egts_call_count++;
    if (agent_assigned_task.empty()) return;

    // Step 1: collect all (agent, task) pairs
    std::vector<std::pair<int, int>> all_pairs;
    for (auto& at : agent_assigned_task)
        if (at.second >= 0) all_pairs.push_back(at);
    if (all_pairs.size() < 2) return;

    // Step 2: pre-filter - compute state for each (a,t) once
    struct FilteredPair {
        int agent_id;
        int task_id;
        double efficiency;
        bool agent_on_cooldown;
        bool task_on_cooldown;
    };
    std::vector<FilteredPair> filtered_pairs;
    filtered_pairs.reserve(all_pairs.size());

    for (auto& p : all_pairs) {
        int a = p.first;
        int t = p.second;

        // H23+: Skip if source agent is already in middle of a task chain
        if (agent_assigned_task.count(a) > 0) {
            int curr_task = agent_assigned_task[a];
            if (env->task_pool[curr_task].idx_next_loc > 0) {
                continue;  // cannot be reassigned — already in middle of task chain
            }
        }

        // H23+: Do NOT swap if agent is very close to task completion
        // (within 5 cells or 1% of map size — task is almost done)
        const Task& task = env->task_pool[t];
        int next_loc_idx = task.idx_next_loc;
        int task_target = (next_loc_idx < (int)task.locations.size())
                           ? task.locations[next_loc_idx]
                           : task.locations.back();
        int agent_loc = env->curr_states[a].location;
        int dist_to_target = DefaultPlanner::get_h(env, agent_loc, task_target);
        int map_total_cells = env->rows * env->cols;
        int dist_threshold = std::max(10, std::max(5, (int)(map_total_cells * 0.01)));
        if (dist_to_target <= dist_threshold) continue;

        // Agent cooldown check (call-count based, not time-based)
        bool agent_on_cooldown = false;
        auto swap_it = egts_last_swap_call_count.find(a);
        if (swap_it != egts_last_swap_call_count.end() &&
            (schedule_plan_call_count - swap_it->second) < EGTS_AGENT_COOLDOWN) {
            agent_on_cooldown = true;
        }

        // Task cooldown check (call-count based)
        bool task_on_cooldown = false;
        auto task_swap_it = egts_task_last_swap_call_count.find(t);
        if (task_swap_it != egts_task_last_swap_call_count.end() &&
            (schedule_plan_call_count - task_swap_it->second) < EGTS_TASK_COOLDOWN) {
            task_on_cooldown = true;
        }

        // Compute efficiency using REAL simulation time
        double eff = get_agent_efficiency(a, t, env, current_time);

        // Filter: not on cooldown && efficiency below threshold
        if (!agent_on_cooldown && !task_on_cooldown && eff < EGTS_EFFICIENCY_THRESHOLD) {
            filtered_pairs.push_back({a, t, eff, agent_on_cooldown, task_on_cooldown});
        }
    }
    if (filtered_pairs.size() < 2) return;

    // Step 3: O(1) lookup for already-swapped agents
    std::unordered_set<int> swapped_agents;

    // Step 4: double loop to find optimal swap pairs
    for (size_t i = 0; i < filtered_pairs.size(); i++) {
        const FilteredPair& p1 = filtered_pairs[i];
        int a1 = p1.agent_id;
        int t1 = p1.task_id;
        double eff1 = p1.efficiency;

        if (swapped_agents.find(a1) != swapped_agents.end()) continue;

        for (size_t j = i + 1; j < filtered_pairs.size(); j++) {
            const FilteredPair& p2 = filtered_pairs[j];
            int a2 = p2.agent_id;
            int t2 = p2.task_id;
            double eff2 = p2.efficiency;

            if (swapped_agents.find(a2) != swapped_agents.end()) continue;

            // Compute swap gain using actual distances
            int a1_loc = env->curr_states[a1].location;
            int a2_loc = env->curr_states[a2].location;
            int t1_start = env->task_pool[t1].locations[0];
            int t2_start = env->task_pool[t2].locations[0];
            int a1_to_t1 = DefaultPlanner::get_h(env, a1_loc, t1_start);
            int a1_to_t2 = DefaultPlanner::get_h(env, a1_loc, t2_start);
            int a2_to_t1 = DefaultPlanner::get_h(env, a2_loc, t1_start);
            int a2_to_t2 = DefaultPlanner::get_h(env, a2_loc, t2_start);
            int old_total = a1_to_t1 + a2_to_t2;
            int new_total = a1_to_t2 + a2_to_t1;
            double swap_gain = (old_total > 0) ? ((double)(old_total - new_total) / (double)old_total) : 0.0;

            if (swap_gain > EGTS_SWAP_GAIN_THRESHOLD) {
                // H31: track attempts that passed gain threshold
                // egts_swap_attempts++;

                // H31 FIX: Check if target tasks are already opened by OTHER agents
                // If a target task is already started (idx_next_loc > 0) by someone else,
                // the swap would fail validation and cause scheduler errors
                const Task& t1_task = env->task_pool[t1];
                const Task& t2_task = env->task_pool[t2];
                bool t1_already_opened = (t1_task.idx_next_loc > 0 && t1_task.agent_assigned != a1);
                bool t2_already_opened = (t2_task.idx_next_loc > 0 && t2_task.agent_assigned != a2);
                if (t1_already_opened || t2_already_opened) {
                    // Cannot swap - target task is already opened by another agent
                    // egts_swap_rejects_opened++;
                    continue;  // try next j candidate
                }

                egts_swap_count++;
                proposed_schedule[a1] = t2;
                proposed_schedule[a2] = t1;
                agent_assigned_task[a1] = t2;
                agent_assigned_task[a2] = t1;
                egts_last_swap_call_count[a1] = schedule_plan_call_count;
                egts_last_swap_call_count[a2] = schedule_plan_call_count;
                egts_task_last_swap_call_count[t1] = schedule_plan_call_count;
                egts_task_last_swap_call_count[t2] = schedule_plan_call_count;

                swapped_agents.insert(a1);
                swapped_agents.insert(a2);
                break;  // a1 is swapped, move to next i
            }
        }
    }
}

// ================================================================
void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    mt.seed(0);
    free_agents.clear();
    free_tasks.clear();
    task_age_map.clear();
    agent_assigned_task.clear();
    last_reassess_time = 0;
    schedule_reassess_count = 0;
    egts_last_swap_call_count.clear();
    egts_task_last_swap_call_count.clear();
    egts_swap_count = 0;
    egts_call_count = 0;
    // egts_swap_attempts = 0;
    // egts_swap_rejects_opened = 0;
    schedule_plan_call_count = 0;
    return;
}

// ================================================================
void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env)
{
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(time_limit);

    // ✅ FIXED: use REAL simulation timestep, NOT scheduler_call_count * window_size
    int current_time = env->curr_timestep;

    // Sync agent_assigned_task from env
    for (int a = 0; a < env->num_of_agents; a++) {
        int task_id = env->curr_task_schedule[a];
        if (task_id >= 0) {
            if (agent_assigned_task.find(a) == agent_assigned_task.end()) {
                agent_assigned_task[a] = task_id;
            }
        } else {
            auto it = agent_assigned_task.find(a);
            if (it != agent_assigned_task.end())
                agent_assigned_task.erase(it);
        }
    }

    // EGTS re-assessment trigger: every REASSESS_CALL_INTERVAL calls to schedule_plan
    schedule_plan_call_count++;
    schedule_reassess_count++;
    bool do_reassess = (schedule_reassess_count >= REASSESS_CALL_INTERVAL);

    // EGTS - Efficient Guided Task Swap
    if (do_reassess) {
        schedule_reassess_count = 0;
        efficient_task_swap(proposed_schedule, env, current_time);
    }

    // Add new tasks to free_tasks and track their age
    for (int t_id : env->new_tasks)
    {
        if (free_tasks.find(t_id) == free_tasks.end())
        {
            task_age_map[t_id] = current_time;
        }
    }
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    // Baseline greedy assignment: free agents -> nearest free task (by makespan heuristic)
    // IMPORTANT: skip agents already assigned (EGTS may have assigned them)
    int min_task_i, min_task_makespan, dist, c_loc, count;

    std::unordered_set<int>::iterator it = free_agents.begin();
    while (it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime)
        {
            break;
        }
        int i = *it;

        // Skip agents that are already assigned to a task (prevents double-assignment)
        // Also check proposed_schedule in case EGTS assigned them
        if (agent_assigned_task.find(i) != agent_assigned_task.end() || proposed_schedule[i] != -1) {
            it = free_agents.erase(it);
            continue;
        }

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

            // Skip tasks already assigned in proposed_schedule (EGTS may have assigned them)
            bool already_assigned = false;
            for (int a_check = 0; a_check < env->num_of_agents; a_check++) {
                if (proposed_schedule[a_check] == t_id) { already_assigned = true; break; }
            }
            if (already_assigned) { count++; continue; }

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

            // H23: Age-based distance penalty (using REAL simulation timesteps)
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
                    dist += (task_age - TASK_REASSIGN_THRESHOLD) * REASSIGN_AGE_PENALTY_PER_STEP_MAX;
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
            task_age_map.erase(min_task_i);
            // Note: do NOT remove from agent_assigned_task here -
            // the agent now has this task and we want EGTS to track it
        }
        else{
            // No task available for this agent
            // Only set -1 if not already assigned by EGTS
            if (proposed_schedule[i] == -1) {
                // genuinely unassigned - keep -1
            }
            it++;
        }
    }

    // H31: EGTS statistics summary at end of each schedule_plan call
    // static int last_print_call = 0;
    // if (schedule_plan_call_count != last_print_call) {
    //     last_print_call = schedule_plan_call_count;
    //     fprintf(stderr, "[EGTS_STATS] calls=%d swaps=%d attempts=%d rejects_opened=%d\n",
    //             schedule_plan_call_count, egts_swap_count, egts_swap_attempts, egts_swap_rejects_opened);
    // }

    return;
}

} // namespace