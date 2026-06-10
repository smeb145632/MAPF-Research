#include "TaskScheduler.h"

#include "submission_constants.h"
#include "submission_heuristics.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>

bool TaskScheduler::is_task_active(int task_id)
{
    if (task_id < 0) return false;
    const auto task_it = env->task_pool.find(task_id);
    if (task_it == env->task_pool.end()) return false;
    return !task_it->second.is_finished();
}

void TaskScheduler::remove_task_from_caches(int task_id)
{
    if (task_id < 0) return;

    free_tasks.erase(task_id);
    task_age_map.erase(task_id);
    task_start_time.erase(task_id);
    task_location_cache.erase(task_id);
    egts_task_last_swap_time.erase(task_id);

    for (auto it = agent_assigned_task.begin(); it != agent_assigned_task.end(); )
    {
        if (it->second == task_id)
        {
            agent_consecutive_wait[it->first] = 0;
            agent_prev_remaining.erase(it->first);
            agent_task_start_location.erase(it->first);
            agent_waypoint_wait_state.erase(it->first);
            it = agent_assigned_task.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void TaskScheduler::sanitize_proposed_schedule(std::vector<int>& proposed_schedule)
{
    std::vector<int> stale_tasks;
    for (const auto& agent_task : agent_assigned_task)
        if (!is_task_active(agent_task.second))
            stale_tasks.push_back(agent_task.second);
    for (const auto& task_time : task_start_time)
        if (!is_task_active(task_time.first))
            stale_tasks.push_back(task_time.first);
    for (const auto& task_age : task_age_map)
        if (!is_task_active(task_age.first))
            stale_tasks.push_back(task_age.first);
    for (int task_id : free_tasks)
        if (!is_task_active(task_id))
            stale_tasks.push_back(task_id);
    for (int task_id : stale_tasks)
        remove_task_from_caches(task_id);

    std::unordered_set<int> assigned_tasks;
    if (static_cast<int>(proposed_schedule.size()) != env->num_of_agents)
        proposed_schedule.assign(env->num_of_agents, -1);

    for (int agent = 0; agent < env->num_of_agents; agent++)
    {
        const int task_id = proposed_schedule[agent];
        const int current_task_id = agent < static_cast<int>(env->curr_task_schedule.size()) ? env->curr_task_schedule[agent] : -1;

        if (current_task_id >= 0 && is_task_active(current_task_id))
        {
            const Task& current_task = env->task_pool[current_task_id];
            if (current_task.idx_next_loc > 0 && task_id != current_task_id)
            {
                proposed_schedule[agent] = current_task_id;
                assigned_tasks.insert(current_task_id);
                agent_assigned_task[agent] = current_task_id;
                continue;
            }
        }

        if (task_id < 0)
        {
            agent_assigned_task.erase(agent);
            agent_consecutive_wait[agent] = 0;
            agent_prev_remaining.erase(agent);
            agent_task_start_location.erase(agent);
            agent_waypoint_wait_state.erase(agent);
            continue;
        }

        if (!is_task_active(task_id))
        {
            proposed_schedule[agent] = -1;
            remove_task_from_caches(task_id);
            free_agents.insert(agent);
            continue;
        }

        const Task& proposed_task = env->task_pool[task_id];
        if (proposed_task.idx_next_loc > 0 && proposed_task.agent_assigned != agent)
        {
            proposed_schedule[agent] = -1;
            agent_assigned_task.erase(agent);
            agent_consecutive_wait[agent] = 0;
            agent_prev_remaining.erase(agent);
            agent_task_start_location.erase(agent);
            agent_waypoint_wait_state.erase(agent);
            free_agents.insert(agent);
            continue;
        }

        if (assigned_tasks.find(task_id) != assigned_tasks.end())
        {
            proposed_schedule[agent] = -1;
            agent_assigned_task.erase(agent);
            agent_consecutive_wait[agent] = 0;
            agent_prev_remaining.erase(agent);
            agent_task_start_location.erase(agent);
            agent_waypoint_wait_state.erase(agent);
            free_agents.insert(agent);
            continue;
        }

        assigned_tasks.insert(task_id);
    }
}

void TaskScheduler::update_task_location_cache(int task_id)
{
    if (task_location_cache.find(task_id) != task_location_cache.end()) return;
    if (env->task_pool.find(task_id) == env->task_pool.end()) return;

    std::vector<int> locs;
    const auto& task_locs = env->task_pool[task_id].locations;
    for (int loc : task_locs) locs.push_back(loc);
    task_location_cache[task_id] = locs;
}

double TaskScheduler::calculate_overlap_ratio(int task1_id, int task2_id)
{
    const bool cached = task_location_cache.find(task1_id) != task_location_cache.end() &&
                        task_location_cache.find(task2_id) != task_location_cache.end();

    std::vector<int> locs1;
    std::vector<int> locs2;
    if (cached)
    {
        locs1 = task_location_cache[task1_id];
        locs2 = task_location_cache[task2_id];
    }
    else
    {
        if (env->task_pool.find(task1_id) == env->task_pool.end() ||
            env->task_pool.find(task2_id) == env->task_pool.end())
            return 0.0;
        const auto& tl1 = env->task_pool[task1_id].locations;
        const auto& tl2 = env->task_pool[task2_id].locations;
        locs1.assign(tl1.begin(), tl1.end());
        locs2.assign(tl2.begin(), tl2.end());
    }

    std::unordered_set<int> set1(locs1.begin(), locs1.end());
    std::unordered_set<int> set2(locs2.begin(), locs2.end());
    int intersection = 0;
    for (int loc : set1)
        if (set2.find(loc) != set2.end())
            intersection++;

    const int union_count = static_cast<int>(set1.size()) + static_cast<int>(set2.size()) - intersection;
    if (union_count == 0) return 0.0;
    return static_cast<double>(intersection) / static_cast<double>(union_count);
}

std::vector<int> TaskScheduler::compute_segment_costs(const Task& task)
{
    std::vector<int> cumulative;
    cumulative.push_back(0);
    int sum = 0;
    for (size_t k = 1; k < task.locations.size(); k++)
    {
        sum += SubmissionPlanner::get_h(env, task.locations[k - 1], task.locations[k]);
        cumulative.push_back(sum);
    }
    return cumulative;
}

int TaskScheduler::compute_actual_progress_cost(int agent_id, const Task& task)
{
    if (task.locations.empty()) return 0;

    const int idx = task.idx_next_loc;
    const int agent_loc = env->curr_states[agent_id].location;
    const auto cumulative = compute_segment_costs(task);
    const int total = cumulative.back();

    if (idx <= 0)
    {
        const int dist_to_goal = SubmissionPlanner::get_h(env, agent_loc, task.locations[0]);
        return std::max(0, total - dist_to_goal);
    }
    if (idx >= static_cast<int>(task.locations.size()))
        return cumulative.back();

    int completed = 0;
    for (int k = 1; k < idx; k++)
        completed += SubmissionPlanner::get_h(env, task.locations[k - 1], task.locations[k]);

    const int next_loc = task.locations[idx];
    const int prev_loc = task.locations[idx - 1];
    const int segment = SubmissionPlanner::get_h(env, prev_loc, next_loc);
    const int dist_to_next = SubmissionPlanner::get_h(env, agent_loc, next_loc);
    const int traveled = segment - dist_to_next;
    const int actual = completed + traveled;
    return std::max(0, std::min(actual, total));
}

int TaskScheduler::compute_ideal_waypoint_index(const Task& task, int t_revealed, int current_time)
{
    if (current_time <= t_revealed) return 0;

    const int time_elapsed = current_time - t_revealed;
    const auto cumulative = compute_segment_costs(task);
    int lo = 0;
    int hi = static_cast<int>(cumulative.size()) - 1;
    while (lo < hi)
    {
        const int mid = (lo + hi) / 2;
        if (cumulative[mid] < time_elapsed)
            lo = mid + 1;
        else
            hi = mid;
    }
    return lo;
}

void TaskScheduler::update_consecutive_wait_with_next_loc(int agent_id, int task_id, int current_time)
{
    if (task_id < 0)
    {
        agent_waypoint_wait_state.erase(agent_id);
        agent_consecutive_wait[agent_id] = 0;
        return;
    }

    const Task& task = env->task_pool[task_id];
    const int ideal_idx = compute_ideal_waypoint_index(task, task.t_revealed, current_time);
    const int actual_progress = compute_actual_progress_cost(agent_id, task);
    const auto cumulative = compute_segment_costs(task);
    int actual_approx = 0;
    for (int k = 0; k < static_cast<int>(cumulative.size()); k++)
        if (cumulative[k] <= actual_progress)
            actual_approx = k;

    auto it = agent_waypoint_wait_state.find(agent_id);
    if (it == agent_waypoint_wait_state.end())
    {
        agent_waypoint_wait_state[agent_id] = WaypointWaitState{actual_progress, current_time, 0};
        agent_consecutive_wait[agent_id] = 0;
        return;
    }

    WaypointWaitState& state = it->second;
    const bool made_progress = actual_progress > state.last_actual_progress;
    if (made_progress)
        agent_consecutive_wait[agent_id] = 0;
    else if (actual_approx < ideal_idx - 1)
        agent_consecutive_wait[agent_id] += 2;
    else
        agent_consecutive_wait[agent_id]++;

    state.last_actual_progress = actual_progress;
    state.last_check_time = current_time;
}

double TaskScheduler::get_agent_efficiency(int agent_id, int task_id, int current_time)
{
    if (task_id < 0) return 0.0;

    const Task& task = env->task_pool[task_id];
    const auto start_it = task_start_time.find(task_id);
    const int start_time = start_it != task_start_time.end() ? start_it->second : task.t_revealed;
    const int time_elapsed = current_time - start_time;
    if (time_elapsed <= 0) return 0.0;

    const auto cumulative = compute_segment_costs(task);
    const int total = cumulative.back();
    const int ideal_progress = std::min(total, time_elapsed);
    const int actual_progress = compute_actual_progress_cost(agent_id, task);
    double efficiency = ideal_progress > 0 ? static_cast<double>(actual_progress) / static_cast<double>(ideal_progress) : 0.0;
    if (actual_progress < ideal_progress)
    {
        const int behind_cost = ideal_progress - actual_progress;
        efficiency *= std::pow(0.95, behind_cost);
    }
    return efficiency;
}

void TaskScheduler::efficient_task_swap(std::vector<int>& proposed_schedule, int current_time, int window_size)
{
    egts_call_count++;
    if (agent_assigned_task.empty()) return;

    std::vector<std::pair<int, int>> all_pairs;
    for (const auto& agent_task : agent_assigned_task)
        if (agent_task.second >= 0)
            all_pairs.push_back(agent_task);
    if (all_pairs.size() < 2) return;

    std::vector<int> swapped_agents;
    for (size_t i = 0; i < all_pairs.size(); i++)
    {
        const int a1 = all_pairs[i].first;
        const int t1 = all_pairs[i].second;
        if (t1 < 0) continue;
        if (std::find(swapped_agents.begin(), swapped_agents.end(), a1) != swapped_agents.end()) continue;

        if (agent_assigned_task.count(a1) > 0)
        {
            const int curr_task = agent_assigned_task[a1];
            if (env->task_pool[curr_task].idx_next_loc > 0) continue;
        }

        auto swap_it1 = egts_last_swap_time.find(a1);
        if (swap_it1 != egts_last_swap_time.end() &&
            (current_time - swap_it1->second) < EGTS_AGENT_COOLDOWN * window_size)
            continue;

        auto task_swap_it1 = egts_task_last_swap_time.find(t1);
        if (task_swap_it1 != egts_task_last_swap_time.end() &&
            (current_time - task_swap_it1->second) < EGTS_TASK_COOLDOWN * window_size)
            continue;

        const double eff1 = get_agent_efficiency(a1, t1, current_time);
        if (eff1 >= EGTS_EFFICIENCY_THRESHOLD) continue;

        for (size_t j = i + 1; j < all_pairs.size(); j++)
        {
            const int a2 = all_pairs[j].first;
            const int t2 = all_pairs[j].second;
            if (t2 < 0) continue;
            if (std::find(swapped_agents.begin(), swapped_agents.end(), a2) != swapped_agents.end()) continue;

            if (agent_assigned_task.count(a2) > 0)
            {
                const int curr_task = agent_assigned_task[a2];
                if (env->task_pool[curr_task].idx_next_loc > 0) continue;
            }

            auto swap_it2 = egts_last_swap_time.find(a2);
            if (swap_it2 != egts_last_swap_time.end() &&
                (current_time - swap_it2->second) < EGTS_AGENT_COOLDOWN * window_size)
                continue;

            auto task_swap_it2 = egts_task_last_swap_time.find(t2);
            if (task_swap_it2 != egts_task_last_swap_time.end() &&
                (current_time - task_swap_it2->second) < EGTS_TASK_COOLDOWN * window_size)
                continue;

            const double eff2 = get_agent_efficiency(a2, t2, current_time);
            if (eff2 >= EGTS_EFFICIENCY_THRESHOLD) continue;

            const Task& t1_task = env->task_pool[t1];
            if (t1_task.idx_next_loc > 0 && t1_task.agent_assigned == a1) continue;

            const Task& t2_task = env->task_pool[t2];
            if (t2_task.idx_next_loc > 0 && t2_task.agent_assigned == a2) continue;

            const auto start_loc_it1 = agent_task_start_location.find(a1);
            const auto start_loc_it2 = agent_task_start_location.find(a2);
            const int a1_loc = start_loc_it1 != agent_task_start_location.end() ? start_loc_it1->second : env->curr_states[a1].location;
            const int a2_loc = start_loc_it2 != agent_task_start_location.end() ? start_loc_it2->second : env->curr_states[a2].location;
            const int t1_start = env->task_pool[t1].locations[0];
            const int t2_start = env->task_pool[t2].locations[0];
            const int a1_to_t1 = SubmissionPlanner::get_h(env, a1_loc, t1_start);
            const int a1_to_t2 = SubmissionPlanner::get_h(env, a1_loc, t2_start);
            const int a2_to_t1 = SubmissionPlanner::get_h(env, a2_loc, t1_start);
            const int a2_to_t2 = SubmissionPlanner::get_h(env, a2_loc, t2_start);
            const int old_total = a1_to_t1 + a2_to_t2;
            const int new_total = a1_to_t2 + a2_to_t1;
            const double swap_gain = old_total > 0 ? static_cast<double>(old_total - new_total) / static_cast<double>(old_total) : 0.0;

            if (swap_gain > EGTS_SWAP_GAIN_THRESHOLD)
            {
                egts_swap_count++;
                proposed_schedule[a1] = t2;
                proposed_schedule[a2] = t1;
                agent_assigned_task[a1] = t2;
                agent_assigned_task[a2] = t1;
                task_start_time[t2] = current_time;
                task_start_time[t1] = current_time;
                agent_task_start_location[a1] = env->curr_states[a1].location;
                agent_task_start_location[a2] = env->curr_states[a2].location;
                egts_last_swap_time[a1] = current_time;
                egts_last_swap_time[a2] = current_time;
                egts_task_last_swap_time[t1] = current_time;
                egts_task_last_swap_time[t2] = current_time;
                agent_consecutive_wait[a1] = 0;
                agent_consecutive_wait[a2] = 0;

                swapped_agents.push_back(a1);
                swapped_agents.push_back(a2);
                break;
            }
        }
    }
}

void TaskScheduler::initialize(int preprocess_time_limit)
{
    mt.seed(0);
    free_agents.clear();
    free_tasks.clear();
    task_age_map.clear();
    agent_assigned_task.clear();
    task_start_time.clear();
    agent_task_start_location.clear();
    agent_last_switch_time.clear();
    agent_consecutive_wait.clear();
    agent_prev_remaining.clear();
    task_location_cache.clear();
    agent_waypoint_wait_state.clear();
    egts_last_swap_time.clear();
    egts_task_last_swap_time.clear();
    reassign_age_penalty_multiplier = 1.0;
    last_reassess_time = 0;
    switch_waiting_triggered = 0;
    egts_swap_count = 0;
    egts_call_count = 0;
    scheduler_call_count = 0;

    SubmissionPlanner::init_heuristics(env);

    if (env->num_of_agents > 0 && !env->new_tasks.empty())
    {
        const int sample_size = std::min(20, static_cast<int>(env->new_tasks.size()));
        long long total_dist = 0;
        int sample_count = 0;
        std::vector<int> task_ids(env->new_tasks.begin(), env->new_tasks.end());
        std::shuffle(task_ids.begin(), task_ids.end(), mt);
        for (int s = 0; s < sample_size; s++)
        {
            const int task_id = task_ids[s];
            const int task_loc = env->task_pool[task_id].locations[0];
            for (int agent = 0; agent < std::min(5, env->num_of_agents); agent++)
            {
                const int agent_loc = env->curr_states[agent].location;
                total_dist += SubmissionPlanner::get_h(env, agent_loc, task_loc);
                sample_count++;
            }
        }

        const double avg_dist = sample_count > 0 ? static_cast<double>(total_dist) / static_cast<double>(sample_count) : 0.0;
        reassign_age_penalty_multiplier = avg_dist < 40.0 ? 0.4 : 1.0;
    }
}

void TaskScheduler::plan(int time_limit, std::vector<int>& proposed_schedule)
{
    proposed_schedule = env->curr_task_schedule;
    if (static_cast<int>(proposed_schedule.size()) != env->num_of_agents)
        proposed_schedule.assign(env->num_of_agents, -1);

    const int scheduler_time_limit = std::max(1, time_limit / 2 - SubmissionPlanner::SCHEDULER_TIMELIMIT_TOLERANCE);
    const TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(scheduler_time_limit);

    scheduler_call_count++;
    const int action_window = env->action_time * env->max_counter;
    const int window_size = action_window > 0 ? static_cast<int>(env->min_planner_communication_time / action_window) + 1 : 1;
    const int current_time = scheduler_call_count * window_size;

    sanitize_proposed_schedule(proposed_schedule);

    for (int agent = 0; agent < env->num_of_agents; agent++)
    {
        const int task_id = proposed_schedule[agent];
        if (task_id >= 0)
        {
            const bool is_new_assignment = agent_assigned_task.find(agent) == agent_assigned_task.end() ||
                                           agent_assigned_task[agent] != task_id;
            if (is_new_assignment)
            {
                agent_assigned_task[agent] = task_id;
                if (task_start_time.find(task_id) == task_start_time.end())
                    task_start_time[task_id] = current_time;
                const int goal_loc = env->task_pool[task_id].locations.back();
                const int init_remaining = SubmissionPlanner::get_h(env, env->curr_states[agent].location, goal_loc);
                agent_prev_remaining[agent] = init_remaining;
                agent_task_start_location[agent] = env->curr_states[agent].location;
                agent_consecutive_wait[agent] = 0;
            }
        }
        else
        {
            agent_assigned_task.erase(agent);
            agent_task_start_location.erase(agent);
            agent_consecutive_wait[agent] = 0;
        }
    }

    const bool do_reassess = current_time - last_reassess_time >= REASSESS_INTERVAL * window_size;
    if (do_reassess && !agent_assigned_task.empty())
    {
        last_reassess_time = current_time;
        for (const auto& agent_task : agent_assigned_task)
        {
            const int task_id = agent_task.second;
            auto start_it = task_start_time.find(task_id);
            if (start_it == task_start_time.end()) continue;

            const int time_elapsed = current_time - start_it->second;
            if (time_elapsed <= 20 * window_size) continue;

            const int agent_id = agent_task.first;
            const int goal_loc = env->task_pool[task_id].locations.back();
            const int agent_loc = env->curr_states[agent_id].location;
            const int remaining = SubmissionPlanner::get_h(env, agent_loc, goal_loc);
            const int initial_dist = SubmissionPlanner::get_h(env, env->task_pool[task_id].locations[0], goal_loc);
            if (initial_dist > 0 && remaining > initial_dist * 8 / 10)
                task_age_map[task_id] = current_time - TASK_FORCE_REASSIGN_THRESHOLD * window_size - 10 * window_size;
        }
    }

    for (int agent = 0; agent < env->num_of_agents; agent++)
        update_consecutive_wait_with_next_loc(agent, proposed_schedule[agent], current_time);

    if (do_reassess)
        efficient_task_swap(proposed_schedule, current_time, window_size);

    if (do_reassess && !agent_assigned_task.empty())
    {
        for (const auto& agent_task : agent_assigned_task)
        {
            const int agent = agent_task.first;
            auto switch_it = agent_last_switch_time.find(agent);
            if (switch_it != agent_last_switch_time.end() &&
                (current_time - switch_it->second) < TASK_SWITCH_COOLDOWN_H31 * window_size)
                continue;

            const int curr_task_id = agent_task.second;
            if (curr_task_id < 0) continue;

            const int wait_count = agent_consecutive_wait.count(agent) > 0 ? agent_consecutive_wait[agent] : 0;
            if (wait_count < WAIT_THRESHOLD) continue;

            const int agent_loc = env->curr_states[agent].location;
            int best_free_task = -1;
            double best_score = -1.0;
            for (int free_task_id : free_tasks)
            {
                const int task_loc = env->task_pool[free_task_id].locations[0];
                const int travel_dist = SubmissionPlanner::get_h(env, agent_loc, task_loc);
                const int task_goal = env->task_pool[free_task_id].locations.back();
                const int task_dist = SubmissionPlanner::get_h(env, task_loc, task_goal);
                const int new_total_dist = travel_dist + task_dist;
                double score = new_total_dist > 0 ? 1000.0 / static_cast<double>(new_total_dist) : 1000.0;
                auto age_it = task_age_map.find(free_task_id);
                if (age_it != task_age_map.end() && current_time - age_it->second > 50)
                    score *= 0.8;
                if (score > best_score)
                {
                    best_score = score;
                    best_free_task = free_task_id;
                }
            }

            if (env->task_pool[curr_task_id].idx_next_loc > 0) continue;

            if (best_free_task != -1)
            {
                proposed_schedule[agent] = best_free_task;
                task_age_map[curr_task_id] = current_time - TASK_REASSIGN_THRESHOLD * window_size - 10 * window_size;
                free_tasks.erase(best_free_task);
                agent_assigned_task[agent] = best_free_task;
                task_start_time[best_free_task] = current_time;
                agent_task_start_location[agent] = agent_loc;
                const int goal_loc = env->task_pool[best_free_task].locations.back();
                const int init_remaining = SubmissionPlanner::get_h(env, agent_loc, goal_loc);
                agent_prev_remaining[agent] = init_remaining;
                agent_consecutive_wait[agent] = 0;
                task_start_time.erase(curr_task_id);
                agent_last_switch_time[agent] = current_time;
                switch_waiting_triggered++;
            }
        }
    }

    if (do_reassess && !free_agents.empty() && !agent_assigned_task.empty())
    {
        std::vector<int> free_agent_snapshot(free_agents.begin(), free_agents.end());
        for (int agent : free_agent_snapshot)
        {
            if (free_agents.find(agent) == free_agents.end()) continue;
            auto switch_it = agent_last_switch_time.find(agent);
            if (switch_it != agent_last_switch_time.end() &&
                (current_time - switch_it->second) < TASK_SWITCH_COOLDOWN * window_size)
                continue;

            const int curr_task_id = env->curr_task_schedule[agent];
            if (curr_task_id < 0) continue;
            if (env->task_pool[curr_task_id].idx_next_loc > 0) continue;

            const int agent_loc = env->curr_states[agent].location;
            const int goal_loc = env->task_pool[curr_task_id].locations.back();
            const int remaining = SubmissionPlanner::get_h(env, agent_loc, goal_loc);
            auto start_it = task_start_time.find(curr_task_id);
            const int time_elapsed = start_it != task_start_time.end() ? current_time - start_it->second : 1;
            const int progress = SubmissionPlanner::get_h(env, env->task_pool[curr_task_id].locations[0], goal_loc) - remaining;
            const double curr_efficiency = time_elapsed > 0 ? static_cast<double>(progress) / static_cast<double>(time_elapsed) : 0.0;
            if (curr_efficiency >= 0.3) continue;

            int best_free_task = -1;
            double best_efficiency = curr_efficiency * EFFICIENCY_SWITCH_THRESHOLD;
            for (int free_task_id : free_tasks)
            {
                const int task_loc = env->task_pool[free_task_id].locations[0];
                const int travel_dist = SubmissionPlanner::get_h(env, agent_loc, task_loc);
                const int task_goal = env->task_pool[free_task_id].locations.back();
                const int task_dist = SubmissionPlanner::get_h(env, task_loc, task_goal);
                const int new_total_dist = travel_dist + task_dist;
                const double free_efficiency = new_total_dist > 0 ? 1.0 / static_cast<double>(new_total_dist) : 1.0;
                if (free_efficiency > best_efficiency)
                {
                    best_efficiency = free_efficiency;
                    best_free_task = free_task_id;
                }
            }

            if (best_free_task != -1 && best_efficiency > curr_efficiency * EFFICIENCY_SWITCH_THRESHOLD)
            {
                proposed_schedule[agent] = best_free_task;
                task_age_map[curr_task_id] = current_time - TASK_REASSIGN_THRESHOLD * window_size - 10 * window_size;
                free_tasks.erase(best_free_task);
                agent_assigned_task[agent] = best_free_task;
                task_start_time[best_free_task] = current_time;
                agent_task_start_location[agent] = agent_loc;
                task_start_time.erase(curr_task_id);
                agent_last_switch_time[agent] = current_time;
                free_agents.erase(agent);
            }
        }
    }

    if (do_reassess && !agent_assigned_task.empty() && !free_tasks.empty())
    {
        for (const auto& agent_task : agent_assigned_task)
            update_task_location_cache(agent_task.second);

        std::vector<int> agents_with_tasks;
        for (const auto& agent_task : agent_assigned_task)
            agents_with_tasks.push_back(agent_task.first);

        for (size_t i = 0; i < agents_with_tasks.size(); i++)
        {
            const int a1 = agents_with_tasks[i];
            const int task1 = agent_assigned_task[a1];
            auto switch_it1 = agent_last_switch_time.find(a1);
            if (switch_it1 != agent_last_switch_time.end() &&
                (current_time - switch_it1->second) < MUTUAL_SWITCH_COOLDOWN * window_size)
                continue;

            for (size_t j = i + 1; j < agents_with_tasks.size(); j++)
            {
                const int a2 = agents_with_tasks[j];
                const int task2 = agent_assigned_task[a2];
                const double overlap = calculate_overlap_ratio(task1, task2);
                if (overlap < MUTUAL_INHIBITION_OVERLAP_THRESHOLD) continue;

                const double eff1 = get_agent_efficiency(a1, task1, current_time);
                const double eff2 = get_agent_efficiency(a2, task2, current_time);
                if (eff1 > MUTUAL_INHIBITION_EFFICIENCY_THRESHOLD &&
                    eff2 > MUTUAL_INHIBITION_EFFICIENCY_THRESHOLD)
                    continue;

                const int switch_agent = eff1 < eff2 ? a1 : a2;
                const int switch_task = eff1 < eff2 ? task1 : task2;
                const int stay_agent = eff1 < eff2 ? a2 : a1;
                if (env->task_pool[switch_task].idx_next_loc > 0) continue;

                const int agent_loc = env->curr_states[switch_agent].location;
                int best_free_task = -1;
                double best_score = -1.0;
                for (int free_task_id : free_tasks)
                {
                    if (free_task_id == switch_task) continue;
                    update_task_location_cache(free_task_id);
                    const double overlap_with_stay = calculate_overlap_ratio(free_task_id, agent_assigned_task[stay_agent]);
                    if (overlap_with_stay > MUTUAL_INHIBITION_OVERLAP_THRESHOLD) continue;

                    const int task_loc = env->task_pool[free_task_id].locations[0];
                    const int travel_dist = SubmissionPlanner::get_h(env, agent_loc, task_loc);
                    const int task_goal = env->task_pool[free_task_id].locations.back();
                    const int task_dist = SubmissionPlanner::get_h(env, task_loc, task_goal);
                    const int new_total_dist = travel_dist + task_dist;
                    const double score = new_total_dist > 0 ? 1000.0 / static_cast<double>(new_total_dist) : 1000.0;
                    if (score > best_score)
                    {
                        best_score = score;
                        best_free_task = free_task_id;
                    }
                }

                if (best_free_task != -1)
                {
                    proposed_schedule[switch_agent] = best_free_task;
                    task_age_map[switch_task] = current_time - TASK_REASSIGN_THRESHOLD * window_size - 10 * window_size;
                    free_tasks.erase(best_free_task);
                    agent_assigned_task[switch_agent] = best_free_task;
                    task_start_time[best_free_task] = current_time;
                    agent_task_start_location[switch_agent] = agent_loc;
                    task_start_time.erase(switch_task);
                    agent_last_switch_time[switch_agent] = current_time;
                    task_location_cache.erase(switch_task);
                    break;
                }
            }
        }
    }

    for (int task_id : env->new_tasks)
        if (free_tasks.find(task_id) == free_tasks.end())
            task_age_map[task_id] = current_time;
    free_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    free_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());

    auto free_agent_it = free_agents.begin();
    while (free_agent_it != free_agents.end())
    {
        if (std::chrono::steady_clock::now() > endtime) break;

        const int agent = *free_agent_it;
        if (env->curr_task_schedule[agent] != -1)
        {
            free_agent_it = free_agents.erase(free_agent_it);
            continue;
        }

        int best_task = -1;
        int best_cost = INT_MAX;
        int count = 0;
        for (int task_id : free_tasks)
        {
            if (count % 10 == 0 && std::chrono::steady_clock::now() > endtime) break;

            const int agent_loc = env->curr_states.at(agent).location;
            const int travel_dist = SubmissionPlanner::get_h(env, agent_loc, env->task_pool[task_id].locations[0]);
            int task_internal = 0;
            int prev_loc = env->task_pool[task_id].locations[0];
            for (size_t k = 1; k < env->task_pool[task_id].locations.size(); k++)
            {
                task_internal += SubmissionPlanner::get_h(env, prev_loc, env->task_pool[task_id].locations[k]);
                prev_loc = env->task_pool[task_id].locations[k];
            }

            int cost = travel_dist + task_internal / 2;
            auto age_it = task_age_map.find(task_id);
            if (age_it != task_age_map.end())
            {
                const int task_age = current_time - age_it->second;
                if (task_age > TASK_FORCE_REASSIGN_THRESHOLD * window_size && TASK_FORCE_REASSIGN_THRESHOLD > 0)
                    cost += 10000;
                else if (task_age > TASK_REASSIGN_THRESHOLD * window_size)
                    cost += static_cast<int>((task_age - TASK_REASSIGN_THRESHOLD) *
                                             REASSIGN_AGE_PENALTY_PER_STEP_MAX *
                                             reassign_age_penalty_multiplier);
            }

            if (cost < best_cost)
            {
                best_task = task_id;
                best_cost = cost;
            }
            count++;
        }

        if (best_task != -1)
        {
            proposed_schedule[agent] = best_task;
            agent_assigned_task[agent] = best_task;
            task_start_time[best_task] = current_time;
            agent_task_start_location[agent] = env->curr_states.at(agent).location;
            free_agent_it = free_agents.erase(free_agent_it);
            free_tasks.erase(best_task);
            task_age_map.erase(best_task);
        }
        else
        {
            proposed_schedule[agent] = -1;
            free_agent_it++;
        }
    }

    sanitize_proposed_schedule(proposed_schedule);
}
