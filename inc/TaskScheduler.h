#pragma once
#include "Tasks.h"
#include "SharedEnv.h"

#include <random>
#include <unordered_map>
#include <unordered_set>


class TaskScheduler
{
    public:
        SharedEnvironment* env;

        TaskScheduler(SharedEnvironment* env): env(env){};
        TaskScheduler(){env = new SharedEnvironment();};
        virtual ~TaskScheduler(){delete env;};
        virtual void initialize(int preprocess_time_limit);
        virtual void plan(int time_limit, std::vector<int> & proposed_schedule);

    private:
        struct WaypointWaitState
        {
            int last_actual_progress = 0;
            int last_check_time = 0;
            int consecutive_no_progress = 0;
        };

        static constexpr int TASK_REASSIGN_THRESHOLD = 100;
        static constexpr int REASSIGN_AGE_PENALTY_PER_STEP_MAX = 5;
        static constexpr int TASK_FORCE_REASSIGN_THRESHOLD = 200;
        static constexpr int REASSESS_INTERVAL = 5;
        static constexpr double EFFICIENCY_SWITCH_THRESHOLD = 1.5;
        static constexpr int TASK_SWITCH_COOLDOWN = 100;
        static constexpr int WAIT_THRESHOLD = 5;
        static constexpr int TASK_SWITCH_COOLDOWN_H31 = 80;
        static constexpr double MUTUAL_INHIBITION_OVERLAP_THRESHOLD = 0.70;
        static constexpr double MUTUAL_INHIBITION_EFFICIENCY_THRESHOLD = 0.3;
        static constexpr int MUTUAL_SWITCH_COOLDOWN = 100;
        static constexpr double EGTS_EFFICIENCY_THRESHOLD = 0.3;
        static constexpr double EGTS_SWAP_GAIN_THRESHOLD = 0.2;
        static constexpr int EGTS_AGENT_COOLDOWN = 15;
        static constexpr int EGTS_TASK_COOLDOWN = 20;

        std::mt19937 mt;
        std::unordered_set<int> free_agents;
        std::unordered_set<int> free_tasks;
        std::unordered_map<int, int> task_age_map;
        std::unordered_map<int, int> agent_assigned_task;
        std::unordered_map<int, int> task_start_time;
        std::unordered_map<int, int> agent_last_switch_time;
        std::unordered_map<int, int> agent_consecutive_wait;
        std::unordered_map<int, int> agent_prev_remaining;
        std::unordered_map<int, std::vector<int>> task_location_cache;
        std::unordered_map<int, int> egts_last_swap_time;
        std::unordered_map<int, int> egts_task_last_swap_time;
        std::unordered_map<int, WaypointWaitState> agent_waypoint_wait_state;

        double reassign_age_penalty_multiplier = 1.0;
        int last_reassess_time = 0;
        int switch_waiting_triggered = 0;
        int egts_swap_count = 0;
        int egts_call_count = 0;
        int scheduler_call_count = 0;

        bool is_task_active(int task_id);
        void remove_task_from_caches(int task_id);
        void sanitize_proposed_schedule(std::vector<int>& proposed_schedule);
        void update_task_location_cache(int task_id);
        double calculate_overlap_ratio(int task1_id, int task2_id);
        std::vector<int> compute_segment_costs(const Task& task);
        int compute_actual_progress_cost(int agent_id, const Task& task);
        int compute_ideal_waypoint_index(const Task& task, int t_revealed, int current_time);
        void update_consecutive_wait_with_next_loc(int agent_id, int task_id, int current_time);
        double get_agent_efficiency(int agent_id, int task_id, int current_time);
        void efficient_task_swap(std::vector<int>& proposed_schedule, int current_time, int window_size);
};
