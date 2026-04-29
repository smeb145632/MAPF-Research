#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "SharedEnv.h"
#include <unordered_set>
#include <vector>
#include <chrono>

namespace DefaultPlanner{

// ============================================================
// H23: Task Reassignment Threshold Optimization
// When a task has been in free_tasks for too long (due to poor
// initial assignment from heuristic error), reassign it to a
// different agent rather than waiting forever.
// ============================================================

// Maximum age (in timesteps) a task can wait in free_tasks before being marked for reassignment
// Tasks older than this get a distance penalty to encourage reassignment
const int TASK_REASSIGN_THRESHOLD = 100;

// How much to penalize old tasks per timestep of age
// Task effective distance += task_age * REASSIGN_AGE_PENALTY_PER_STEP
const int REASSIGN_AGE_PENALTY_PER_STEP = 5;

// Force-reassign tasks older than this (completely bypass distance check)
// Set to 0 to disable force-reassignment (use penalty only)
const int TASK_FORCE_REASSIGN_THRESHOLD = 200;

// ============================================================

void schedule_initialize(int preprocess_time_limit, SharedEnvironment* env);
void schedule_plan(int time_limit, std::vector<int> & proposed_schedule,  SharedEnvironment* env);

// Internal state (persistent across timesteps)
extern std::unordered_set<int> free_agents;
extern std::unordered_set<int> free_tasks;

// Task age tracking: task_id -> timestep when task first entered free_tasks
// (persists across timesteps until task is assigned)
extern std::unordered_map<int, int> task_age_map;

}

#endif