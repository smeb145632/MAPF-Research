#pragma once
#include <ctime>
#include "SharedEnv.h"
#include "ActionModel.h"
#include "Plan.h"
#include "MAPFPlanner.h"
#include "TaskScheduler.h"


class Entry
{
public:
    SharedEnvironment* env;
    MAPFPlanner* mapfPlanner;  // 路径规划器（负责计算 agent 移动路径）
    TaskScheduler* taskScheduler;  // 任务调度器（负责分配任务给 agents）

	Entry(SharedEnvironment* env): env(env)
    {
        mapfPlanner = new MAPFPlanner(env);
    };
    Entry()
    {
        env = new SharedEnvironment();
        mapfPlanner = new MAPFPlanner(env);
        taskScheduler = new TaskScheduler(env);

    };
	virtual ~Entry(){delete env;};


    virtual void initialize(int preprocess_time_limit);

    // return next actions and the proposed task schedule for all agents
    virtual void compute(int time_limit, Plan & plan, std::vector<int> & proposed_schedule);

    void update_goal_locations(std::vector<int> & proposed_schedule);
};