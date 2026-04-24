#ifndef CONST_H
#define CONST_H
namespace DefaultPlanner
{
    // pibt runtime (ms) per 100 agents. 
    // The default planner will use this value to determine how much time to allocate for PIBT action time.
    // The default planner compute the end time for traffic flow assignment by subtracting PIBT action time from the time limit.
    const int PIBT_RUNTIME_PER_100_AGENTS = 1;

    // Traffic flow assignment end time tolerance in ms.
    // The default planner will end the traffic flow assignment phase this many milliseconds before traffic flow assignment end time.
    const int TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE = 10;


    // The default planner timelimit tolerance in ms.
    // The MAPFPlanner will deduct this value from the time limit for default planner.
    const int PLANNER_TIMELIMIT_TOLERANCE = 10;

    // The default scheduler timelimit tolerance in ms.
    // The TaskScheduler will deduct this value from the time limit for default scheduler.
    const int SCHEDULER_TIMELIMIT_TOLERANCE = 10;

    // Frank-Wolfe optimization: number of top-deviation agents to optimize per iteration
    // Set to 0 or negative to disable top-K filtering (use all deviation agents)
    const int FW_TOP_K_AGENTS = 10;

    // ============================================================
    // Adaptive PIBT parameters (H03)
    // Based on scene density and time budget to dynamically adjust PIBT time allocation
    // ============================================================
    
    // Density-based adjustment: when density is high, more conflicts occur,
    // so we allocate less time per agent (more frequent but shorter planning)
    const double PIBT_DENSITY_HIGH_THRESHOLD = 0.7;  // High density threshold
    const double PIBT_DENSITY_LOW_THRESHOLD = 0.3;   // Low density threshold
    const double PIBT_DENSITY_FACTOR = 0.5;          // Time adjustment factor based on density
    
    // Time budget-based adjustment: when time is tight, reduce PIBT time to ensure flow optimization
    const double PIBT_TIME_BUDGET_THRESHOLD = 0.5;  // Ratio of time limit to consider "tight"
    const double PIBT_TIME_BUDGET_FACTOR = 0.7;     // Multiplier when time is tight

    // ============================================================
    // Dynamic staged actions cache configuration
    // When task density is high (busy_agents/total_agents > high_density_threshold),
    // reduce planning steps to save memory. When low, increase for better efficiency.
    // ============================================================
    const bool ENABLE_DYNAMIC_STAGED_CACHE = true;
    const double HIGH_DENSITY_THRESHOLD = 0.7;  // If >70% agents have tasks, reduce cache
    const double LOW_DENSITY_THRESHOLD = 0.3;   // If <30% agents have tasks, increase cache
    const int MIN_DYNAMIC_STEPS = 10;           // Minimum planning steps
    const int MAX_DYNAMIC_STEPS = 50;           // Maximum planning steps
    const double DENSITY_FACTOR = 30;           // Steps adjustment factor based on density

}
#endif