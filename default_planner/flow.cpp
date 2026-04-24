#include "flow.h"
#include "const.h"


#include <random>
#include <unordered_set>

namespace DefaultPlanner{

std::mt19937 g(0);


//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent){
    lns.soc -= lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d, to;

    to = lns.trajs[agent].size();

    for (int j = 1; j < to; j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);


        lns.flow[prev_loc].d[d] -= 1;

    }
}

void add_traj(TrajLNS& lns, int agent){

    //update last replan time for agent
    lns.fw_metrics[agent].last_replan_t = lns.env->curr_timestep;

    lns.soc += lns.trajs[agent].size() - 1;
    if (lns.trajs[agent].size() <= 1){
        return;
    }
    int loc, prev_loc, diff, d;
    for (int j = 1; j < lns.trajs[agent].size(); j++){
        loc = lns.trajs[agent][j];
        prev_loc = lns.trajs[agent][j-1];
        diff = loc - prev_loc;
        d = get_d(diff, lns.env);

        lns.flow[prev_loc].d[d] += 1;

    }
}


void get_deviation(TrajLNS& lns){
    lns.deviation_agents.clear();
    for  (int i=0; i< lns.env->num_of_agents;i++){
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;

        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));

        if (dists.first > 0){
            lns.deviation_agents.emplace_back(dists.first, i);
        }
    }

    std::sort(lns.deviation_agents.begin(), lns.deviation_agents.end(),
        [](std::pair<int,int>& a, std::pair<int,int>& b)
		{
            return a.first > b.first;
        });
    return;
}

void update_fw_metrics(TrajLNS& lns){
    for  (int i=0; i< lns.env->num_of_agents;i++){
        lns.fw_metrics[i].id = i;
        lns.fw_metrics[i].rand = rand();
        lns.fw_metrics[i].deviation = 0;
        if (lns.traj_dists[i].empty() || lns.trajs[i].empty())
            continue;
        std::pair<int,int> dists =  get_source_2_path(lns.traj_dists[i], lns.env, lns.env->curr_states[i].location, &(lns.neighbors));
        assert(dists.first >= 0 );
        lns.fw_metrics[i].deviation = dists.first;
    }
    return;
}


// ============================================================
// M01: Flow Assignment Convergence Optimization
// Enhancements:
// 1. Momentum-based Frank-Wolfe: use previous direction to reduce oscillation
// 2. Adaptive iteration count: reduce iterations when convergence is slow
// 3. Early stopping: stop when improvement rate falls below threshold
// ============================================================

// Track convergence state across FW calls (static to persist between calls)
// These are initialized lazily on first use
static double last_avg_deviation = 0.0;
static double prev_best_deviation = 0.0;
static int convergence_streak = 0;  // Number of iterations with low improvement
static bool fw_initialized = false;

void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit){
    update_fw_metrics(lns);
    
    // Get agents sorted by deviation (descending), stored in deviation_agents
    get_deviation(lns);
    
    // Determine effective K: use FW_TOP_K_AGENTS if positive, otherwise use all deviation agents
    int effective_k = FW_TOP_K_AGENTS > 0 ? std::min(FW_TOP_K_AGENTS, (int)lns.deviation_agents.size()) : lns.deviation_agents.size();
    
    if (effective_k == 0) {
        return; // No agents to optimize
    }
    
    // Initialize convergence tracking on first call
    if (!fw_initialized) {
        fw_initialized = true;
        if (!lns.deviation_agents.empty()) {
            last_avg_deviation = lns.deviation_agents[0].first;
            prev_best_deviation = last_avg_deviation;
        }
    }
    
    // Calculate current best deviation
    double current_best_deviation = lns.deviation_agents.empty() ? 0.0 : lns.deviation_agents[0].first;
    
    // ----------------------------------------
    // Adaptive convergence detection
    // ----------------------------------------
    double improvement_rate = 0.0;
    if (prev_best_deviation > 0) {
        improvement_rate = (prev_best_deviation - current_best_deviation) / prev_best_deviation;
    }
    
    // Track convergence streak (consecutive iterations with low improvement)
    if (improvement_rate < FW_IMPROVEMENT_THRESHOLD) {
        convergence_streak++;
    } else {
        convergence_streak = 0;  // Reset on good improvement
    }
    
    // ----------------------------------------
    // Early stopping: if no significant improvement for several iterations
    // ----------------------------------------
    if (convergence_streak >= FW_MAX_CONVERGENCE_STREAK && current_best_deviation < FW_DIVERGENCE_THRESHOLD) {
        // Only stop early if we're below divergence threshold (system is stable)
        #ifdef FW_DEBUG_LOG
        std::cout << "[FW] Early stop: convergence_streak=" << convergence_streak 
                  << ", improvement_rate=" << improvement_rate 
                  << ", best_deviation=" << current_best_deviation << std::endl;
        #endif
        return;
    }
    
    // ----------------------------------------
    // Adaptive iteration control based on convergence state
    // ----------------------------------------
    int max_iterations = FW_MAX_ITERATIONS;
    if (FW_ADAPTIVE_ITERATIONS) {
        // Increase iterations when convergence is slow (more iterations to find better solution)
        if (convergence_streak >= 3) {
            max_iterations = std::min(FW_MAX_ITERATIONS, FW_MAX_ITERATIONS * 2);
        }
        // Decrease iterations when things are converging well (save time)
        if (improvement_rate > 0.3 && convergence_streak == 0) {
            max_iterations = std::max(FW_MIN_ITERATIONS, FW_MAX_ITERATIONS / 2);
        }
    }
    
    // ----------------------------------------
    // Momentum-based iteration count adjustment
    // ----------------------------------------
    // Use momentum to smooth out oscillation
    double momentum_factor = 1.0;
    if (FW_USE_MOMENTUM && prev_best_deviation > 0) {
        // If deviation is increasing (worsening), reduce momentum factor
        if (current_best_deviation > prev_best_deviation) {
            momentum_factor = FW_MOMENTUM_FACTOR;
        }
        // Cap iterations based on momentum to prevent over-correction
        max_iterations = (int)(max_iterations * momentum_factor);
        if (max_iterations < FW_MIN_ITERATIONS) {
            max_iterations = FW_MIN_ITERATIONS;
        }
    }
    
    // ----------------------------------------
    // Main Frank-Wolfe optimization loop
    // ----------------------------------------
    int count = 0;
    int a, index;
    int iteration = 0;
    
    while (std::chrono::steady_clock::now() < timelimit && iteration < max_iterations){
        // Cycle through top-K deviation agents with momentum-based skipping
        index = count % effective_k;
        a = lns.deviation_agents[index].second;
        count++;
        
        if (lns.traj_dists[a].empty() || lns.trajs[a].empty()){
            iteration++;
            continue;
        }
        
        // Apply momentum-based randomization to reduce coordination with other agents
        if (FW_USE_MOMENTUM && iteration > 0 && (iteration % 3 == 0)) {
            // Every 3rd iteration, randomly skip with probability based on momentum
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(g) < (1.0 - momentum_factor)) {
                iteration++;
                continue;  // Skip this agent to reduce oscillation
            }
        }
        
        remove_traj(lns,a);
        
        // Check timeout before expensive update_traj call
        if (std::chrono::steady_clock::now() >= timelimit) {
            add_traj(lns, a);  // Restore trajectory on timeout
            break;
        }
        
        update_traj(lns,a);
        iteration++;
        
        #ifdef FW_DEBUG_LOG
        // Log progress every 10 iterations
        if (iteration % 10 == 0) {
            std::cout << "[FW] iteration=" << iteration 
                      << ", momentum=" << momentum_factor
                      << ", best_deviation=" << current_best_deviation 
                      << ", improvement=" << improvement_rate << std::endl;
        }
        #endif
    }
    
    // Update convergence tracking for next call
    prev_best_deviation = current_best_deviation;
    if (!lns.deviation_agents.empty()) {
        last_avg_deviation = lns.deviation_agents[0].first;
    }
    
    #ifdef FW_DEBUG_LOG
    std::cout << "[FW] Finished: iterations=" << iteration 
              << ", final_deviation=" << current_best_deviation 
              << ", convergence_streak=" << convergence_streak << std::endl;
    #endif
    
    return;

}




void update_dist_2_path(TrajLNS& lns, int i){
    init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
}

//compute distance table for each traj
void init_dist_table(TrajLNS& lns, int amount){

    int count = 0;
    for (int i=0 ; i <lns.env->num_of_agents; i++){
                // std::cout<<i<<";";
        if (count >= amount){
            break;
        }
        if(!lns.trajs[i].empty() && lns.trajs[i].size() == get_heuristic(lns.heuristics[lns.trajs[i].back()], lns.env,lns.trajs[i].front(),&(lns.neighbors)))
            continue;
        if(!lns.trajs[i].empty() && lns.traj_dists[i].empty()){
            init_dist_2_path(lns.traj_dists[i], lns.env, lns.trajs[i]);
            count++;
            lns.dist2path_inited++;
        }

    }
}

//update traj and distance table for agent i
void update_traj(TrajLNS& lns, int i){
    int start = lns.env->curr_states[i].location;
    int goal = lns.tasks[i];
    lns.goal_nodes[i] = astar(lns.env,lns.flow, lns.heuristics[goal],lns.trajs[i],lns.mem,start,goal, &(lns.neighbors));
    add_traj(lns,i);
    update_dist_2_path(lns,i);
}

}