#include "sipps.h"
#include "const.h"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <cmath>

namespace DefaultPlanner {

// Get soft obstacle indicator at vertex v for time t
// Returns true if there's significant traffic (soft obstacle)
bool hasSoftObstacleAt(SharedEnvironment* env, std::vector<Int4>& flow, int v, int t) {
    int total_flow = 0;
    for (int d = 0; d < 4; d++) {
        total_flow += flow[v].d[d];
    }
    return total_flow > SOFT_OBSTACLE_FLOW_THRESHOLD;
}

// Get soft edge obstacle count
// Returns the number of agents that will traverse the edge (from->to) at time t
int getSoftEdgeObstacleCount(SharedEnvironment* env, std::vector<Int4>& flow, int from, int to, int t) {
    int diff = to - from;
    if (diff == 0) return 0;  // waiting
    
    int d = -1;
    if (diff == 1) d = 0;      // east
    else if (diff == env->cols) d = 1;  // south
    else if (diff == -1) d = 2;  // west
    else if (diff == -env->cols) d = 3; // north
    
    if (d == -1) return 0;
    
    // flow[from].d[d] = agents moving from 'from' in direction d
    // These agents will be at 'to' at time t+1
    return flow[from].d[d];
}

// Build safe interval table for a vertex
// Returns intervals [low, high) where soft obstacle status is constant
void buildSafeIntervalTable(
    SharedEnvironment* env,
    std::vector<Int4>& flow,
    std::vector<std::vector<SafeInterval>>& T,
    int max_time
) {
    int n = env->map.size();
    T.assign(n, {});
    
    for (int v = 0; v < n; v++) {
        if (env->map[v] == 1) {
            // Obstacle - no safe intervals
            continue;
        }
        
        // Build intervals with potentially different soft obstacle status
        // We sample at key timesteps and create intervals between them
        int t = 0;
        while (t < max_time) {
            bool has_soft = hasSoftObstacleAt(env, flow, v, t);
            int next_change = max_time;
            
            // Find next timestep where soft status changes
            // For simplicity, we check a few future timesteps
            for (int dt = 1; dt <= 10 && t + dt < max_time; dt++) {
                if (hasSoftObstacleAt(env, flow, v, t + dt) != has_soft) {
                    next_change = t + dt;
                    break;
                }
            }
            
            T[v].push_back(SafeInterval(t, next_change, has_soft, false));
            t = next_change;
        }
    }
}

// Extract path from goal node to start node
void extractSIPPSPath(sipps_node* goal_node, Traj& traj) {
    traj.clear();
    std::vector<int> reversed;
    sipps_node* curr = goal_node;
    
    while (curr != nullptr) {
        reversed.push_back(curr->v);
        curr = curr->parent;
    }
    
    std::reverse(reversed.begin(), reversed.end());
    traj = reversed;
}

// SIPPS algorithm - Safe Interval Path Planning with Soft constraints
// Minimizes path length while minimizing soft collisions (traffic flow)
int sipps(
    SharedEnvironment* env,
    std::vector<Int4>& flow,
    HeuristicTable& ht,
    Traj& traj,
    int start,
    int goal,
    Neighbors* ns,
    int max_time
) {
    // Handle trivial case
    if (start == goal) {
        traj.clear();
        traj.push_back(start);
        return 0;
    }
    
    // Build safe interval table
    std::vector<std::vector<SafeInterval>> T;
    buildSafeIntervalTable(env, flow, T, max_time);
    
    if (T[start].empty() || T[goal].empty()) {
        traj.clear();
        return -1;
    }
    
    // Priority queue: primary by c (soft collisions), secondary by f (g+h)
    std::priority_queue<sipps_node*, std::vector<sipps_node*>, typename sipps_node::compare> open;
    
    // Closed set: (vertex, interval_id) pairs
    std::unordered_set<uint64_t> closed;
    auto make_key = [](int v, int id) -> uint64_t {
        return (static_cast<uint64_t>(v) << 32) | static_cast<uint32_t>(id);
    };
    
    // Memory management
    std::vector<sipps_node*> all_nodes;
    all_nodes.reserve(5000);
    
    auto make_node = [&](int v, int low, int high, int id, bool is_goal, 
                         int g, int h, int c, sipps_node* parent) -> sipps_node* {
        sipps_node* node = new sipps_node(v, low, high, id, is_goal, g, h, c, parent);
        all_nodes.push_back(node);
        return node;
    };
    
    // T_min: lower bound on travel time
    int T_min = ht.empty() ? manhattanDistance(start, goal, env) 
                            : get_heuristic(ht, env, start, ns);
    
    // Create root node with first safe interval at start
    int h0 = ht.empty() ? manhattanDistance(start, goal, env) 
                         : get_heuristic(ht, env, start, ns);
    int c0 = T[start][0].has_soft_vertex ? 1 : 0;
    
    sipps_node* root = make_node(start, 0, T[start][0].high, 0, false, 0, h0, c0, nullptr);
    open.push(root);
    
    sipps_node* goal_node = nullptr;
    int goal_c = INT_MAX;
    
    // Get neighbors helper
    int neighbors[4];
    
    // Main search loop
    while (!open.empty()) {
        sipps_node* curr = open.top();
        open.pop();
        
        uint64_t key = make_key(curr->v, curr->id);
        if (closed.find(key) != closed.end()) {
            continue;
        }
        closed.insert(key);
        
        // Goal check (lines 10-15 from paper)
        if (curr->v == goal && curr->low >= T_min) {
            // Count future soft collisions at goal
            int c_future = 0;
            // Check if goal has soft obstacles after curr->low
            for (int t = curr->low; t < curr->low + T_min && t < max_time; t++) {
                if (hasSoftObstacleAt(env, flow, goal, t)) {
                    c_future++;
                }
            }
            
            if (c_future == 0) {
                goal_node = curr;
                goal_c = curr->c;
                break;
            } else {
                // Create goal node with future collisions
                sipps_node* goal_copy = make_node(
                    curr->v, curr->low, curr->high, curr->id, true,
                    curr->g, curr->h, curr->c + c_future, curr->parent
                );
                open.push(goal_copy);
                continue;
            }
        }
        
        // Expand node (Algorithm 2 in paper)
        getNeighborLocs(ns, neighbors, curr->v);
        
        int curr_time = curr->low;
        
        // Move to neighbors (lines 1-5 of Algorithm 2)
        for (int i = 0; i < 4; i++) {
            int next_v = neighbors[i];
            if (next_v == -1) continue;  // Invalid neighbor
            
            // Find safe intervals at next_v that overlap with [curr_time + 1, curr_time + 2)
            for (int sid = 0; sid < (int)T[next_v].size(); sid++) {
                const SafeInterval& si = T[next_v][sid];
                
                // Check if we can arrive at next_v within this interval
                int arrival = curr_time + 1;
                if (arrival < si.low) continue;
                if (arrival >= si.high) continue;
                
                // Valid move found
                int new_g = std::max(arrival, si.low);
                
                // Calculate soft collision cost
                int c_add = si.has_soft_vertex ? 1 : 0;
                c_add += (getSoftEdgeObstacleCount(env, flow, curr->v, next_v, curr_time) > 0) ? 1 : 0;
                
                int new_c = curr->c + c_add;
                
                // Heuristic
                int h = ht.empty() ? manhattanDistance(next_v, goal, env) 
                                    : get_heuristic(ht, env, next_v, ns);
                
                uint64_t next_key = make_key(next_v, sid);
                if (closed.find(next_key) != closed.end()) {
                    continue;
                }
                
                sipps_node* next_node = make_node(
                    next_v, new_g, si.high, sid, false,
                    new_g, h, new_c, curr
                );
                open.push(next_node);
            }
        }
        
        // Wait at current vertex (stay within same interval)
        const SafeInterval& curr_si = T[curr->v][curr->id];
        if (curr_time < curr_si.high && curr_si.high > curr_time + 1) {
            // Can wait until interval ends
            // Check if there's a next interval to transition to
            for (int next_sid = 0; next_sid < (int)T[curr->v].size(); next_sid++) {
                if (next_sid == curr->id) continue;
                const SafeInterval& next_si = T[curr->v][next_sid];
                
                if (next_si.low == curr_si.high || 
                    (next_si.low <= curr_si.high && curr_si.high < next_si.high)) {
                    
                    uint64_t next_key = make_key(curr->v, next_sid);
                    if (closed.find(next_key) != closed.end()) {
                        continue;
                    }
                    
                    int c_add = next_si.has_soft_vertex ? 1 : 0;
                    int new_c = curr->c + c_add;
                    
                    sipps_node* wait_node = make_node(
                        curr->v, curr_si.high, next_si.high, next_sid, false,
                        curr_si.high, curr->h, new_c, curr
                    );
                    open.push(wait_node);
                }
            }
        }
    }
    
    // Extract path if found
    if (goal_node != nullptr) {
        extractSIPPSPath(goal_node, traj);
    } else {
        traj.clear();
    }
    
    // Cleanup
    for (auto node : all_nodes) {
        delete node;
    }
    
    return goal_c;
}

} // namespace DefaultPlanner