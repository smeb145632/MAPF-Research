#ifndef SIPPS_HPP
#define SIPPS_HPP

#include "Types.h"
#include "utils.h"
#include "Memory.h"
#include "heap.h"
#include "search_node.h"
#include "heuristics.h"

namespace DefaultPlanner{

// Safe interval structure
struct SafeInterval {
    int low;  // inclusive start time
    int high; // exclusive end time (MAX_TIMESTEP means infinite)
    bool has_soft_vertex;  // whether this interval contains soft vertex/target obstacles
    bool has_soft_edge;    // whether this interval contains soft edge obstacles
    
    SafeInterval(int l, int h, bool sv = false, bool se = false) : low(l), high(h), has_soft_vertex(sv), has_soft_edge(se) {}
};

// SIPPS node - differs from s_node by using safe intervals instead of discrete timesteps
struct sipps_node {
    int v;              // vertex/location
    int low;            // earliest arrival time within the safe interval
    int high;           // exclusive end time of safe interval
    int id;             // index into the safe interval table
    bool is_goal;       // whether this is a goal node
    int g;              // arrival time (same as low)
    int h;              // heuristic estimate
    int c;              // number of soft collisions so far
    sipps_node* parent; // parent node
    
    sipps_node(int v, int low, int high, int id, bool is_goal, int g, int h, int c, sipps_node* parent)
        : v(v), low(low), high(high), id(id), is_goal(is_goal), g(g), h(h), c(c), parent(parent) {}
    
    int get_f() const { return g + h; }
    
    // For priority queue ordering: primary by c, secondary by f
    struct compare {
        bool operator()(const sipps_node* lhs, const sipps_node* rhs) const {
            if (lhs->c != rhs->c) return lhs->c > rhs->c;  // min c first
            if (lhs->get_f() != rhs->get_f()) return lhs->get_f() > rhs->get_f();  // min f first
            return lhs->g > rhs->g;  // max g first for tie-breaking
        }
    };
};

// Build safe interval table for a vertex
// Oh = hard obstacles (set of (vertex, timestep) pairs that are impassable)
// Os = soft obstacles (set of (vertex, timestep) or (edge, timestep) pairs we want to avoid)
void buildSafeIntervalTable(
    SharedEnvironment* env,
    std::vector<SafeInterval>& T,  // output: safe interval table per vertex
    const std::vector<std::vector<int>>& soft_vertex_times,  // soft obstacles at each vertex
    int start, int goal
);

// SIPPS algorithm - finds shortest path minimizing soft collisions
// flow represents soft obstacles (other agents' paths)
// Returns the number of soft collisions in the path
int sipps(
    SharedEnvironment* env,
    std::vector<Int4>& flow,  // traffic flow as soft obstacles
    HeuristicTable& ht,
    Traj& traj,
    int start,
    int goal,
    Neighbors* ns,
    int max_time = MAX_TIMESTEP
);

} // namespace DefaultPlanner

#endif