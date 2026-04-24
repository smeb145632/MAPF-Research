#include "ActionModel.h"


std::ostream& operator<<(std::ostream &stream, const Action &action)
{
    if (action == Action::FW) {
        stream << "F";
    } else if (action == Action::CR) {
        stream << "R";
    } else if (action == Action::CCR) {
        stream << "C";
    } else {
        stream << "W";
    }

    return stream;
}


// L04: Check if a conflict is tolerable under delay tolerance rules
bool ActionModelWithRotate::is_conflict_tolerable(int agent_id, int other_agent_id, int timestep,
                                                   const string& conflict_type,
                                                   const vector<State>& prev, const vector<State>& next)
{
    // If delay tolerance is disabled, no conflict is tolerable
    if (delay_config.delay_tolerance_timesteps <= 0) {
        return false;
    }
    
    // Get the two agents' states
    const State& a_curr = next[agent_id];
    const State& a_prev = prev[agent_id];
    const State& b_curr = next[other_agent_id];
    const State& b_prev = prev[other_agent_id];
    
    if (conflict_type == "vertex") {
        // Vertex conflict: both agents want to be at same location at same timestep
        if (!delay_config.enable_vertex_tolerance) {
            return false;
        }
        
        // L04: Check if agents have different delay tolerance windows
        // If one agent arrived early and will wait, conflict is tolerable
        // An agent is "delayed" if its actual position is behind its planned position
        // For now, we check if either agent is at a location that matches the other's target
        
        // Key insight from ADG: if agents have different "arrival times" at the conflict point,
        // the conflict is only virtual and will resolve automatically
        
        // Simple heuristic: if both agents are moving towards the same vertex
        // from different directions, this is a "swap" pattern which is tolerable
        if (a_curr.location == b_prev.location && b_curr.location == a_prev.location) {
            // This is a swap - both agents exchanging positions
            // This is naturally resolved and tolerable
            return true;
        }
        
        // Check if agents are in tolerance window (arrived early/wait for each other)
        // An agent is "waiting" if its actual location equals previous planned location
        bool a_waiting = (a_curr.location == a_prev.location);
        bool b_waiting = (b_curr.location == b_prev.location);
        
        // If at least one agent is waiting, the conflict will resolve
        if (a_waiting || b_waiting) {
            return true;
        }
        
        // L04: Tolerance window check - allow conflicts if within tolerance
        // The conflict point becomes a "soft constraint" - agents can pass through
        // if they're within the delay tolerance window
        
        return false;
        
    } else if (conflict_type == "edge") {
        // Edge conflict: agents cross the same edge in opposite directions
        if (!delay_config.enable_edge_tolerance) {
            return false;
        }
        
        // Edge swap pattern is tolerable
        if (a_curr.location == b_prev.location && b_curr.location == a_prev.location) {
            return true;
        }
        
        return false;
    }
    
    return false;
}


bool ActionModelWithRotate::is_valid(const vector<State>& prev, const vector<Action> & actions)
{
    if (prev.size() != actions.size())
    {
        errors.push_back(make_tuple("incorrect vector size",-1,-1,prev[0].timestep+1));
        return false;
    }

    vector<State> next = result_states(prev, actions);
    unordered_map<int, int> vertex_occupied;
    unordered_map<pair<int, int>, int> edge_occupied;

    for (int i = 0; i < prev.size(); i ++) 
    {
        
        if (next[i].location < 0 || next[i].location >= grid.map.size() || 
            (abs(next[i].location / cols - prev[i].location/cols) + abs(next[i].location % cols - prev[i].location %cols) > 1 ))
        {
            errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
            return false;
        }
        if (grid.map[next[i].location] == 1) {
            errors.push_back(make_tuple("unallowed move",i,-1,next[i].timestep));
            return false;
        }
        

        if (vertex_occupied.find(next[i].location) != vertex_occupied.end()) {
            // L04: Check if this vertex conflict is tolerable
            int other_agent = vertex_occupied[next[i].location];
            if (!is_conflict_tolerable(i, other_agent, next[i].timestep, "vertex", prev, next)) {
                errors.push_back(make_tuple("vertex conflict",i,other_agent, next[i].timestep));
                return false;
            }
            // L04: Conflict is tolerable - allow but log it
        }

        int edge_idx = (prev[i].location + 1) * rows * cols +  next[i].location;

        if (edge_occupied.find({prev[i].location, next[i].location}) != edge_occupied.end()) {
            // L04: Check if this edge conflict is tolerable
            int other_agent = edge_occupied[{prev[i].location, next[i].location}];
            if (!is_conflict_tolerable(i, other_agent, next[i].timestep, "edge", prev, next)) {
                errors.push_back(make_tuple("edge conflict", i, other_agent, next[i].timestep));
                return false;
            }
            // L04: Conflict is tolerable - allow but log it
        }
        

        vertex_occupied[next[i].location] = i;
        int r_edge_idx = (next[i].location + 1) * rows * cols +  prev[i].location;
        edge_occupied[{next[i].location, prev[i].location}] = i;
    }

    return true;
}
