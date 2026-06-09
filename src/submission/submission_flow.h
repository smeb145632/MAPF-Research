#ifndef flow_hpp
#define flow_hpp
#include "submission_types.h"
#include "submission_search.h"
#include "submission_traj_lns.h"
#include "submission_heuristics.h"

#include <random>
#include <unordered_set>

namespace SubmissionPlanner{

//remove flow for each location's outgoing edge according to the traj
void remove_traj(TrajLNS& lns, int agent);

void add_traj(TrajLNS& lns, int agent);


void get_deviation(TrajLNS& lns);

void update_fw_metrics(TrajLNS& lns);

void frank_wolfe(TrajLNS& lns,std::unordered_set<int>& updated, TimePoint timelimit);

void update_dist_2_path(TrajLNS& lns, int i);

//compute distance table for each traj
void init_dist_table(TrajLNS& lns, int amount);

//update traj and distance table for agent i (with optional deadline)
bool update_traj(TrajLNS& lns, int i, const TimePoint* deadline = nullptr);

// LNS-style local replanning fallback when update_traj fails
// Returns true if local replanning succeeded
bool lns_local_replan(TrajLNS& lns, int failed_agent);


}
#endif
