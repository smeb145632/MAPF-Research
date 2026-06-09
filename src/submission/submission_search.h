
#ifndef search_hpp
#define search_hpp

#include "submission_types.h"
#include "submission_utils.h"
#include "submission_memory.h"
#include "submission_heap.h"
#include "submission_search_node.h"
#include "submission_heuristics.h"

namespace SubmissionPlanner{
//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns, const TimePoint* deadline = nullptr);
}

#endif
