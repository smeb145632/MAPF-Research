#ifndef TRAJ_LNS_H
#define TRAJ_LNS_H

#include "Types.h"
#include "Memory.h"
#include "search_node.h"
#include "heap.h"
#include "heuristics.h"
#include <iostream>
#include <set>

namespace DefaultPlanner{

// ============================================================
// 自适应策略枚举 — 决定何时重新规划轨迹
// ============================================================
enum ADAPTIVE {
    RANDOM,     // 随机触发重规划（简单粗暴）
    CONGESTION, // 当某区域 agent 密度过高时触发
    DEVIATION,  // 当轨迹偏离原路径时触发
    COUNT       // 枚举值计数（用于数组大小等）
};


// ============================================================
// 全局启发式表和邻居表（extern 声明，在其他地方定义）
// ============================================================
extern std::vector<HeuristicTable> global_heuristictable;  // 启发式距离表
extern Neighbors global_neighbors;                          // 邻居表


// ============================================================
// FW_Metric — Frank-Wolfe 算法的指标追踪
// ============================================================
struct FW_Metric{
    int id;               // agent ID
    int deviation;        // 偏离程度（实际轨迹 vs 期望轨迹）
    int last_replan_t;    // 上次重规划的时间步
    int rand;             // 随机值（用于自适应策略）

    // 构造函数
    FW_Metric(int i, int d, int l) : id(i), deviation(d), last_replan_t(l){};
    FW_Metric(){};
};


// ============================================================
// FlowHeuristic — 流式启发式（用于路径搜索加速）
// ============================================================
struct FlowHeuristic{
    HeuristicTable* h;    // 指向某个目标的启发式表
    int target;           // 目标位置
    int origin;           // 起点位置
    pqueue_min_of open;   // A* 搜索的优先队列（最小堆）
    MemoryPool mem;       // A* 搜索的内存池

    // 检查是否为空（没有生成任何节点）
    bool empty(){
        return mem.generated() == 0;
    }

    // 重置搜索状态（清空队列和内存池，但保留启发式表）
    void reset(){
        open.clear();
        mem.reset();
    }
};


// ============================================================
// TrajLNS — Trajectory Large Neighborhood Search
// ============================================================
// 核心数据结构，管理所有 agent 的轨迹、任务分配和流量启发式
class TrajLNS{
public:
    // ----------------------------------------
    // 基础环境指针
    // ----------------------------------------
    SharedEnvironment* env;   // 共享环境（地图、agent状态等）

    // ----------------------------------------
    // 任务和轨迹
    // ----------------------------------------
    std::vector<int> tasks;   // tasks[agent_id] = 当前任务ID（-1表示无任务）
    std::vector<Traj> trajs;  // trajs[agent_id] = 该agent的完整轨迹

    // ----------------------------------------
    // 流量统计与偏离
    // ----------------------------------------
    std::vector<std::pair<int,int>> deviation_agents;  // 发生偏离的agent列表 (agent_id, 偏离程度)
    std::vector<Int4> flow;  // flow[location] = {流入, 流出, 占用, 最大占用} 四元组
                              // Int4 = struct{int a,b,c,d;} 用于统计每个格子的人流量

    // ----------------------------------------
    // 启发式相关（引用类型，指向外部数据）
    // ----------------------------------------
    std::vector<HeuristicTable>& heuristics;  // 全局启发式表引用 [目标位置] -> 启发式信息
    std::vector<Dist2Path> traj_dists;         // 每个agent的轨迹到各位置的距离
    std::vector<s_node> goal_nodes;           // 每个agent搜索到的目标节点（包含完整代价信息）

    // ----------------------------------------
    // Frank-Wolfe 算法相关
    // ----------------------------------------
    std::vector<FW_Metric> fw_metrics;  // 每个agent的Frank-Wolfe指标

    // ----------------------------------------
    // 内存池（搜索算法用）
    // ----------------------------------------
    MemoryPool mem;           // 全局内存池（A*搜索等）
    Neighbors& neighbors;     // 邻居表引用

    // ----------------------------------------
    // 统计指标
    // ----------------------------------------
    int traj_inited = 0;       // 已初始化轨迹的agent数量
    int dist2path_inited = 0; // 已初始化距离路径的agent数量
    int soc = 0;              // Sum Of Costs（路径总代价，用于评估）

    // ----------------------------------------
    // 内存初始化
    // ----------------------------------------
    // 为内存池预分配空间，避免搜索过程中反复new/delete
    void init_mem(){
        mem.init(env->map.size());
    }

    // ----------------------------------------
    // 构造函数（带参数）
    // ----------------------------------------
    // 注意：heuristics 和 neighbors 是引用类型，必须在构造时绑定
    TrajLNS(SharedEnvironment* env,
            std::vector<HeuristicTable>& heuristics,
            Neighbors& neighbors)
        : env(env),
          trajs(env->num_of_agents),       // 每个agent一条轨迹
          tasks(env->num_of_agents),        // 每个agent一个任务
          flow(env->map.size(), Int4({0,0,0,0})),  // 每个格子一个流量统计
          heuristics(heuristics),
          traj_dists(env->num_of_agents),   // 每个agent一个距离路径表
          goal_nodes(env->num_of_agents),   // 每个agent一个搜索结果
          fw_metrics(env->num_of_agents),   // 每个agent一个FW指标
          neighbors(neighbors)
    { };

    // ----------------------------------------
    // 默认构造函数
    // ----------------------------------------
    // 使用全局的启发式表和邻居表（在全局变量初始化时使用）
    TrajLNS(): heuristics(global_heuristictable), neighbors(global_neighbors){};

};

}  // namespace DefaultPlanner
#endif
