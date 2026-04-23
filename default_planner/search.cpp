


#include "search.h"


namespace DefaultPlanner{
std::chrono::nanoseconds t;

/// A* 算法变体：在存在交通流量的情况下寻找冲突最小的路径
///
/// 与标准 A* 的区别：
/// - 标准 A* 只最小化路径长度 (g = 步数)
/// - 本算法同时最小化：路径长度 + 对向流量 + 顶点流量
///
/// 关键概念：
/// - op_flow: 对向流量，走相反方向的 agent 数量（造成拥堵）
/// - all_vertex_flow: 顶点流量，经过某顶点的 agent 总数
/// - flow[loc].d[d]: 从位置 loc 朝方向 d 移动的 agent 数量
///
/// @param env       共享环境（地图、agent数量等）
/// @param flow      流量表，flow[loc].d[d] = 位置loc朝方向d的流量
/// @param ht        启发式距离表（按需懒加载）
/// @param traj      输出：搜索到的路径
/// @param mem       内存池（避免频繁new/delete）
/// @param start     起点位置（一维索引）
/// @param goal      目标位置（一维索引）
/// @param ns        邻居表
/// @return          目标搜索节点
s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns)
{
    mem.reset();  // 重置内存池

    int expanded=0;    // 扩展的节点数（调试用）
    int generated=0;   // 生成的节点数（调试用）
    int h;             // 启发式估计值

    // 如果启发式表还没初始化，用曼哈顿距离
    if(ht.empty())
        h = manhattanDistance(start,goal,env);
    else
        h = get_heuristic(ht,env, start, ns);

    // 创建根节点（起点）
    // 参数：位置, g值(0), h值, op_flow(0), depth(0)
    s_node* root = mem.generate_node(start,0, h,0,0,0);

    // 起点即终点？直接返回
    if (start == goal){
        traj.clear();
        traj.push_back(start);
        return *root;
    }

    pqueue_min_of open;  // A* 的开放列表（最小堆，按 f = g+h 排序）
    re_of re;            // 节点比较函数（用于判断是否需要更新）

    open.push(root);

    // 临时变量声明
    int  diff, d, cost, op_flow, total_cross, all_vertex_flow, vertex_flow, depth, p_diff, p_d;
    int next_d1, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    double tie_breaker, decay_factor;

    s_node* goal_node = nullptr;  // 找到的目标节点
    int neighbors[4];              // 当前节点的四个邻居
    int next_neighbors[4];

    // ============================================================
    // A* 主循环
    // ============================================================
    while (open.size() > 0){
        // 取出 f 值最小的节点
        s_node* curr = open.pop();
        curr->close();  // 标记为已关闭

        // 到达目标？退出
        if (curr->id == goal){
            goal_node = curr;
            break;
        }

        expanded++;
        getNeighborLocs(ns, neighbors, curr->id);  // 获取四个邻居

        // 遍历四个方向
        for (int i=0; i<4; i++){
            int next = neighbors[i];
            if (next == -1) continue;  // 无邻居（边界或障碍）

            // --------------------
            // 计算代价 g
            // --------------------
            cost = curr->g + 1;  // 每步代价为1

            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1;

            // --------------------
            // 计算启发式 h
            // --------------------
            if(ht.empty())
                h = manhattanDistance(next, goal, env);
            else
                h = get_heuristic(ht, env, next, ns);

            // --------------------
            // 计算方向相关代价
            // --------------------
            diff = next - curr->id;
            d = get_d(diff, env);  // diff 转方向编号 (0=东,1=南,2=西,3=北)

            // tie_breaker：平局打破
            // 优先选择直线移动（前一个方向 == 当前方向），减少拐弯
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
                p_d = get_d(p_diff, env);
                if (p_d != d)
                    tie_breaker = 0.1;  // 转弯 +0.1 惩罚
                else
                    tie_breaker = 0;     // 直线无惩罚
            }

            // --------------------
            // op_flow：对向流量代价
            // --------------------
            // 对向流量 = (当前边流量+1) × (下一边反方向流量)
            // 例如：我要往东走，如果有人要往西走经过这里，流量就高
            // (d+2)%4 是反方向：东<->西，南<->北
            temp_op = ( (flow[curr->id].d[d] + 1) * flow[next].d[(d+2)%4] );
            op_flow = temp_op;  // 初始值

            // --------------------
            // all_vertex_flow：顶点流量代价
            // --------------------
            // 经过某顶点的 agent 总数越多，拥堵越严重
            temp_vertex = 1;  // 自己经过一次
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];  // 加上所有出边的流量
            }
            all_vertex_flow = (temp_vertex - 1) / 2;

            op_flow += curr->op_flow;
            all_vertex_flow += curr->all_vertex_flow;

            // --------------------
            // 创建临时节点（用于比较）
            // --------------------
            s_node temp_node(next, cost, h, op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow, all_vertex_flow);

            // --------------------
            // 尝试加入开放列表
            // --------------------
            if (!mem.has_node(next)){
                // 新节点：创建并加入
                s_node* next_node = mem.generate_node(next, cost, h, op_flow, depth, all_vertex_flow);
                next_node->parent = curr;
                next_node->tie_breaker = tie_breaker;
                open.push(next_node);
                generated++;
            }
            else{
                // 节点已存在：检查是否需要更新
                s_node* existing = mem.get_node(next);

                if (!existing->is_closed()){
                    // 节点还在开放列表中
                    if (re(temp_node, *existing)){
                        // 临时节点更优，更新
                        existing->g = cost;
                        existing->parent = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow, all_vertex_flow);
                        open.decrease_key(existing);  // 更新堆中的 key
                    }
                }
                else{
                    // 节点已关闭（已扩展过）
                    if (re(temp_node, *existing)){
                        // 不应该发生！输出错误
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }
                }
            }
        }
    }

    // 未找到路径？报错
    if (goal_node == nullptr){
        std::cout << "error in astar: no path found " << start << "," << goal << std::endl;
        assert(false);
        exit(1);
    }

    // 从目标节点回溯，构建路径
    traj.resize(goal_node->depth + 1);
    s_node* curr = goal_node;
    for (int i = goal_node->depth; i >= 0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }

    return *goal_node;
}
}