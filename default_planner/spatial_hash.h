#pragma once

#include <unordered_map>
#include <vector>
#include <utility>

namespace DefaultPlanner {

// ============================================================
// SpatialHash — 空间哈希加速碰撞检测
// ============================================================
// 将地图划分为固定大小的网格，每个格子维护该区域内所有 agent
// 使得碰撞检测从 O(n²) 降低到 O(k) 其中 k 是相邻格子内的平均 agent 数

class SpatialHash {
public:
    // 网格大小（单位：格子数）
    // CELL_SIZE = 2 意味着每个网格是 2x2 的区域
    static const int CELL_SIZE = 2;

private:
    int cols;           // 地图列数
    int rows;           // 地图行数
    int hash_cols;      // 哈希表列数 = cols / CELL_SIZE
    int hash_rows;      // 哈希表行数 = rows / CELL_SIZE

    // 空间哈希表：cell_id -> 该格子内的 agent ID 列表
    std::unordered_map<int, std::vector<int>> cells;

    // Agent 位置缓存：agent_id -> (location, cell_id)
    // 用于快速判断 agent 是否移动到了新的格子
    std::unordered_map<int, std::pair<int, int>> agent_positions;

    // 计算位置对应的格子 ID
    inline int get_cell_id(int location) const {
        int row = location / cols;
        int col = location % cols;
        int cell_row = row / CELL_SIZE;
        int cell_col = col / CELL_SIZE;
        return cell_row * hash_cols + cell_col;
    }

    // 获取格子 ID 对应的哈希键
    inline int hash_key(int cell_id) const {
        return cell_id;
    }

public:
    // 构造函数
    // @param cols 地图列数
    // @param rows 地图行数
    SpatialHash(int cols = 0, int rows = 0) : cols(cols), rows(rows) {
        hash_cols = (cols + CELL_SIZE - 1) / CELL_SIZE;
        hash_rows = (rows + CELL_SIZE - 1) / CELL_SIZE;
    }

    // 清空哈希表
    void clear() {
        cells.clear();
        agent_positions.clear();
    }

    // 更新单个 agent 的位置
    // @param agent_id agent ID
    // @param location 新的位置
    void update_agent(int agent_id, int location) {
        int new_cell_id = get_cell_id(location);

        // 如果 agent 已经在同一个格子里，只更新位置缓存
        auto it = agent_positions.find(agent_id);
        if (it != agent_positions.end() && it->second.second == new_cell_id) {
            it->second.first = location;
            return;
        }

        // 从旧格子移除
        if (it != agent_positions.end()) {
            int old_cell_id = it->second.second;
            auto cell_it = cells.find(old_cell_id);
            if (cell_it != cells.end()) {
                auto& agent_list = cell_it->second;
                for (auto list_it = agent_list.begin(); list_it != agent_list.end(); ++list_it) {
                    if (*list_it == agent_id) {
                        agent_list.erase(list_it);
                        break;
                    }
                }
            }
        }

        // 添加到新格子
        agent_positions[agent_id] = {location, new_cell_id};
        cells[hash_key(new_cell_id)].push_back(agent_id);
    }

    // 批量更新所有 agent 的位置
    // @param locations 位置数组，locations[agent_id] = location
    // @param num_agents agent 总数
    void update_all(const std::vector<int>& locations, int num_agents) {
        clear();
        for (int i = 0; i < num_agents; i++) {
            if (locations[i] >= 0) {  // 忽略无效位置
                update_agent(i, locations[i]);
            }
        }
    }

    // 获取指定位置相邻格子内的所有 agent
    // @param location 查询的位置
    // @param exclude_agent 排除的 agent ID（通常是当前正在检查的 agent）
    // @return 相邻格子内的 agent ID 列表
    std::vector<int> get_nearby_agents(int location, int exclude_agent = -1) const {
        std::vector<int> result;

        int row = location / cols;
        int col = location % cols;

        // 计算相邻的格子（当前格子 + 周围的 8 个格子）
        for (int dr = -1; dr <= 1; dr++) {
            for (int dc = -1; dc <= 1; dc++) {
                int cell_row = (row / CELL_SIZE) + dr;
                int cell_col = (col / CELL_SIZE) + dc;

                // 检查边界
                if (cell_row < 0 || cell_row >= hash_rows || cell_col < 0 || cell_col >= hash_cols) {
                    continue;
                }

                int cell_id = cell_row * hash_cols + cell_col;
                auto it = cells.find(cell_id);
                if (it != cells.end()) {
                    for (int agent_id : it->second) {
                        if (agent_id != exclude_agent) {
                            result.push_back(agent_id);
                        }
                    }
                }
            }
        }

        return result;
    }

    // 检查指定位置是否有其他 agent
    // @param location 查询的位置
    // @param exclude_agent 排除的 agent ID
    // @return true 表示有冲突
    bool has_conflict(int location, int exclude_agent = -1) const {
        int cell_id = get_cell_id(location);
        auto it = cells.find(cell_id);
        if (it == cells.end()) {
            return false;
        }
        for (int agent_id : it->second) {
            if (agent_id != exclude_agent) {
                return true;
            }
        }
        return false;
    }

    // 获取当前区域的 agent 密度（用于自适应策略）
    // @param location 中心位置
    // @return 每格子的平均 agent 数
    double get_density(int location) const {
        int count = 0;
        int row = location / cols;
        int col = location % cols;

        for (int dr = -1; dr <= 1; dr++) {
            for (int dc = -1; dc <= 1; dc++) {
                int cell_row = (row / CELL_SIZE) + dr;
                int cell_col = (col / CELL_SIZE) + dc;

                if (cell_row < 0 || cell_row >= hash_rows || cell_col < 0 || cell_col >= hash_cols) {
                    continue;
                }

                int cell_id = cell_row * hash_cols + cell_col;
                auto it = cells.find(cell_id);
                if (it != cells.end()) {
                    count += static_cast<int>(it->second.size());
                }
            }
        }

        // 返回平均每个格子的 agent 数
        return static_cast<double>(count) / 9.0;
    }
};

}  // namespace DefaultPlanner