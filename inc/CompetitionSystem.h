#pragma once
// #include "BasicSystem.h"
#include "SharedEnv.h"
#include "Grid.h"
#include "Tasks.h"
#include "ActionModel.h"
#include "Plan.h"
#include "Entry.h"
#include "Logger.h"
#include "TaskManager.h"
#include "DelayGenerator.h"
#include <pthread.h>
#include <future>
#include "Simulator.h"
#include <memory>


/**
 * BaseSystem — 竞赛系统的核心中央控制器
 *
 * 职责：
 *   1. 管理双速率控制循环（慢速规划 + 快速执行）
 *   2. 协调 Simulator、TaskManager、Entry/Planner 的交互
 *   3. 处理规划与执行的并行化
 *   4. 保存和输出仿真结果
 *
 * 核心架构：
 *   仿真开始 → 预处理 → 首次规划 → 主循环(规划+执行并行)
 */
class BaseSystem
{
public:
    // ============================================================
    // 公共成员
    // ============================================================

    Logger* logger = nullptr;  // 日志记录器，用于输出调试信息和错误


    // ============================================================
    // 构造函数与析构函数
    // ============================================================

    /**
     * 构造函数 — 初始化竞赛系统
     *
     * 参数：
     *   grid           - 地图数据（包含障碍物信息）
     *   planner        - Entry 规划器入口（包含 MAPFPlanner 和 TaskScheduler）
     *   executor       - 执行器（处理动作执行和碰撞检测）
     *   start_locs     - 所有 agent 的起始位置列表
     *   tasks          - 所有预定义任务列表
     *   model          - 动作模型（碰撞检测参数）
     *   max_counter    - 每个 action 需要的最大 tick 数（默认10）
     */
	BaseSystem(Grid &grid, Entry* planner, Executor* executor, std::vector<int>& start_locs,
               std::vector<list<int>>& tasks, ActionModelWithRotate* model, int max_counter = 10):
        map(grid), planner(planner), env(planner->env),
        task_manager(tasks, start_locs.size()), simulator(grid, start_locs, model, executor, max_counter)
    {
        num_of_agents = start_locs.size();
        starts.resize(num_of_agents);

        // 检查每个 agent 的起始位置是否为障碍物
        for (size_t i = 0; i < start_locs.size(); i++)
        {
            if (grid.map[start_locs[i]] == 1)  // 1 = 障碍物
            {
                cout << "error: agent " << i << "'s start location is an obstacle(" << start_locs[i] << ")" << endl;
                exit(0);
            }
            // 创建初始状态：位置=start_locs[i], 时间步=0, 朝向=0(北)
            starts[i] = State(start_locs[i], 0, 0);
        }
    };


    /**
     * 析构函数 — 安全退出
     *
     * 注意：
     *   - 如果有规划线程在运行，等待其结束（join）
     *   - 删除 planner（如果非空）
     */
	virtual ~BaseSystem()
    {
        // 安全退出：先等待规划线程结束，再删除 planner
        if (started)
        {
            task_td.join();
        }
        if (planner != nullptr)
        {
            delete planner;
        }
    };


    // ============================================================
    // 配置方法（setter）
    // ============================================================

    /// 设置任务池目标大小（相对于 agent 数量）
    void set_num_tasks_reveal(float num){task_manager.set_num_tasks_reveal(num);};


    /**
     * 设置各种时间限制参数
     *
     * 参数：
     *   initial              - 首次规划时间限制（毫秒）
     *   comm                - 两次规划之间的最小间隔（毫秒）
     *   move                - 每个 action 的执行时间（毫秒）
     *   process_new_plan    - 执行器处理新计划的时间限制（毫秒）
     */
    void set_plan_time_limit(int initial, int comm, int move, int process_new_plan){
        initial_plan_time_limit = initial;
        min_comm_time = comm;
        simulator_time_limit = move;
        process_new_plan_time_limit = process_new_plan;
    };


    /// 设置预处理时间限制（毫秒）
    void set_preprocess_time_limit(int limit){preprocess_time_limit = limit;};


    /// 设置日志级别
    void set_log_level(int level){log_level = level;};


    /**
     * 设置日志记录器
     *
     * 注意：logger 会同时传递给 TaskManager
     */
    void set_logger(Logger* logger){
        this->logger = logger;
        task_manager.set_logger(logger);
    }


    /// 设置延迟生成器（用于模拟随机延迟）
    void set_delay_generator(std::unique_ptr<DelayGenerator> generator)
    {
        simulator.set_delay_generator(std::move(generator));
    }


    // ============================================================
    // 核心仿真方法
    // ============================================================

    /**
     * 开始仿真主循环
     *
     * 参数：
     *   simulation_time - 仿真总时长（毫秒）
     *   chunk_size     - 每次规划的时间窗口（固定100ms）
     *
     * 流程：预处理 → 首次规划 → 主仿真循环
     */
    void simulate(int simulation_time, int chunk_size);


    /// 规划器包装函数（供线程调用）
    bool planner_wrapper();


    /**
     * 保存仿真结果到 JSON 文件
     *
     * 参数：
     *   fileName      - 输出文件路径
     *   screen        - 输出详细程度（1=最详细，2=中等，3=最少）
     *   pretty_print  - 是否格式化输出（缩进）
     */
    void saveResults(const string &fileName, int screen, bool pretty_print = false) const;


    // ============================================================
    // 受保护成员（子类可访问）
    // ============================================================

protected:
    // ----------------------------------------
    // 地图与仿真参数
    // ----------------------------------------

    Grid map;                // 地图数据（障碍物网格）
    int simulation_time;     // 仿真总时长（毫秒）


    // ----------------------------------------
    // 规划相关数据
    // ----------------------------------------

    /**
     * proposed_plan — 规划器返回的路径计划
     *
     * 结构：plan.actions[i] = agent i 的动作序列
     * 动作类型：FW=0(前进), CR=1(右转), CCR=2(左转), W=3(等待)
     */
    Plan proposed_plan;


    /**
     * proposed_schedule — 任务调度方案
     *
     * 结构：proposed_schedule[i] = agent i 被分配的任务 ID
     *       -1 表示无任务分配
     *
     * 示例：[0, 2, -1, 1]
     *   → agent 0 做任务 0
     *   → agent 1 做任务 2
     *   → agent 2 无任务
     *   → agent 3 做任务 1
     */
    vector<int> proposed_schedule;


    // ----------------------------------------
    // 线程与异步控制
    // ----------------------------------------

    int total_timetous = 0;  // 规划器超时次数统计


    std::future<bool> future;   // 异步规划任务的结果
    std::thread task_td;        // 规划器运行线程
    bool started = false;       // 标记：规划器是否正在运行


    // ----------------------------------------
    // 组件指针
    // ----------------------------------------

    Entry* planner;            // 规划器入口（包含 MAPFPlanner 和 TaskScheduler）
    SharedEnvironment* env;    // 共享环境（供规划器读取状态）


    // ----------------------------------------
    // 时间限制参数（毫秒）
    // ----------------------------------------

    int preprocess_time_limit = 10;        // 预处理时间限制

    int plan_time_limit = 0;               // 当前规划的时间限制（动态计算）

    int initial_plan_time_limit = 1000;    // 首次规划时间限制（默认1秒）
    int min_comm_time = 1000;              // 两次规划之间最小间隔（默认1秒）
    int simulator_time_limit = 100;         // 每个 action 执行时间（默认100ms）
    int process_new_plan_time_limit = 100; // 执行器处理新计划的时间限制


    // ----------------------------------------
    // Agent 相关
    // ----------------------------------------

    vector<State> starts;   // 所有 agent 的起始状态
    int num_of_agents;       // agent 总数


    // ----------------------------------------
    // 日志与调试
    // ----------------------------------------

    int log_level = 1;       // 日志详细程度


    // ----------------------------------------
    // 事件记录
    // ----------------------------------------

    /**
     * events — 记录所有任务完成事件
     *
     * 结构：list of tuple (timestep, agent_id, task_id, seq_id)
     *   - timestep  : 事件发生的时间步
     *   - agent_id  : 完成任务的 agent ID
     *   - task_id   : 被完成的任务 ID
     *   - seq_id    : 任务中的子任务序号
     */
    vector<list<std::tuple<int,int,std::string>>> events;


    // ----------------------------------------
    // 评估指标
    // ----------------------------------------

    vector<int> solution_costs;  // 每个 agent 完成任务的总成本（makespan）
                                 // 注意：当前代码中未被实际使用，可能是遗留代码

    list<double> planner_times;  // 每次规划的实际耗时记录（用于分析）

    bool fast_mover_feasible = true;  // 快速移动器可行性标记


    // ============================================================
    // 私有方法
    // ============================================================

    /// 仿真前初始化（预处理阶段）
    void initialize();

    /// 规划器初始化（预处理阶段）
    bool planner_initialize();


    // ============================================================
    // 核心组件（组合关系）
    // ============================================================

    TaskManager task_manager;  // 任务管理器（负责任务池、调度）
    Simulator simulator;       // 仿真器（负责动作执行、碰撞检测）


    /// 同步共享环境（将 simulator 和 task_manager 的状态同步到 env）
    virtual void sync_shared_env();


    // 动作相关（当前版本未使用）
    void move(vector<Action>& actions);
    bool valid_moves(vector<State>& prev, vector<Action>& next);


    /// 记录预处理结果
    void log_preprocessing(bool succ);
};


/* ============================================================
 * 以下为已注释掉的遗留代码（原 TaskAssignSystem）
 * 保留作为参考，不参与编译
 * ============================================================
// class TaskAssignSystem : public BaseSystem
// {
// public:
// 	TaskAssignSystem(Grid &grid, MAPFPlanner* planner, std::vector<int>& start_locs,
//                      std::vector<int>& tasks, ActionModelWithRotate* model):
//         BaseSystem(grid, planner, model)
//     {
//         int task_id = 0;
//         for (auto& task_location: tasks)
//         {
//             all_tasks.emplace_back(task_id++, task_location);
//             task_queue.emplace_back(all_tasks.back().task_id, all_tasks.back().locations.front());
//         }
//         num_of_agents = start_locs.size();
//         starts.resize(num_of_agents);
//         for (size_t i = 0; i < start_locs.size(); i++)
//         {
//             starts[i] = State(start_locs[i], 0, 0);
//         }
//     };
//
// 	~TaskAssignSystem(){};
//
//
// private:
//     deque<Task> task_queue;  // 任务队列（按顺序分配给 agent）
//
// 	void update_tasks();
// };
*/
