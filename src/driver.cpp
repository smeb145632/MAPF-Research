/**
 * @file driver.cpp
 * @brief MAPF-Research 竞赛系统的入口点
 *
 * 本文件是整个程序的入口，负责：
 * 1. 解析命令行参数
 * 2. 加载问题配置（地图、agent初始位置、任务）
 * 3. 初始化竞赛系统（BaseSystem）
 * 4. 运行模拟
 * 5. 保存结果到 JSON 文件
 *
 * ======================== 程序整体流程 ========================
 *
 * 输入：JSON 格式的问题描述文件
 *       ├── mapFile: 地图文件路径
 *       ├── agentFile: agent 初始位置文件路径
 *       ├── taskFile: 任务文件路径
 *       └── 其他配置参数...
 *
 * 输出：JSON 格式的结果文件
 *       ├── actualPaths: 每个 agent 的实际执行路径
 *       ├── plannerPaths: 每个 agent 的规划路径
 *       ├── errors: 冲突/错误信息
 *       ├── events: 任务分配/完成事件
 *       └── 统计信息...
 *
 * ======================== 核心组件关系 ========================
 *
 *   driver.cpp (入口)
 *        │
 *        ▼
 *   BaseSystem (竞赛系统核心)
 *        │
 *        ├── Entry (规划器入口)
 *        │     ├── MAPFPlanner (路径规划)
 *        │     └── TaskScheduler (任务调度)
 *        │
 *        ├── Simulator (模拟器)
 *        │     └── Executor (执行器)
 *        │
 *        └── TaskManager (任务管理器)
 *
 * ======================== 双速率循环 ========================
 *
 * 模拟采用双速率循环：
 * - 慢循环（规划线程）：规划器计算路径，耗时较长
 * - 快循环（主线程）：模拟器每个 tick 执行动作，耗时短
 *
 * 时间轴：
 * [规划阶段] ──► [执行 tick 1] ──► [执行 tick 2] ──► ... ──► [新规划阶段] ──► ...
 *                (100ms)           (100ms)                      (1000ms+)
 *
 * 每个规划阶段生成一批动作（staged_actions），然后在多个 tick 中逐步执行。
 */

#include "CompetitionSystem.h"
#include "Evaluation.h"
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include "nlohmann/json.hpp"
#include <signal.h>
#include <climits>
#include <memory>
#include <stdexcept>


#ifdef PYTHON
#if PYTHON
#include "pyMAPFPlanner.hpp"
#include <pybind11/embed.h>
#include "pyEntry.hpp"
#include "pyTaskScheduler.hpp"
#endif
#endif

namespace po = boost::program_options;
using json = nlohmann::json;

/**
 * @brief 全局变量：存储解析后的命令行参数
 *
 * vm (variables_map) 存储所有命令行参数的值，
 * 在整个程序中都可以访问这些参数。
 */
po::variables_map vm;

/**
 * @brief 全局指针：指向竞赛系统实例
 *
 * 使用 unique_ptr 管理内存，确保程序退出时自动释放资源。
 * 在信号处理函数中也需要访问，所以设为全局变量。
 */
std::unique_ptr<BaseSystem> system_ptr;

/**
 * @brief SIGINT 信号处理函数（Ctrl+C 中断）
 *
 * 当用户按 Ctrl+C 时，保存当前结果并安全退出程序。
 * 这确保即使模拟被中断，已有的计算结果也不会丢失。
 *
 * @param a 信号编号（未使用）
 */
void sigint_handler(int a)
{
    fprintf(stdout, "正在停止模拟...\n");

    // 将当前模拟结果保存到 JSON 文件
    system_ptr->saveResults(
        vm["output"].as<std::string>(),      // 输出文件路径
        vm["outputScreen"].as<int>(),        // 输出详细程度
        vm["prettyPrintJson"].as<bool>()     // 是否格式化输出
    );

    _exit(0);  // 强制退出
}


/**
 * @brief 程序入口点
 *
 * main 函数是整个程序的起点，负责初始化和运行模拟。
 *
 * ======================== 命令行参数说明 ========================
 *
 * 必要参数：
 *   -i, --inputFile     输入问题描述文件（JSON格式）【必须】
 *
 * 可选参数：
 *   -o, --output         输出结果文件（默认: ./output.json）
 *   -s, --simulationTime 模拟时间长度，以 tick 为单位（默认: 5000）
 *   -n, --initialPlanTimeLimit  首次规划时间限制，毫秒（默认: 1000）
 *   -t, --planCommTimeLimit     两次规划之间的最小间隔，毫秒（默认: 1000）
 *   -a, --actionMoveTimeLimit   每个 action 的执行时间，毫秒（默认: 100）
 *   -p, --preprocessTimeLimit   预处理时间限制，毫秒（默认: 30000）
 *   -x, --executorProcessPlanTimeLimit 执行器处理新计划的时间限制（默认: 100）
 *   -c, --outputScreen   输出详细程度: 1=完整, 2=仅实际路径, 3=仅统计
 *   -d, --logDetailLevel 日志详细程度: 1=全部, 2=警告+错误, 3=仅错误
 *   -l, --logFile        日志文件路径
 *   -f, --fileStoragePath 大文件存储路径
 *   -w, --outputActionWindow 输出窗口大小（默认: 100）
 *   -m, --evaluationMode  评估模式（用于评估已有输出文件）
 *   --prettyPrintJson     格式化输出 JSON
 *
 * ======================== 示例命令 ========================
 *
 * 运行模拟：
 *   ./build/lifelong --inputFile ./example_problems/random.domain/random_32_32_20_100.json -o output.json
 *
 * 显示帮助：
 *   ./build/lifelong --help
 *
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 程序退出码
 */
int main(int argc, char **argv)
{
    // ============================================================
    // 阶段 1: Python 解释器初始化（仅在 PYTHON 模式下）
    // ============================================================
    // 如果使用 Python 绑定，需要初始化 Python 解释器
    // 这允许在 C++ 程序中运行 Python 代码
#ifdef PYTHON
#if PYTHON
    pybind11::initialize_interpreter();
#endif
#endif

    // ============================================================
    // 阶段 2: 解析命令行参数
    // ============================================================
    // 使用 boost::program_options 库解析命令行参数
    // 这提供了一种方便的方式来处理命令行选项

    // 创建一个选项描述对象，包含所有支持的选项
    po::options_description desc("支持的选项");
    desc.add_options()
        // 帮助信息
        ("help", "显示帮助信息")
        // 输入文件【必需】
        ("inputFile,i", po::value<std::string>()->required(), "输入问题描述文件路径（JSON格式）")
        // 输出文件
        ("output,o", po::value<std::string>()->default_value("./output.json"), "输出结果文件路径（JSON格式）")
        // 模拟时间
        ("simulationTime,s", po::value<int>()->default_value(5000), "模拟时间长度（tick数）")
        // 首次规划时间限制
        ("initialPlanTimeLimit,n", po::value<int>()->default_value(1000), "首次规划时间限制（毫秒）")
        // 规划通信间隔（两次规划之间的最小间隔）
        ("planCommTimeLimit,t", po::value<int>()->default_value(1000), "两次规划之间的最小间隔（毫秒）")
        // 每个 action 的执行时间
        ("actionMoveTimeLimit,a", po::value<int>()->default_value(100), "每个action的执行时间（毫秒）")
        // 预处理时间限制
        ("preprocessTimeLimit,p", po::value<int>()->default_value(30000), "预处理时间限制（毫秒）")
        // 执行器处理新计划的时间限制
        ("executorProcessPlanTimeLimit,x", po::value<int>()->default_value(100), "执行器处理新计划的时间限制（毫秒）")
        // 输出详细程度
        ("outputScreen,c", po::value<int>()->default_value(1), "输出详细程度: 1=完整, 2=仅实际路径和统计, 3=仅统计")
        // 日志详细程度
        ("logDetailLevel,d", po::value<int>()->default_value(1), "日志详细程度: 1=全部, 2=警告+错误, 3=仅错误")
        // 日志文件
        ("logFile,l", po::value<std::string>()->default_value(""), "日志文件路径")
        // 大文件存储路径
        ("fileStoragePath,f", po::value<std::string>()->default_value(""), "大文件存储路径")
        // 输出窗口大小
        ("outputActionWindow,w", po::value<int>()->default_value(100), "输出窗口大小")
        // 评估模式
        ("evaluationMode,m", po::value<bool>()->default_value(false), "评估模式（评估已有输出文件）")
        // 格式化输出
        ("prettyPrintJson", po::bool_switch()->default_value(false), "格式化输出JSON")
    ;

    // 记录程序开始时间（用于计算总运行时间）
    clock_t start_time = clock();

    // 解析命令行参数
    po::store(po::parse_command_line(argc, argv, desc), vm);

    // 如果用户请求帮助，显示帮助信息并退出
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    // 检查必需参数并将其存储到变量中
    po::notify(vm);

    // ============================================================
    // 阶段 3: 设置日志系统
    // ============================================================
    // 根据命令行参数确定日志级别
    // 日志级别映射：命令行值 -> Boost.Log 值
    // 1 (全部) -> 2 (info), 2 (警告+错误) -> 3 (warning), 3 (仅错误) -> 5 (fatal)
    int log_level = vm["logDetailLevel"].as<int>();
    if (log_level <= 1)
        log_level = 2;  // info 级别
    else if (log_level == 2)
        log_level = 3;  // warning 级别
    else
        log_level = 5;  // fatal 级别

    // 创建日志记录器
    Logger *logger = new Logger(vm["logFile"].as<std::string>(), log_level);

    // ============================================================
    // 阶段 4: 验证输出目录
    // ============================================================
    // 确保输出文件所在的目录存在
    std::filesystem::path filepath(vm["output"].as<std::string>());
    if (filepath.parent_path().string().size() > 0 && !std::filesystem::is_directory(filepath.parent_path()))
    {
        logger->log_fatal("输出目录不存在", 0);
        _exit(1);
    }

    // ============================================================
    // 阶段 5: 创建规划器（Entry）和执行器（Executor）
    // ============================================================
    // Entry 是规划器的入口类，负责调用 MAPFPlanner 和 TaskScheduler
    // Executor 负责执行规划器生成的动作序列

    Entry *planner = nullptr;
    Executor *executor = nullptr;

#ifdef PYTHON
#if PYTHON
    // 如果使用 Python 模式，创建 Python 版本的规划器
    planner = new PyEntry();
#else
    // 使用 C++ 版本的默认规划器
    planner = new Entry();
    // 创建执行器，管理 staged_actions（暂存的动作序列）
    executor = new Executor(planner->env);
#endif
#endif

    // ============================================================
    // 阶段 6: 加载输入问题描述
    // ============================================================
    // 输入 JSON 文件包含：
    // - mapFile: 地图文件路径
    // - agentFile: agent 初始位置文件路径
    // - taskFile: 任务文件路径
    // - teamSize: agent 数量
    // - maxCounter: 完成任务所需的 tick 数
    // - delayConfig: 延迟配置
    // - 等等...

    auto input_json_file = vm["inputFile"].as<std::string>();
    json data;

    // 从输入文件路径中提取目录，作为相对文件的基准路径
    boost::filesystem::path p(input_json_file);
    boost::filesystem::path dir = p.parent_path();
    std::string base_folder = dir.string();
    if (base_folder.size() > 0 && base_folder.back() != '/')
    {
        base_folder += "/";
    }

    // 打开并解析输入 JSON 文件
    std::ifstream f(input_json_file);
    try
    {
        data = json::parse(f);
    }
    catch (json::parse_error error)
    {
        std::cerr << "无法加载文件: " << input_json_file << std::endl;
        std::cerr << "错误信息: " << error.what() << std::endl;
        exit(1);
    }

    // ============================================================
    // 阶段 7: 加载地图
    // ============================================================
    // 从 JSON 中读取地图文件路径，加载地图数据
    auto map_path = read_param_json<std::string>(data, "mapFile");
    Grid grid(base_folder + map_path);  // base_folder 是输入文件所在的目录

    // 保存地图名称到环境对象中
    planner->env->map_name = map_path.substr(map_path.find_last_of("/") + 1);

    // ============================================================
    // 阶段 8: 设置大文件存储路径
    // ============================================================
    // 某些大规模问题可能需要额外的存储空间
    string file_storage_path = vm["fileStoragePath"].as<std::string>();
    if (file_storage_path == ""){
        // 如果命令行没有指定，尝试从环境变量读取
        char const* tmp = getenv("LORR_LARGE_FILE_STORAGE_PATH");
        if (tmp != nullptr) {
            file_storage_path = string(tmp);
        }
    }

    // 检查路径是否存在，如果不存在则记录警告
    if (file_storage_path != "" && !std::filesystem::exists(file_storage_path)){
        std::ostringstream stringStream;
        stringStream << "fileStoragePath (" << file_storage_path << ") 无效";
        logger->log_warning(stringStream.str());
    }
    planner->env->file_storage_path = file_storage_path;

    // ============================================================
    // 阶段 9: 读取 agent 大小配置
    // ============================================================
    // agent_size 用于基于重叠的碰撞检测
    float agent_size = read_param_json<float>(data, "agentSize", 1.0f);
    if (agent_size <= 0.0f)
    {
        throw std::invalid_argument("agentSize 必须是正数");
    }

    // ============================================================
    // 阶段 10: 创建动作模型
    // ============================================================
    // ActionModelWithRotate 定义了机器人的动作和碰撞检测规则
    // 支持的动作：FW（前移）、CR（顺时针旋转）、CCR（逆时针旋转）、W（等待）
    ActionModelWithRotate *model = new ActionModelWithRotate(grid, agent_size);
    model->set_logger(logger);

    // ============================================================
    // 阶段 11: 加载 agent 初始位置
    // ============================================================
    // team_size 是问题中机器人的数量
    // agents 是包含所有机器人初始位置索引的向量
    int team_size = read_param_json<int>(data, "teamSize");

    // ============================================================
    // 阶段 12: 解析延迟配置
    // ============================================================
    // 延迟配置用于模拟真实世界中的不确定性
    // 支持的模型：Bernoulli（伯努利）、Poisson（泊松）
    // 持续时间模型：Uniform（均匀）、Gaussian（高斯）
    DelayConfig delay_config;
    try
    {
        delay_config = parse_delay_config(data);
    }
    catch (const std::invalid_argument& error)
    {
        logger->log_fatal(error.what(), 0);
        _exit(1);
    }

    // ============================================================
    // 阶段 13: 加载 agent 初始位置和任务列表
    // ============================================================
    // agents: 向量，每个元素是对应 agent 的初始位置（位置索引）
    // tasks: 向量，每个元素是一个任务，任务是位置索引的列表
    std::vector<int> agents = read_int_vec(base_folder + read_param_json<std::string>(data, "agentFile"), team_size);
    std::vector<list<int>> tasks = read_int_vec(base_folder + read_param_json<std::string>(data, "taskFile"));

    // 警告：如果 agent 数量多于任务数量
    if (agents.size() > tasks.size())
        logger->log_warning("Agent 数量多于任务数量，可能有 agent 处于空闲状态");

    // ============================================================
    // 阶段 14: 创建竞赛系统（核心系统）
    // ============================================================
    // BaseSystem 是整个模拟系统的核心，它管理：
    // - Simulator: 执行模拟
    // - Executor: 管理动作执行
    // - TaskManager: 管理任务
    // - Entry: 规划器入口
    system_ptr = std::make_unique<BaseSystem>(
        grid,                      // 地图
        planner,                   // 规划器入口
        executor,                  // 执行器
        agents,                    // agent 初始位置
        tasks,                     // 任务列表
        model,                     // 动作模型
        read_param_json<int>(data, "maxCounter", 10)  // 每个 action 需要的 tick 数
    );

    // ============================================================
    // 阶段 15: 配置系统参数
    // ============================================================

    // 设置日志记录器
    system_ptr->set_logger(logger);

    // 设置时间限制参数
    // 参数顺序：首次规划时间、通信间隔、每 action 执行时间、执行器处理时间
    system_ptr->set_plan_time_limit(
        vm["initialPlanTimeLimit"].as<int>(),          // 首次规划时间限制（毫秒）
        vm["planCommTimeLimit"].as<int>(),              // 规划通信间隔（毫秒）
        vm["actionMoveTimeLimit"].as<int>(),            // action 执行时间（毫秒）
        vm["executorProcessPlanTimeLimit"].as<int>()    // 执行器处理时间（毫秒）
    );

    // 设置预处理时间限制
    system_ptr->set_preprocess_time_limit(vm["preprocessTimeLimit"].as<int>());

    // 设置每个时间步揭示的任务数量
    // numTasksReveal 是任务池中应保持的任务数量（相对于 team_size）
    system_ptr->set_num_tasks_reveal(read_param_json<float>(data, "numTasksReveal", 1));

    // ============================================================
    // 阶段 16: 创建延迟生成器
    // ============================================================
    // 延迟生成器模拟机器人执行动作时的不确定性
    // 例如：机器人在移动时可能遇到障碍物、机械故障等导致的延迟
    try
    {
        system_ptr->set_delay_generator(std::make_unique<DelayGenerator>(delay_config, team_size));
    }
    catch (const std::invalid_argument& error)
    {
        logger->log_fatal(error.what(), 0);
        _exit(1);
    }

    // ============================================================
    // 阶段 17: 注册信号处理函数
    // ============================================================
    // 注册 SIGINT 信号处理（Ctrl+C）
    // 这样当用户中断程序时，可以保存当前结果
    signal(SIGINT, sigint_handler);

    // ============================================================
    // 阶段 18: 运行模拟（核心阶段）
    // ============================================================
    // simulate() 是整个程序的核心，它会：
    // 1. 进行预处理（规划器的初始化）
    // 2. 进入主循环，交替执行规划和执行
    // 3. 直到达到指定的模拟时间
    system_ptr->simulate(vm["simulationTime"].as<int>(), 100);  // 100 是 chunk_size，用于输出压缩

    // ============================================================
    // 阶段 19: 保存结果并退出
    // ============================================================
    // 将模拟结果保存到 JSON 文件
    system_ptr->saveResults(
        vm["output"].as<std::string>(),      // 输出文件路径
        vm["outputScreen"].as<int>(),        // 输出详细程度
        vm["prettyPrintJson"].as<bool>()     // 是否格式化
    );

    // 清理资源
    delete model;
    delete logger;

    _exit(0);  // 正常退出
}


/**
 * ======================== 输入文件格式 ========================
 *
 * 输入 JSON 文件示例：
 * {
 *   "mapFile": "maps/random-32-32-20.map",
 *   "agentFile": "agents/random_32_32_20_100.agents",
 *   "taskFile": "tasks/random_32_32_20.tasks",
 *   "teamSize": 100,
 *   "numTasksReveal": 1.0,
 *   "agentSize": 1.0,
 *   "maxCounter": 10,
 *   "delayConfig": {
 *     "seed": 0,
 *     "minDelay": 1,
 *     "maxDelay": 5,
 *     "eventModel": "bernoulli",
 *     "pDelay": 0.1,
 *     "durationModel": "uniform"
 *   }
 * }
 *
 * ======================== 输出文件格式 ========================
 *
 * 输出 JSON 文件包含：
 * - actionModel: 动作模型名称（"MAPF_T"）
 * - teamSize: agent 数量
 * - makespan: 模拟总时间步
 * - numTaskFinished: 完成的任务数
 * - actualPaths: 每个 agent 的实际执行路径（压缩格式）
 * - plannerPaths: 每个 agent 的规划路径（压缩格式）
 * - errors: 冲突和错误信息
 * - events: 任务事件列表
 * - scheduleErrors: 调度错误列表
 * - plannerTimes: 每次规划的计算时间
 * - 等等...
 *
 * ======================== 位置编码 ========================
 *
 * 地图上的位置使用线性索引（不是 row, col）：
 *   location = row * width + col
 *
 * 例如：5x4 的地图（5行，4列）
 *   位置 0  1  2  3
 *   位置 4  5  6  7
 *   位置 8  9  10 11
 *   位置 12 13 14 15
 *   位置 16 17 18 19
 *
 * 方向编码：
 *   0 = East（东）
 *   1 = South（南）
 *   2 = West（西）
 *   3 = North（北）
 */
