#include "TaskManager.h"
#include "Tasks.h"
#include "nlohmann/json.hpp"
#include <vector>

using json = nlohmann::ordered_json;

/**
 * 此函数用于验证参赛者提出的调度方案（任务分配）
 *
 * @param assignment 任务ID向量，每个agent对应一个任务ID，向量长度应等于agent数量
 */
bool TaskManager::validate_task_assignment(vector<int>& assignment)
{
    if (assignment.size() != num_of_agents)
    {
        schedule_errors.push_back(make_tuple("Invalid schedule size",-1,-1,-1,curr_timestep+1));
        logger->log_warning("Scheduler Error: assignment size does not match number of agents",curr_timestep+1);
        return false;
    }

    unordered_map<int,int> idx_set;

    //here we only check the first assignment to each agent
    for (int i_agent = 0; i_agent < assignment.size(); i_agent ++)
    {
        // task should be a ongoing task
        if (assignment[i_agent] != -1 && ongoing_tasks.find(assignment[i_agent]) == ongoing_tasks.end())
        {
            assignment[i_agent] = -1;
            schedule_errors.push_back(make_tuple("task already finished",assignment[i_agent],i_agent,-1,curr_timestep+1));
            logger->log_warning("Scheduler Error: schedule agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already finished",curr_timestep+1);
            logger->flush();
            return false;
        }

        // one task should not appear in the assignment twice
        if (assignment[i_agent] != -1 && idx_set.find(assignment[i_agent]) != idx_set.end())
        {
            assignment[i_agent] = -1;
            schedule_errors.push_back(make_tuple("task is already assigned by the second agent at the same time",assignment[i_agent],i_agent,idx_set[assignment[i_agent]],curr_timestep+1));
            logger->log_warning("Scheduler Error: schedule agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already assigned to agent " + std::to_string(idx_set[assignment[i_agent]]),curr_timestep+1);
            return false;
        }

        // if agent is already executing some task, it should be assigned the same task.
        if (current_assignment[i_agent] != -1 && ongoing_tasks[current_assignment[i_agent]]->idx_next_loc > 0 && assignment[i_agent] != current_assignment[i_agent])
        {
            assignment[i_agent] = current_assignment[i_agent];
            schedule_errors.push_back(make_tuple("task is already opened by the second agent",assignment[i_agent],i_agent,ongoing_tasks[current_assignment[i_agent]]->agent_assigned,curr_timestep+1));
            logger->log_warning("Scheduler Error: schedule agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already opened by agent " + std::to_string(ongoing_tasks[current_assignment[i_agent]]->agent_assigned),curr_timestep+1);
            return false;
        }

        // if agent assign a task that is opened by other, it should not be assigned to the task.
        if (assignment[i_agent] != -1 && ongoing_tasks[assignment[i_agent]]->idx_next_loc > 0 && ongoing_tasks[assignment[i_agent]]->agent_assigned != i_agent)
        {
            assignment[i_agent] = -1;
            schedule_errors.push_back(make_tuple("task is already opened by other agent",assignment[i_agent],i_agent,ongoing_tasks[assignment[i_agent]]->agent_assigned,curr_timestep+1));
            logger->log_warning("Scheduler Error: schedule agent " + std::to_string(i_agent) + " to task " + std::to_string(assignment[i_agent]) + " wrong because the task is already opened by agent " + std::to_string(ongoing_tasks[assignment[i_agent]]->agent_assigned),curr_timestep+1);
            return false;
        }

        if (assignment[i_agent] != -1)
        {
            idx_set[assignment[i_agent]] = i_agent;
        }
    }

    return true;
}


/**
 * 此函数用于更新 agent 的当前任务分配。
 * 它首先检查提议的分配是否有效，然后更新当前分配并更新每个受影响任务的相应 agent_assigned。
 *
 * @param assignment 任务ID向量，每个agent对应一个任务ID，向量长度应等于agent数量
 */
bool TaskManager::set_task_assignment(vector< int>& assignment)
{
    for (int a = 0; a < assignment.size(); a++)
    {
        if (planner_schedule[a].empty() || assignment[a] != planner_schedule[a].back().second)
        {
            planner_schedule[a].push_back(make_pair(curr_timestep,assignment[a]));
        }
    }
    // if (! validate_task_assignment(assignment))
    // {
    //     return false;
    // }

    //reset all the agent_assigned to -1, so that any droped task->agent_assignment will be -1
    for (int a = 0; a < assignment.size(); a++)
    {
        if (current_assignment[a] >= 0){
            ongoing_tasks[current_assignment[a]]->agent_assigned = -1;
        }
    }

    // then set the updated agent_assigned according to new assignments.
    for (int a = 0; a < assignment.size(); a++)
    {
        int t_id = assignment[a];
        current_assignment[a] = t_id;
        if (assignment[a] < 0)
        {
            continue;
        }
        ongoing_tasks[t_id]->agent_assigned = a;
    }

    for (int a = 0; a < current_assignment.size(); a++)
    {
        if (actual_schedule[a].empty() || current_assignment[a] != actual_schedule[a].back().second)
        {
            actual_schedule[a].push_back(make_pair(curr_timestep,current_assignment[a]));
        }
    }

    return true;
}

/**
 * 此函数用于检查当前时间步是否有任务完成。
 * 如果任务完成，则更新任务的完成时间和 agent 的当前分配。
 *
 * @param states 所有agent的状态向量，包括每个agent在地图上的当前位置
 * @param timestep 当前时间步
 */
list<int> TaskManager::check_finished_tasks(vector<State>& states, int timestep)
{ 
    list<int> finished_agents_this_timestep; // task id
    //new_freeagents.clear(); //prepare to push all new free agents to the shared environment
    for (int k = 0; k < num_of_agents; k++)
    {
        if (current_assignment[k] != -1 && states[k].location == ongoing_tasks[current_assignment[k]]->get_next_loc())
        {
            Task * task = ongoing_tasks[current_assignment[k]];
            task->idx_next_loc += 1;

            if (task->is_finished())
            {
                current_assignment[k] = -1;
                ongoing_tasks.erase(task->task_id);
                task->t_completed = timestep;

                finished_agents_this_timestep.push_back(k);
                finished_tasks[task->agent_assigned].emplace_back(task);
                num_of_task_finish++;
                new_freeagents.push_back(k); // record the new free agent
                logger->log_info("Agent " + std::to_string(task->agent_assigned) + " finishes task " + std::to_string(task->task_id), timestep);
                logger->flush();
            }
            events.push_back(make_tuple(timestep,k,task->task_id,task->idx_next_loc));
            // cout<<"open or finished"<<endl;
        }
    }
    return finished_agents_this_timestep;
}

/**
 * 此函数用于将共享环境与当前任务管理器同步。
 * 它将当前任务池、当前任务调度、新空闲的agent和新任务复制到共享环境。
 *
 * @param env 指向共享环境的指针
 */
void TaskManager::sync_shared_env(SharedEnvironment* env) 
{
    env->task_pool.clear();
    for (auto& task: ongoing_tasks)
    {
        env->task_pool[task.first] = *task.second;
    }
    env->curr_task_schedule = current_assignment;
    env->new_freeagents = new_freeagents;
    env->new_tasks = new_tasks; 
}

/**
 * 此函数用于在当前时间步揭示新任务。
 * 它在每个时间步揭示固定数量的任务，并将它们添加到进行中的任务、新任务和所有任务列表中。
 *
 * @param timestep 当前时间步
 */
void TaskManager::reveal_tasks(int timestep)
{
    //new_tasks.clear(); //prepare to push all new revealed tasks to the shared environment
    while (ongoing_tasks.size() < num_tasks_reveal)
    {
        int i = task_id%tasks.size();
        list<int> locs = tasks[i];
        Task* task = new Task(task_id,locs,timestep);
        ongoing_tasks[task->task_id] = task;
        all_tasks.push_back(task);
        new_tasks.push_back(task->task_id);         // record the new tasks
        logger->log_info("Task " + std::to_string(task_id) + " is revealed");
        task_id++;
    }
}

/**
 * 此函数负责任务管理流程：
 * 1. 使用参赛者提出的调度方案更新agent的当前分配
 * 2. 检查当前时间步是否有任务完成
 * 3. 在当前时间步揭示新任务
 *
 * @param states 所有agent的状态向量，包括每个agent在地图上的当前位置
 * @param assignment 任务ID向量，每个agent对应一个任务ID，向量长度应等于agent数量
 * @param timestep 当前时间步
 */
void TaskManager::update_tasks(vector<State>& states, vector<int>& assignment, int timestep)
{
    curr_timestep = timestep;
    set_task_assignment(assignment);
    list<int> finsihed_agents = check_finished_tasks(states,timestep);
    for (int agent_id: finsihed_agents)
    {
        assignment[agent_id] = -1; //reset the assignment of finished agents
    }
    reveal_tasks(timestep);
}

/**
 * 此函数将所有任务转换为 JSON 对象
 *
 * @param map_cols 地图的列数
 */
json TaskManager::to_json(int map_cols) const
{
    
    json tasks = json::array();
    for (auto t: all_tasks)
    {
        json task = json::array();
        task.push_back(t->task_id);
        // TODO rewrite the task output part
        // task.push_back(t->locations.front()/map_cols);
        // task.push_back(t->locations.front()%map_cols);
        task.push_back(t->t_revealed);
        json locs = json::array();
        for (auto loc: t->locations)
        {
            locs.push_back(loc/map_cols);
            locs.push_back(loc%map_cols);
        }
        task.push_back(locs);
        tasks.push_back(task);
    }
    return tasks;
}
