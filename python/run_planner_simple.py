#!/usr/bin/env python3
"""
Standalone pure Python runner for LORR.
Uses a simple random planner without C++ dependencies.
"""

import json
import os
import random

def load_map(map_file_path, problem_dir):
    """Load map from file"""
    if '/' in map_file_path:
        map_file_path = map_file_path.replace('/', '\\')
    full_path = os.path.join(problem_dir, map_file_path)
    full_path = full_path.replace('\\\\', '\\')
    with open(full_path, 'r') as f:
        lines = f.readlines()
    # Skip header (type, height, width, map)
    data_lines = lines[4:]
    rows = len(data_lines)
    cols = len(data_lines[0].strip())
    grid = []
    for line in data_lines:
        for c in line.strip():
            grid.append(1 if c in '@T' else 0)
    return rows, cols, grid

def load_agents(agent_file_path, problem_dir):
    """Load agents from file"""
    if '/' in agent_file_path:
        agent_file_path = agent_file_path.replace('/', '\\')
    full_path = os.path.join(problem_dir, agent_file_path)
    full_path = full_path.replace('\\\\', '\\')
    with open(full_path, 'r') as f:
        lines = f.readlines()
    num_agents = int(lines[0].strip())
    starts = []
    for i in range(1, num_agents + 1):
        loc = int(lines[i].strip())
        starts.append(loc)
    return starts

def load_tasks(task_file_path, problem_dir):
    """Load tasks from file"""
    if '/' in task_file_path:
        task_file_path = task_file_path.replace('/', '\\')
    full_path = os.path.join(problem_dir, task_file_path)
    full_path = full_path.replace('\\\\', '\\')
    with open(full_path, 'r') as f:
        lines = f.readlines()
    num_tasks = int(lines[0].strip())
    tasks = []
    for i in range(1, num_tasks + 1):
        loc = int(lines[i].strip())
        tasks.append(loc)
    return tasks

def idx_to_location(idx, cols):
    return idx // cols, idx % cols

def get_forward(idx, orient, cols, rows):
    """Get forward neighbor"""
    r, c = idx_to_location(idx, cols)
    if orient == 0:  # N
        nr, nc = r - 1, c
    elif orient == 1:  # E
        nr, nc = r, c + 1
    elif orient == 2:  # S
        nr, nc = r + 1, c
    else:  # W
        nr, nc = r, c - 1
    if 0 <= nr < rows and 0 <= nc < cols:
        return nr * cols + nc
    return idx  # Invalid, stay

def run_random_planner():
    """Run the random planner and generate output.json"""
    # Get project root
    script_dir = os.path.dirname(os.path.abspath(__file__))
    base_dir = os.path.dirname(script_dir)
    if base_dir.startswith('/mnt/'):
        parts = base_dir.split('/')
        drive = parts[2].upper()
        rest = '/'.join(parts[3:]).replace('/', '\\')
        base_dir = f"{drive}:\\{rest}"
    
    problem_dir = os.path.join(base_dir, 'example_problems', 'random.domain')
    input_file = os.path.join(problem_dir, 'random_20.json')
    output_file = os.path.join(base_dir, 'output_python.json')
    
    print(f"Loading input from {input_file}", flush=True)
    
    with open(input_file, 'r') as f:
        config = json.load(f)
    
    # Load map
    rows, cols, grid = load_map(config['mapFile'], problem_dir)
    print(f"Map: {rows}x{cols}", flush=True)
    
    # Load agents
    starts = load_agents(config['agentFile'], problem_dir)
    print(f"Agents: {len(starts)}", flush=True)
    
    # Load tasks
    tasks = load_tasks(config['taskFile'], problem_dir)
    print(f"Tasks: {len(tasks)}", flush=True)
    
    # Current states: [location_idx, orientation]
    # 0=N, 1=E, 2=S, 3=W
    states = [[loc, random.randint(0, 3)] for loc in starts]
    
    # Generate actions (random walk)
    num_steps = 50
    all_actions = []
    
    for step in range(num_steps):
        step_actions = []
        for i, (loc, orient) in enumerate(states):
            forward = get_forward(loc, orient, cols, rows)
            
            # Check if forward is valid and free
            valid_forward = forward != loc and grid[forward] == 0
            occupied = any(states[j][0] == forward for j in range(len(states)) if j != i)
            
            if valid_forward and not occupied:
                step_actions.append('F')
                states[i][0] = forward
            else:
                # Random turn or wait
                action = random.choice(['W', 'R', 'C'])
                step_actions.append(action)
                if action == 'R':
                    states[i][1] = (states[i][1] + 1) % 4
                elif action == 'C':
                    states[i][1] = (states[i][1] + 3) % 4
        
        all_actions.append(','.join(step_actions))
    
    # Build output
    dirs = ['N', 'E', 'S', 'W']
    start_locs = []
    for loc, orient in states:
        r, c = idx_to_location(loc, cols)
        start_locs.append([r, c, dirs[orient]])
    
    output = {
        "actionModel": "MAPF_T",
        "AllValid": "Yes",
        "teamSize": len(starts),
        "start": start_locs,
        "numTaskFinished": 0,
        "sumOfCost": 0,
        "makespan": num_steps,
        "actualPaths": all_actions,
        "plannerPaths": all_actions,
        "plannerTimes": [0.001] * num_steps,
        "errors": [],
        "events": [],
        "tasks": []
    }
    
    print(f"Writing output to {output_file}", flush=True)
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=2)
    
    print("Done!", flush=True)

if __name__ == '__main__':
    run_random_planner()
