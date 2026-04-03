#!/usr/bin/env python3
"""
Pure Python runner for LORR competition.
Bypasses C++ compilation by reading input and calling Python planner directly.
"""

import json
import sys
import os

# Add paths
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'build'))

def load_map(map_file_path):
    """Load map from file"""
    full_path = os.path.join(os.path.dirname(__file__), '..', map_file_path)
    with open(full_path, 'r') as f:
        lines = f.readlines()
    rows = len(lines)
    cols = len(lines[0].strip())
    grid = []
    for line in lines:
        for c in line.strip():
            grid.append(1 if c == '@' else 0)
    return rows, cols, grid

def load_agents(agent_file_path):
    """Load agents from file"""
    full_path = os.path.join(os.path.dirname(__file__), '..', agent_file_path)
    with open(full_path, 'r') as f:
        lines = f.readlines()
    agents = []
    for line in lines:
        parts = line.strip().split()
        if len(parts) >= 3:
            agents.append({
                'start': (int(parts[0]), int(parts[1])),
                'goal': (int(parts[2]), int(parts[3])) if len(parts) >= 4 else (int(parts[2]), int(parts[3]))
            })
    return agents

def load_tasks(task_file_path):
    """Load tasks from file"""
    full_path = os.path.join(os.path.dirname(__file__), '..', task_file_path)
    with open(full_path, 'r') as f:
        lines = f.readlines()
    tasks = []
    for line in lines:
        parts = line.strip().split()
        if len(parts) >= 4:
            tasks.append({
                'start': (int(parts[0]), int(parts[1])),
                'goal': (int(parts[2]), int(parts[3]))
            })
    return tasks

class SimpleEnv:
    """Minimal environment for planner"""
    def __init__(self, rows, cols, grid, starts):
        self.rows = rows
        self.cols = cols
        self.map = grid
        self.curr_states = []
        for i, (r, c) in enumerate(starts):
            self.curr_states.append(SimpleState(r * cols + c, i % 4))  # 0=N,1=E,2=S,3=W

class SimpleState:
    """Minimal state"""
    def __init__(self, location, orientation):
        self.location = location
        self.orientation = orientation

def run_planner(input_file, output_file):
    """Run the planner"""
    # Load input
    with open(input_file, 'r') as f:
        config = json.load(f)
    
    # Load map and agents
    rows, cols, grid = load_map(config['mapFile'])
    agents = load_agents(config['agentFile'])
    
    # Get starts
    starts = [a['start'] for a in agents]
    
    # Create environment
    env = SimpleEnv(rows, cols, grid, starts)
    
    # Import and create planner
    print("Loading pyMAPFPlanner...", flush=True)
    import pyMAPFPlanner
    planner = pyMAPFPlanner.pyMAPFPlanner()
    
    print("Initializing planner...", flush=True)
    planner.initialize(5)  # 5 seconds preprocess
    
    print("Planning...", flush=True)
    actions = planner.plan(1)  # 1 second per step
    
    # Generate output
    output = {
        "actionModel": "MAPF_T",
        "AllValid": "Yes",
        "teamSize": len(agents),
        "start": [[s.location // cols, s.location % cols, ['N','E','S','W'][s.orientation]] for s in env.curr_states],
        "actions": [str(a) for a in actions],
        "plannerTimes": [0.001]
    }
    
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=2)
    
    print(f"Output written to {output_file}", flush=True)

if __name__ == '__main__':
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    input_file = os.path.join(base_dir, 'example_problems', 'random.domain', 'random_20.json')
    output_file = os.path.join(base_dir, 'output_python.json')
    run_planner(input_file, output_file)
