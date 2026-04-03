import MAPF
from typing import Dict, List, Tuple, Set
import sys
import traceback

# 导入 EECBS Planner
from EECBSPlanner import EECBSPlanner

class pyMAPFPlanner:
    def __init__(self, pyenv=None) -> None:
        if pyenv is not None:
            self.env = pyenv.env
        print("pyMAPFPlanner (EECBS) created!", flush=True)
        # 创建 EECBS Planner 实例
        self.eecbs = EECBSPlanner(self.env, w=1.2)

    def initialize(self, preprocess_time_limit: int):
        print("[Python] initialize() called", flush=True)
        return True

    def plan(self, time_limit):
        print(f"[Python] plan() called, time_limit={time_limit}", flush=True)
        try:
            # 使用 EECBS Planner
            actions, _ = self.eecbs.plan(time_limit)
            print(f"[Python] plan() returning {len(actions)} actions", flush=True)
            return actions
        except Exception as e:
            print(f"[Python] plan() ERROR: {e}", flush=True)
            traceback.print_exc()
            return [MAPF.Action.W] * len(self.env.curr_states)
