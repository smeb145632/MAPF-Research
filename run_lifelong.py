#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
方便运行 MAPF-Research lifelong 的封装脚本。

用法:
    python3 run_lifelong.py <problem_name> [options]
    python3 run_lifelong.py warehouse_small_10
    python3 run_lifelong.py random_20 -c 1
    python3 run_lifelong.py warehouse_small_10 -c 2 -s 3000 -o my_output.json

参数:
    <problem_name>            problem 文件名（不含 .json 后缀）
                             脚本会在所有 domain 目录下搜索同名 .json 文件

常用选项:
    -c, --screen   输出详细度: 1=完整(含plannerPaths), 2=仅actualPaths, 3=仅摘要 (默认: 1)
    -s, --sim      仿真步数 (默认: 5000)
    -o, --output   输出文件名 (默认: output.json)
    -n, --plan     初始规划时间限制 ms (默认: 1000)
    -p, --pre      预处理时间限制 ms (默认: 30000)

示例:
    # 跑一个小 case，完整输出，自动弹出 PlanViz
    python3 run_lifelong.py warehouse_small_10

    # 只跑仿真，不出可视化（-c 3）
    python3 run_lifelong.py warehouse_small_50 -c 3

    # 自定义输出文件名
    python3 run_lifelong.py random_100 -o random100_result.json
"""
import sys
import os
import subprocess
import argparse
from pathlib import Path

# 项目根目录
PROJECT_ROOT = Path(__file__).parent.resolve()
LIFELONG_BIN = PROJECT_ROOT / "build" / "lifelong"
PROBLEM_ROOT = PROJECT_ROOT / "example_problems"
PLANVIZ_SCRIPT = Path("/mnt/f/MAPF/PlanViz/script/run_both.py")


def find_problem_file(name: str) -> Path:
    """
    在所有 domain 目录下搜索名为 <name>.json 的文件。
    支持绝对路径。
    """
    if Path(name).exists():
        return Path(name).resolve()

    # 去掉 .json 后缀（如果有）
    base = name
    if base.endswith(".json"):
        base = base[:-5]

    for domain_dir in PROBLEM_ROOT.iterdir():
        if domain_dir.is_dir():
            candidate = domain_dir / f"{base}.json"
            if candidate.exists():
                print(f"[run_lifelong] 找到 problem 文件: {candidate}")
                return candidate

    raise FileNotFoundError(
        f"在 {PROBLEM_ROOT} 下未找到 '{base}.json' 或 '{base}'.json\n"
        f"可用目录: {[d.name for d in PROBLEM_ROOT.iterdir() if d.is_dir()]}"
    )


def build_lifelong_cmd(
    problem_path: Path,
    output_path: Path,
    screen: int,
    sim_time: int,
    plan_time: int,
    pre_time: int,
) -> list[str]:
    return [
        str(LIFELONG_BIN),
        "-i", str(problem_path),
        "-o", str(output_path),
        "-c", str(screen),
        "-s", str(sim_time),
        "-n", str(plan_time),
        "-p", str(pre_time),
    ]


def auto_viz(output_path: Path, problem_path: Path):
    """ lif

ong 成功后自动弹出 PlanViz。 """
    if not PLANVIZ_SCRIPT.exists():
        print(f"[run_lifelong] PlanViz 脚本不存在: {PLANVIZ_SCRIPT}，跳过可视化")
        return

    viz_cmd = [
        sys.executable,
        str(PLANVIZ_SCRIPT),
        str(output_path.resolve()),
        str(problem_path.resolve()),
    ]
    print(f"\n[run_lifelong] 启动 PlanViz 可视化 ...")
    try:
        subprocess.run(viz_cmd)
    except Exception as e:
        print(f"[run_lifelong] 启动 PlanViz 失败: {e}")


def main():
    parser = argparse.ArgumentParser(
        description="运行 MAPF-Research lifelong 的便捷封装",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("problem", help="problem 文件名（不含 .json），或绝对路径")
    parser.add_argument("-c", "--screen", type=int, default=1,
                        help="输出详细度: 1=完整, 2=仅actualPaths, 3=仅摘要 (默认: 1)")
    parser.add_argument("-s", "--sim", type=int, default=5000,
                        help="仿真步数 (默认: 5000)")
    parser.add_argument("-o", "--output", default="output.json",
                        help="输出文件名 (默认: output.json)")
    parser.add_argument("-n", "--plan", type=int, default=1000,
                        help="初始规划时间限制 ms (默认: 1000)")
    parser.add_argument("-p", "--pre", type=int, default=30000,
                        help="预处理时间限制 ms (默认: 30000)")
    parser.add_argument("--no-viz", action="store_true",
                        help="跳过自动可视化")
    parser.add_argument("--viz-only", action="store_true",
                        help="只启动可视化，不运行 lifelong（用已有的 output.json）")

    args = parser.parse_args()

    # 定位 problem 文件
    try:
        problem_path = find_problem_file(args.problem)
    except FileNotFoundError as e:
        print(f"[run_lifelong] 错误: {e}")
        sys.exit(1)

    # 输出文件放 problem 所在目录（与 problem.json 同级）
    output_path = (PROJECT_ROOT / args.output).resolve()

    # viz-only 模式：直接跳可视化
    if args.viz_only:
        if not output_path.exists():
            print(f"[run_lifelong] 错误: output.json 不存在: {output_path}")
            sys.exit(1)
        print(f"[run_lifelong] viz-only 模式，跳过 lifelong，直接启动 PlanViz")
        auto_viz(output_path, problem_path)
        return

    # 构建命令
    cmd = build_lifelong_cmd(
        problem_path, output_path,
        screen=args.screen,
        sim_time=args.sim,
        plan_time=args.plan,
        pre_time=args.pre,
    )

    print(f"[run_lifelong] 运行 lifelong ...")
    print(f"[run_lifelong] problem : {problem_path.name}")
    print(f"[run_lifelong] output  : {output_path}")
    print(f"[run_lifelong] screen  : {args.screen}")
    print(f"[run_lifelong] sim     : {args.sim}")
    print(f"[run_lifelong] 命令    : {' '.join(cmd)}")
    print()

    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"[run_lifelong] lifelong 运行失败，退出码: {result.returncode}")
        sys.exit(result.returncode)

    # 成功后自动弹出 PlanViz
    if not args.no_viz:
        auto_viz(output_path, problem_path)


if __name__ == "__main__":
    main()
