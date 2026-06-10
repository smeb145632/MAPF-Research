# MAPF Benchmark Scenarios

This document records the local benchmark set that mirrors the main contest-style scenarios available in `example_problems`.

The goal is to run large tests serially while keeping terminal output, and therefore assistant token usage, small.

## Scenario Set

All scenarios use `8000` timesteps and summary-only output (`-c 3`).

| Label | Local problem name | Problem file | Agents |
|---|---|---|---:|
| iron | `iron-example_10000` | `example_problems/iron_harvest.domain/iron-example_10000.json` | 10000 |
| fulfill | `fulfill-example_2500` | `example_problems/warehouse.domain/fulfill-example_2500.json` | 2500 |
| orz | `orz-example_1800` | `example_problems/game.domain/orz-example_1800.json` | 1800 |
| rand | `random_800` | `example_problems/random.domain/random_800.json` | 800 |
| room | `room-example_150` | `example_problems/room.domain/room-example_150.json` | 150 |
| maze | `maze-example_40` | `example_problems/maze.domain/maze-example_40.json` | 40 |

## Build First

From WSL:

```bash
cd /mnt/f/MAPF/MAPF-Research
make -C build lifelong -j4
```

If build files are stale:

```bash
cd /mnt/f/MAPF/MAPF-Research
cmake -B build ./ -DCMAKE_BUILD_TYPE=Release
make -C build lifelong -j4
```

## Serial Run

This loop runs one scenario at a time. Full logs go to `benchmark_logs/`; only final summaries are printed.

```bash
cd /mnt/f/MAPF/MAPF-Research
mkdir -p benchmark_logs

for problem in \
  iron-example_10000 \
  fulfill-example_2500 \
  orz-example_1800 \
  random_800 \
  room-example_150 \
  maze-example_40
do
  echo "=== $problem ==="
  /usr/bin/time -f "ELAPSED=%e USER=%U SYS=%S MAXRSS=%M" \
    -o "benchmark_logs/${problem}.time" \
    python3 run_lifelong.py "$problem" -c 3 -s 8000 --no-viz \
      -o "benchmark_logs/${problem}.json" \
      > "benchmark_logs/${problem}.log" 2>&1

  cat "benchmark_logs/${problem}.time"
  python3 - "$problem" <<'PY'
import json
import pathlib
import sys

problem = sys.argv[1]
path = pathlib.Path("benchmark_logs") / f"{problem}.json"
data = json.loads(path.read_text())
for key in [
    "teamSize",
    "makespan",
    "numTaskFinished",
    "numPlannerErrors",
    "numScheduleErrors",
    "numEntryTimeouts",
]:
    print(f"{key}={data.get(key)}")
PY
done
```

## Low Token Result Check

Prefer reading only the time files and JSON summaries:

```bash
cd /mnt/f/MAPF/MAPF-Research

for problem in \
  iron-example_10000 \
  fulfill-example_2500 \
  orz-example_1800 \
  random_800 \
  room-example_150 \
  maze-example_40
do
  echo "=== $problem ==="
  cat "benchmark_logs/${problem}.time"
  python3 - "$problem" <<'PY'
import json
import pathlib
import sys

problem = sys.argv[1]
data = json.loads((pathlib.Path("benchmark_logs") / f"{problem}.json").read_text())
print("teamSize=", data.get("teamSize"))
print("makespan=", data.get("makespan"))
print("numTaskFinished=", data.get("numTaskFinished"))
print("numScheduleErrors=", data.get("numScheduleErrors"))
print("numPlannerErrors=", data.get("numPlannerErrors"))
print("numEntryTimeouts=", data.get("numEntryTimeouts"))
PY
done
```

Only inspect logs on failure:

```bash
tail -100 benchmark_logs/iron-example_10000.log
```

## What To Watch

Primary validity checks:

- `numScheduleErrors` must be `0`.
- `numPlannerErrors` must be `0`.
- `numEntryTimeouts` must be `0`.

Primary performance checks:

- `numTaskFinished`: higher is better.
- `ELAPSED`: wall-clock runtime in seconds.
- `MAXRSS`: peak memory in KB.

