# MAPF Benchmark Scenarios

This document records the local benchmark set that mirrors the main contest-style scenarios available in `example_problems`.

The goal is to run large tests serially while keeping terminal output, and therefore assistant token usage, small.

## Scenario Set

All scenarios use summary-only output (`-c 3`). The standard local benchmark entrypoints are `1000` and `5000` timesteps.

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

These scripts run one scenario at a time. Full logs go to `benchmark_logs/`; only final summaries are printed.

```bash
cd /mnt/f/MAPF/MAPF-Research

./scripts/run_benchmark_1000.sh
./scripts/run_benchmark_5000.sh
```

For an arbitrary timestep count, use:

```bash
./scripts/run_benchmark.sh 8000
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
  prefix="${problem}_5000"
  cat "benchmark_logs/${prefix}.time"
  python3 - "$prefix" <<'PY'
import json
import pathlib
import sys

prefix = sys.argv[1]
data = json.loads((pathlib.Path("benchmark_logs") / f"{prefix}.json").read_text())
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
tail -100 benchmark_logs/iron-example_10000_5000.log
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
