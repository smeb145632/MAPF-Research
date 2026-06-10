#!/usr/bin/env bash
set -u

cd "$(dirname "$0")/.."
mkdir -p benchmark_logs

summary="benchmark_logs/full_8000_summary.txt"
rm -f "$summary"

problems=(
  iron-example_10000
  fulfill-example_2500
  orz-example_1800
  random_800
  room-example_150
  maze-example_40
)

for problem in "${problems[@]}"; do
  echo "=== $problem ===" | tee -a "$summary"

  rm -f \
    "benchmark_logs/${problem}.time" \
    "benchmark_logs/${problem}.json" \
    "benchmark_logs/${problem}.log"

  /usr/bin/time -f "ELAPSED=%e USER=%U SYS=%S MAXRSS=%M" \
    -o "benchmark_logs/${problem}.time" \
    timeout 1800s \
    python3 run_lifelong.py "$problem" -c 3 -s 8000 --no-viz \
      -o "benchmark_logs/${problem}.json" \
      > "benchmark_logs/${problem}.log" 2>&1

  code=$?
  echo "EXIT=$code" | tee -a "$summary"
  cat "benchmark_logs/${problem}.time" | tee -a "$summary"

  if [ -f "benchmark_logs/${problem}.json" ]; then
    python3 - "$problem" <<'PY' | tee -a "$summary"
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
  else
    echo "NO_JSON_OUTPUT" | tee -a "$summary"
    tail -40 "benchmark_logs/${problem}.log" | tee -a "$summary"
  fi
done
