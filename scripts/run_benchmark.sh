#!/usr/bin/env bash
set -u

steps="${1:-}"
if [ -z "$steps" ]; then
  echo "Usage: $0 <timesteps>"
  exit 2
fi

cd "$(dirname "$0")/.."
mkdir -p benchmark_logs

summary="benchmark_logs/full_${steps}_summary.txt"
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
  prefix="${problem}_${steps}"
  echo "=== $problem (${steps}) ===" | tee -a "$summary"

  rm -f \
    "benchmark_logs/${prefix}.time" \
    "benchmark_logs/${prefix}.json" \
    "benchmark_logs/${prefix}.log"

  /usr/bin/time -f "ELAPSED=%e USER=%U SYS=%S MAXRSS=%M" \
    -o "benchmark_logs/${prefix}.time" \
    timeout 1800s \
    python3 run_lifelong.py "$problem" -c 3 -s "$steps" --no-viz \
      -o "benchmark_logs/${prefix}.json" \
      > "benchmark_logs/${prefix}.log" 2>&1

  code=$?
  echo "EXIT=$code" | tee -a "$summary"
  cat "benchmark_logs/${prefix}.time" | tee -a "$summary"

  if [ -f "benchmark_logs/${prefix}.json" ]; then
    python3 - "$prefix" <<'PY' | tee -a "$summary"
import json
import pathlib
import sys

prefix = sys.argv[1]
path = pathlib.Path("benchmark_logs") / f"{prefix}.json"
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
    tail -40 "benchmark_logs/${prefix}.log" | tee -a "$summary"
  fi
done
