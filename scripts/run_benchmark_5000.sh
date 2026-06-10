#!/usr/bin/env bash
set -u

cd "$(dirname "$0")/.."
exec ./scripts/run_benchmark.sh 5000
