#!/usr/bin/env bash
set -euo pipefail
export_root=${1:?Usage: bash run_native.sh EXPORT_ROOT [CPU]}
benchmark_cpu=${2:-2}
script_dir=$(cd "$(dirname "$0")" && pwd)
export_root=$(cd "$export_root" && pwd)
for case_dir in "$export_root"/n*_w*; do
    (
        cd "$case_dir/solver"
        gcc -O3 -DNDEBUG -march=native -Iinc/public -Iinc/private -I. \
            src/*.c workspace.c "$script_dir/replay.c" -lm -o ../replay
        taskset -c "$benchmark_cpu" ../replay ../trace.bin ../structure.bin > ../native.jsonl
        taskset -c "$benchmark_cpu" ../replay ../trace.bin ../structure.bin paced > ../paced.jsonl
    )
done
