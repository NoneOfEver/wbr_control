#!/usr/bin/env bash
# NUC-only, bounded replay. No robot transport; no persistent scheduler changes.
set -euo pipefail
if [[ ${EUID} -ne 0 ]]; then
    echo 'Root is needed for SCHED_FIFO; run this script with sudo.' >&2
    exit 1
fi
benchmark_root=/home/wbr-infantry/wbr_mpc_rt_benchmark
benchmark_cpu=2
output_dir="$benchmark_root/rt-results-20261008"
governor_file="/sys/devices/system/cpu/cpu${benchmark_cpu}/cpufreq/scaling_governor"
saved_governor=$(cat "$governor_file")
restore_governor() {
    printf '%s\n' "$saved_governor" > "$governor_file"
}
trap restore_governor EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
mkdir -p "$output_dir"
printf 'cpu=%s\noriginal_governor=%s\n' "$benchmark_cpu" "$saved_governor" > "$output_dir/environment.txt"
uname -a >> "$output_dir/environment.txt"
for profile in fifo_locked fifo_locked_performance; do
    if [[ $profile == fifo_locked_performance ]]; then
        printf 'performance\n' > "$governor_file"
    fi
    printf '%s governor=%s\n' "$profile" "$(cat "$governor_file")" >> "$output_dir/environment.txt"
    for case_dir in "$benchmark_root"/export/n*_w*; do
        case_name=$(basename "$case_dir")
        [[ -x "$case_dir/replay" ]] || { echo "Missing compiled replay: $case_dir" >&2; exit 2; }
        echo "Testing $profile $case_name"
        # replay sets FIFO 40 and locks/prepares memory only after loading all data.
        # timeout runs outside the pinned FIFO process; each replay is bounded.
        timeout --kill-after=2s 20s taskset -c "$benchmark_cpu" "$case_dir/replay" \
            "$case_dir/trace.bin" "$case_dir/structure.bin" rt \
            > "$output_dir/${profile}_${case_name}.jsonl" \
            2> "$output_dir/${profile}_${case_name}.settings.log"
    done
done
echo "Complete. Results: $output_dir; CPU governor will be restored to $saved_governor."
