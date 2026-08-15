#!/usr/bin/env bash
set -euo pipefail

if [[ $# -gt 2 ]]; then
    echo "Usage: $0 [number_of_trajectories] [batch_size]"
    exit 1
fi

number_of_trajectories="${1:-1000}"
batch_size="${2:-128}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace="$(dirname "$script_dir")"

if ! [[ "$number_of_trajectories" =~ ^[1-9][0-9]*$ && "$batch_size" =~ ^[1-9][0-9]*$ ]]; then
    echo "error: number_of_trajectories and batch_size must be positive integers" >&2
    exit 1
fi

exec python3 "$workspace/curobo_planning/collect_trajectories.py" \
    --num-trajectories "$number_of_trajectories" \
    --batch-size "$batch_size" \
    --output "$workspace/curobo_planning/datasets/warehouse_trajectories.h5" \
    --overwrite
