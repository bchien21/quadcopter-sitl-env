#!/usr/bin/env bash
set -euo pipefail

conversion_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repository_dir="$(cd "${conversion_dir}/.." && pwd)"
input_map="${1:-${repository_dir}/drone_ws/src/moveit_octomaps/maps/warehouse.bt}"
output_dir="${2:-${conversion_dir}/output}"
map_name="$(basename "${input_map%.*}")"
output_prefix="${output_dir}/${map_name}_grid"

cmake -S "${conversion_dir}" -B "${conversion_dir}/build"
cmake --build "${conversion_dir}/build" --parallel
mkdir -p "${output_dir}"

"${conversion_dir}/build/octomap_to_grid" \
  "${input_map}" "${output_prefix}" \
  -15.0 15.0 -25.0 25.0 0.0 12.6 world

python3 "${conversion_dir}/grid_to_esdf.py" \
  "${output_prefix}.bin" \
  "${output_prefix}.json" \
  "${output_dir}/${map_name}_esdf.npz"

