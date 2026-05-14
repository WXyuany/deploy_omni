#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf 'Usage: %s <pm01|kepler_k2|unitree_g1|all> [build-dir]\n' "$0" >&2
  exit 1
fi

robot="$1"
build_dir="${2:-}"
build_targets=(adapter_bridge)

case "$robot" in
  pm01)
    build_dir="${build_dir:-build-pm01-only}"
    configure_args=(
      -DBUILD_ROBOT_KEPLER_K2=OFF
      -DBUILD_ROBOT_PM01=ON
      -DBUILD_ROBOT_UNITREE_G1=OFF
    )
    ;;
  kepler_k2|k2)
    build_dir="${build_dir:-build-k2-only}"
    configure_args=(
      -DBUILD_ROBOT_KEPLER_K2=ON
      -DBUILD_ROBOT_PM01=OFF
      -DBUILD_ROBOT_UNITREE_G1=OFF
    )
    ;;
  unitree_g1|g1)
    build_dir="${build_dir:-build-g1-only}"
    configure_args=(
      -DBUILD_ROBOT_KEPLER_K2=OFF
      -DBUILD_ROBOT_PM01=OFF
      -DBUILD_ROBOT_UNITREE_G1=ON
    )
    build_targets=(adapter_bridge fake_state_publisher state_receiver)
    ;;
  all)
    build_dir="${build_dir:-build}"
    configure_args=(
      -DBUILD_ROBOT_KEPLER_K2=ON
      -DBUILD_ROBOT_PM01=ON
      -DBUILD_ROBOT_UNITREE_G1=ON
    )
    ;;
  *)
    printf 'Unsupported robot target: %s\n' "$robot" >&2
    printf 'Expected one of: pm01, kepler_k2, unitree_g1, all\n' >&2
    exit 1
    ;;
esac

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

cmake -S "$repo_root" -B "$repo_root/$build_dir" "${configure_args[@]}"
cmake --build "$repo_root/$build_dir" --target "${build_targets[@]:-adapter_bridge}"
