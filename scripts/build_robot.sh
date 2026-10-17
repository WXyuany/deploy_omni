#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  printf 'Usage: %s <pm01|kepler_k2|unitree_g1|cvte_carm|all> [build-dir]\n' "$0" >&2
  exit 1
fi

robot="$1"
build_dir="${2:-}"
build_targets=(adapter_bridge)
sample_config="config/kepler_k2.sample.yaml"

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

case "$robot" in
  pm01)
    build_dir="${build_dir:-build-pm01-only}"
    configure_args=(
      -DBRIDGE_ROBOT=pm01
      -DBUILD_GAMEPAD_SDL=OFF
    )
    sample_config="config/pm01.sample.yaml"
    ;;
  kepler_k2|k2)
    build_dir="${build_dir:-build-k2-only}"
    configure_args=(
      -DBRIDGE_ROBOT=kepler_k2
      -DBUILD_GAMEPAD_SDL=OFF
    )
    sample_config="config/kepler_k2.sample.yaml"
    ;;
  unitree_g1|g1)
    build_dir="${build_dir:-build-g1-only}"
    configure_args=(
      -DBRIDGE_ROBOT=unitree_g1
      -DBUILD_GAMEPAD_SDL=OFF
    )
    configure_args+=(-DBUILD_UNITREE_G1_MOTION_DEMO=ON)
    build_targets=(adapter_bridge unitree_g1_motion_demo unitree_g1_sensor_reader lowstate_frequency_receiver lowstate_field_recorder)
    sample_config="config/g1.sample.yaml"
    ;;
  cvte_carm|carm)
    build_dir="${build_dir:-build-cvte-only}"
    configure_args=(
      -DBRIDGE_ROBOT=cvte_carm
      -DBUILD_BRIDGE_DDS=OFF
      -DBUILD_TESTING=OFF
      -DBUILD_GAMEPAD_SDL=OFF
      "-DBRIDGE_CVTE_SDK_DIR=${BRIDGE_CVTE_SDK_DIR:-$repo_root/arm_control_sdk}"
    )
    build_targets=(cvte_carm_check)
    sample_config="config/cvte_carm_a3.sample.yaml"
    ;;
  all)
    build_dir="${build_dir:-build}"
    configure_args=(
      -DBRIDGE_ROBOT=all
      -DBUILD_GAMEPAD_SDL=OFF
    )
    sample_config="config/kepler_k2.sample.yaml"
    ;;
  *)
    printf 'Unsupported robot target: %s\n' "$robot" >&2
    printf 'Expected one of: pm01, kepler_k2, unitree_g1, cvte_carm, all\n' >&2
    exit 1
    ;;
esac

if [[ "$build_dir" = /* ]]; then
  build_path="$build_dir"
else
  build_path="$repo_root/$build_dir"
fi

cmake -S "$repo_root" -B "$build_path" "${configure_args[@]}"
cmake --build "$build_path" --target "${build_targets[@]}" --parallel "${BUILD_JOBS:-2}"

printf '\nBuilt %s adapter in %s. Start with:\n' "$robot" "$build_dir"
if [[ "$robot" == "cvte_carm" || "$robot" == "carm" ]]; then
  printf '  %s/cvte_carm_check --model a3 --ip <A3控制器IP>\n' "$build_path"
elif [[ "$robot" == "unitree_g1" || "$robot" == "g1" ]]; then
  printf '  %s/unitree_g1_motion_demo --check --interface <network-interface>\n' "$build_path"
else
  printf '  %s/adapter_bridge %s\n' "$build_path" "$repo_root/$sample_config"
fi
