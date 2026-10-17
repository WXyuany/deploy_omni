#!/usr/bin/env bash
set -euo pipefail

run_tests=false
build_dir="build-g1-only"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --test)
      run_tests=true
      shift
      ;;
    --build-dir)
      if [[ $# -lt 2 ]]; then
        printf 'Missing value for --build-dir\n' >&2
        exit 1
      fi
      build_dir="$2"
      shift 2
      ;;
    --help|-h)
      printf 'Usage: %s [--test] [--build-dir DIR]\n' "$0"
      printf 'Builds the G1 gamepad DDS demo and demo support test.\n'
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      printf 'Usage: %s [--test] [--build-dir DIR]\n' "$0" >&2
      exit 1
      ;;
  esac
done

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/.." && pwd)"

cmake -S "$repo_root" -B "$repo_root/$build_dir" \
  -DBRIDGE_ROBOT=unitree_g1 \
  -DBUILD_GAMEPAD_SDL=ON

cmake --build "$repo_root/$build_dir" --target \
  gamepad_dds_sender \
  gamepad_dds_receiver \
  unitree_g1_gamepad_test

if [[ "$run_tests" == true ]]; then
  ctest --test-dir "$repo_root/$build_dir" \
    -R '^unitree_g1_gamepad_test$' \
    --output-on-failure
fi

printf '\nBuilt gamepad DDS demo in %s. Run with:\n' "$build_dir"
printf '  %s/%s/gamepad_dds_receiver config/g1.sample.yaml\n' "$repo_root" "$build_dir"
printf '  %s/%s/gamepad_dds_sender config/g1.sample.yaml 5 0\n' "$repo_root" "$build_dir"
