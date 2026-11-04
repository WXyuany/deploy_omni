#!/bin/bash
SCRIPT_DIR="$(cd $(dirname "${BASH_SOURCE[0]}") && pwd)"
LIB1="$SCRIPT_DIR/lib"
case ":$LD_LIBRARY_PATH:" in
  *":$LIB1:"*) ;;
  *) export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$LIB1" ;;
esac
export arm_control_sdk_DIR=$SCRIPT_DIR
echo "find_package DIR: $arm_control_sdk_DIR"
echo "[setup.bash] done!"

