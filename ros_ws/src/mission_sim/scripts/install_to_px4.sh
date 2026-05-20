#!/usr/bin/env bash
#
# Links mission_sim assets into a PX4-Autopilot tree.
# Usage: install_to_px4.sh [PX4_ROOT]
#

set -euo pipefail

PX4_ROOT="${1:-${HOME}/26_robot/firmware/PX4-Autopilot}"
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"

if [ ! -d "$PX4_ROOT" ]; then
  echo "PX4_ROOT not found: $PX4_ROOT" >&2
  exit 1
fi

WORLDS="$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
MODELS="$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
AIRFRAMES="$PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKE_TARGETS="$PX4_ROOT/src/modules/simulation/simulator_mavlink/sitl_targets_gazebo-classic.cmake"
CMAKE_AIRFRAMES="$AIRFRAMES/CMakeLists.txt"

if [ ! -d "$WORLDS" ] || [ ! -d "$MODELS" ] || [ ! -d "$AIRFRAMES" ]; then
  echo "PX4 gazebo-classic submodule not initialised. Run:" >&2
  echo "  (cd $PX4_ROOT && git submodule update --init --recursive Tools/simulation/gazebo-classic/sitl_gazebo-classic)" >&2
  exit 1
fi

link() {
  local src="$1" dst="$2"
  if [ -L "$dst" ] && [ "$(readlink -f "$dst")" = "$(readlink -f "$src")" ]; then
    return
  fi
  if [ -e "$dst" ] && [ ! -L "$dst" ]; then
    mv "$dst" "$dst.bak"
  elif [ -L "$dst" ]; then
    rm -f "$dst"
  fi
  ln -s "$src" "$dst"
  echo "linked $dst"
}

link "$SIM_DIR/worlds/competition.world" "$WORLDS/competition.world"
link "$SIM_DIR/models/iris_comp" "$MODELS/iris_comp"
link "$SIM_DIR/airframes/1099_gazebo-classic_iris_comp" "$AIRFRAMES/1099_gazebo-classic_iris_comp"

if ! grep -q "1099_gazebo-classic_iris_comp" "$CMAKE_AIRFRAMES"; then
  python3 -c "
import re
p = '$CMAKE_AIRFRAMES'
s = open(p).read()
s = re.sub(r'(1019_gazebo-classic_iris_dual_gps\n)',
          r'\g<1>\t1099_gazebo-classic_iris_comp\n', s, count=1)
open(p, 'w').write(s)
"
  echo "patched $CMAKE_AIRFRAMES"
fi

if ! grep -q '^\s*iris_comp\s*$' "$CMAKE_TARGETS"; then
  python3 -c "
import re
p = '$CMAKE_TARGETS'
s = open(p).read()
s = re.sub(r'(\t\tiris_opt_flow_mockup\n)', r'\g<1>\t\tiris_comp\n', s, count=1)
open(p, 'w').write(s)
"
  echo "patched $CMAKE_TARGETS (model)"
fi

if ! grep -q '^\s*competition\s*$' "$CMAKE_TARGETS"; then
  python3 -c "
import re
p = '$CMAKE_TARGETS'
s = open(p).read()
s = re.sub(r'(\t\tyosemite\n)', r'\g<1>\t\tcompetition\n', s, count=1)
open(p, 'w').write(s)
"
  echo "patched $CMAKE_TARGETS (world)"
fi

echo "done. build: make px4_sitl_default gazebo-classic_iris_comp__competition"
