#!/usr/bin/env bash
#
# Removes the symlinks created by install_to_px4.sh.
# CMake list entries are not reverted automatically; use `git checkout` in
# the PX4 tree if you need them gone.
#

set -euo pipefail

PX4_ROOT="${1:-${HOME}/26_robot/firmware/PX4-Autopilot}"

unlink_symlink() {
  if [ -L "$1" ]; then rm "$1"; echo "removed $1"; fi
}

unlink_symlink "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/competition.world"
unlink_symlink "$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_comp"
unlink_symlink "$PX4_ROOT/ROMFS/px4fmu_common/init.d-posix/airframes/1099_gazebo-classic_iris_comp"
