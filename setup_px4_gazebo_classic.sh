#!/usr/bin/env bash

export PX4_ROOT=/project/firmware/PX4-Autopilot
export PX4_BUILD_DIR=$PX4_ROOT/build/px4_sitl_default
export PX4_GAZEBO_CLASSIC_DIR=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic

export GAZEBO_PLUGIN_PATH=$PX4_BUILD_DIR/build_gazebo-classic:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$PX4_GAZEBO_CLASSIC_DIR/models:$GAZEBO_MODEL_PATH
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$PX4_BUILD_DIR/build_gazebo-classic:$LD_LIBRARY_PATH
