#!/usr/bin/env bash

set -e

# ------------------------------------------------------------
# PX4 / Gazebo Classic paths
# PX4와 Gazebo Classic 경로 설정
# ------------------------------------------------------------
export PX4_ROOT=/project/firmware/PX4-Autopilot
export PX4_BUILD_DIR=$PX4_ROOT/build/px4_sitl_default
export PX4_GAZEBO_CLASSIC_DIR=$PX4_ROOT/Tools/simulation/gazebo-classic/sitl_gazebo-classic

# ROS2 Humble environment
source /opt/ros/humble/setup.bash

# Gazebo ROS plugin path
export GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH
export LD_LIBRARY_PATH=/opt/ros/humble/lib:$LD_LIBRARY_PATH

# ------------------------------------------------------------
# Gazebo plugin/model/library paths
# Gazebo가 PX4 plugin, model, shared library를 찾도록 경로 설정
# ------------------------------------------------------------
export GAZEBO_PLUGIN_PATH=$PX4_BUILD_DIR/build_gazebo-classic:$GAZEBO_PLUGIN_PATH
export GAZEBO_MODEL_PATH=$PX4_GAZEBO_CLASSIC_DIR/models:$GAZEBO_MODEL_PATH
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:$PX4_BUILD_DIR/build_gazebo-classic:$LD_LIBRARY_PATH

# ------------------------------------------------------------
# Arguments
# SPAWN_MODEL: Gazebo에 실제로 넣을 SDF 모델
# WORLD: 사용할 Gazebo world 이름
# PX4_MODEL: PX4 airframe 선택에 사용할 모델 이름
# ------------------------------------------------------------
SPAWN_MODEL=${1:-iris}
WORLD=${2:-competition}
PX4_MODEL=${3:-$SPAWN_MODEL}

# ------------------------------------------------------------
# PX4 simulation environment
# PX4가 어떤 airframe을 선택할지 결정하는 핵심 변수
# 예: PX4_MODEL=iris_comp_cam
#     → 1100_gazebo-classic_iris_comp_cam airframe을 찾음
# ------------------------------------------------------------
export PX4_SIM_MODEL=gazebo-classic_${PX4_MODEL}
export PX4_SIM_WORLD=${WORLD}
export PX4_SIM_HOSTNAME=localhost

# ------------------------------------------------------------
# File paths
# ------------------------------------------------------------
WORLD_FILE=$PX4_GAZEBO_CLASSIC_DIR/worlds/${WORLD}.world
MODEL_FILE=$PX4_GAZEBO_CLASSIC_DIR/models/${SPAWN_MODEL}/${SPAWN_MODEL}.sdf

PX4_BIN=$PX4_BUILD_DIR/bin/px4
PX4_ETC=$PX4_BUILD_DIR/etc
PX4_ROOTFS=$PX4_BUILD_DIR/rootfs

cleanup() {
    echo ""
    echo "🧹 Cleaning up PX4 / Gazebo processes..."

    pkill -TERM -f "gz model" 2>/dev/null || true
    pkill -TERM -f gzclient 2>/dev/null || true
    pkill -TERM -f gzserver 2>/dev/null || true
    pkill -TERM -f gazebo 2>/dev/null || true
    pkill -TERM -f px4 2>/dev/null || true

    sleep 1
    echo "✅ Cleanup done."
}

trap cleanup EXIT INT TERM

# ------------------------------------------------------------
# Pre-checks
# ------------------------------------------------------------
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ World file not found: $WORLD_FILE"
    exit 1
fi

if [ ! -f "$MODEL_FILE" ]; then
    echo "❌ Model file not found: $MODEL_FILE"
    exit 1
fi

if [ ! -x "$PX4_BIN" ]; then
    echo "❌ PX4 binary not found or not executable: $PX4_BIN"
    exit 1
fi

echo "🧠 Spawn model      : $SPAWN_MODEL"
echo "🌍 Gazebo world     : $WORLD"
echo "🧠 PX4 airframe     : $PX4_MODEL"
echo "📄 Model file       : $MODEL_FILE"
echo "📄 World file       : $WORLD_FILE"

# ------------------------------------------------------------
# Clean old processes before starting
# ------------------------------------------------------------
echo "🧹 Cleaning old processes..."
pkill -TERM -f "gz model" 2>/dev/null || true
pkill -TERM -f gzclient 2>/dev/null || true
pkill -TERM -f gzserver 2>/dev/null || true
pkill -TERM -f gazebo 2>/dev/null || true
pkill -TERM -f px4 2>/dev/null || true
sleep 2

# ------------------------------------------------------------
# Start gzserver
# GUI 없이 Gazebo server만 실행
# ------------------------------------------------------------
echo "🚀 Starting Gazebo server..."
gzserver --verbose "$WORLD_FILE" \
    -s libgazebo_ros_init.so \
    -s libgazebo_ros_factory.so &

GZSERVER_PID=$!

# ------------------------------------------------------------
# Wait for Gazebo master port 11345
# Gazebo master 포트가 열릴 때까지 대기
# ------------------------------------------------------------
echo "⏳ Waiting for Gazebo master port 11345..."

for i in $(seq 1 30); do
    if ss -ltn 2>/dev/null | grep -q ":11345"; then
        echo "✅ Gazebo master is ready."
        break
    fi

    echo "   waiting for 11345... ($i/30)"
    sleep 1
done

# ------------------------------------------------------------
# Wait for world/factory topic
# 11345가 열려도 world 로딩이 끝난 것은 아니므로 factory topic까지 확인
# ------------------------------------------------------------
echo "⏳ Waiting for Gazebo world/factory to be ready..."

for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/gazebo/${WORLD}/factory"; then
        echo "✅ Gazebo world/factory is ready."
        break
    fi

    echo "   waiting for /gazebo/${WORLD}/factory... ($i/30)"
    sleep 1
done

# Gazebo plugin 초기화 여유 시간
sleep 3

# ------------------------------------------------------------
# Spawn model into the named world
# competition.world의 world name이 competition이므로 --world-name 필수
# ------------------------------------------------------------
echo "🚁 Spawning model: $SPAWN_MODEL"

timeout -k 5 60 gz model --verbose \
    --world-name "$WORLD" \
    --spawn-file="$MODEL_FILE" \
    --model-name="$SPAWN_MODEL" \
    -x 1.01 -y 0.98 -z 0.83 || true

# ------------------------------------------------------------
# Wait for simulator TCP port 4560
# Gazebo mavlink_interface가 PX4 연결 포트 4560을 열 때까지 대기
# ------------------------------------------------------------
echo "⏳ Waiting for Gazebo simulator TCP port 4560..."

for i in $(seq 1 60); do
    if ss -ltn 2>/dev/null | grep -q ":4560"; then
        echo "✅ Gazebo simulator TCP port 4560 is ready."
        break
    fi

    echo "   waiting for 4560... ($i/60)"
    sleep 1
done

if ! ss -ltn 2>/dev/null | grep -q ":4560"; then
    echo "❌ Port 4560 did not open. Model/plugin is not ready."
    exit 1
fi

# ------------------------------------------------------------
# Run PX4
# 이 터미널이 pxh> 콘솔이 된다
# ------------------------------------------------------------
echo "🚀 Starting PX4 SITL..."
mkdir -p "$PX4_ROOTFS"
cd "$PX4_ROOTFS"

"$PX4_BIN" "$PX4_ETC"
