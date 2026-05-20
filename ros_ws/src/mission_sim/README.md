# mission_sim

Gazebo Classic + PX4 SITL assets for the indoor grid competition.

Contents:
- `worlds/competition.world` — 30x22 m floor, 8 vertical + 6 horizontal grid lines (4 m spacing)
- `models/iris_comp/` — iris with a downward RGB camera, lidar and px4flow
- `airframes/1099_gazebo-classic_iris_comp` — PX4 airframe (GPS off, opt flow on)
- `scripts/install_to_px4.sh` — links the world/model/airframe into a PX4 tree

The files here are the source of truth. PX4 sees them through symlinks so
`make distclean` or a clean PX4 reclone is reversible — just run install again.

## Setup

```
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-msgs \
                    ros-humble-gazebo-plugins gazebo libgazebo-dev

cd <px4-autopilot>
git submodule update --init --recursive Tools/simulation/gazebo-classic/sitl_gazebo-classic

bash <ros2_ws>/src/mission_sim/scripts/install_to_px4.sh <px4-autopilot>

make px4_sitl_default gazebo-classic_iris_comp__competition

cd <ros2_ws>
colcon build --symlink-install --packages-select mission_sim
```

## Run

```
# 1) PX4 SITL + Gazebo
cd <px4-autopilot>
make px4_sitl_default gazebo-classic_iris_comp__competition

# 2) Mission nodes (camera topic is published directly by the Classic plugin)
source <ros2_ws>/install/setup.bash
ros2 launch mission_sim sim.launch.py
```

## Frame conventions

Gazebo is ENU, PX4 is NED:
- gz_x = NED y (east)
- gz_y = NED x (north)
- gz_z = -NED z (up)

Grid lives in NED x in [-1, 21], y in [-1, 29]. Vertical lines at y = 0,4,..,28.
Horizontal lines at x = 0,4,..,20.
