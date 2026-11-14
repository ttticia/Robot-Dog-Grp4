#!/usr/bin/env bash
# Install what maze_explorer needs (Ubuntu 22.04, ROS 2 Humble already installed) and put
# explore_lite (m-explore-ros2, source only for Humble) next to this package.
#
#     bash install_deps.sh                 # apt (asks for sudo) + clone explore_lite
#     bash install_deps.sh --no-apt        # only clone explore_lite
#     bash install_deps.sh --no-sim        # skip the Gazebo packages (on the robot)
#
# Then build from the workspace root:
#     colcon build --packages-select explore_lite_msgs explore_lite maze_explorer
set -euo pipefail
cd "$(dirname "$0")"

EXPLORE_REPO=https://github.com/robo-friends/m-explore-ros2.git
EXPLORE_COMMIT=326cf8a0b487c34246bb8f3326afbcd69576dc60   # tested 2026-11-13

APT=1
SIM=1
for arg in "$@"; do
    case "$arg" in
        --no-apt) APT=0 ;;
        --no-sim) SIM=0 ;;
        *) echo "unknown option $arg" >&2; exit 1 ;;
    esac
done

APT_PKGS=(
    ros-humble-navigation2            # controller/planner/behaviors/bt_navigator/smoother
    ros-humble-slam-toolbox
    ros-humble-pointcloud-to-laserscan
    ros-humble-tf2-ros
    python3-numpy python3-opencv
    git
)
if [[ $SIM == 1 ]]; then
    APT_PKGS+=(ros-humble-gazebo-ros-pkgs)
fi

if [[ $APT == 1 ]]; then
    echo ">> apt: ${APT_PKGS[*]}"
    sudo apt-get update
    sudo apt-get install -y "${APT_PKGS[@]}"
fi

# explore_lite goes beside this package (the parent folder must be inside <workspace>/src).
DEST="$(cd .. && pwd)/m-explore-ros2"
if [[ -d "$DEST/.git" ]]; then
    echo ">> explore_lite already at $DEST"
else
    echo ">> cloning explore_lite into $DEST"
    git clone "$EXPLORE_REPO" "$DEST"
fi
git -C "$DEST" checkout -q "$EXPLORE_COMMIT"
# map_merge (multi-robot) isn't needed and pulls in extra dependencies.
touch "$DEST/map_merge/COLCON_IGNORE"

echo ">> checking imports"
python3 - <<'EOF'
import cv2, numpy
assert int(numpy.__version__.split('.')[0]) < 2, 'numpy must be < 2 for Humble'
print(f'OK: opencv {cv2.__version__}, numpy {numpy.__version__}')
EOF
echo ">> done. Build from the workspace root:"
echo "   colcon build --packages-select explore_lite_msgs explore_lite maze_explorer"
