#!/usr/bin/env bash
# Install everything the EXTERNAL package (human_detector, YOLO) needs, on the external
# computer (Ubuntu 22.04, ROS 2 Humble already installed).
#
#     bash install_deps.sh            # apt (asks for sudo) + CPU torch + pip + YOLO weights
#     bash install_deps.sh --gpu      # CUDA torch instead (NVIDIA GPU; then launch device:=0)
#     bash install_deps.sh --no-apt   # skip apt (packages already installed)
#
# Then build from the workspace root:  colcon build --packages-select team_perception_external
set -euo pipefail
cd "$(dirname "$0")"

GPU=0
APT=1
for arg in "$@"; do
    case "$arg" in
        --gpu) GPU=1 ;;
        --no-apt) APT=0 ;;
        *) echo "unknown option: $arg" >&2; exit 1 ;;
    esac
done

APT_PKGS=(
    ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
    ros-humble-launch-ros
    python3-opencv python3-numpy python3-pip
)
TORCH=(torch==2.14.1 torchvision==0.29.2)

if [[ $APT == 1 ]]; then
    echo ">> apt: ${APT_PKGS[*]}"
    sudo apt-get update
    sudo apt-get install -y "${APT_PKGS[@]}"
fi

if [[ $GPU == 1 ]]; then
    echo ">> pip: ${TORCH[*]} (CUDA build)"
    pip3 install --user "${TORCH[@]}"
else
    echo ">> pip: ${TORCH[*]} (CPU build, ~200 MB)"
    pip3 install --user "${TORCH[@]}" --index-url https://download.pytorch.org/whl/cpu
fi

echo ">> pip: requirements.txt"
pip3 install --user -r requirements.txt

# torch pulls in a pip setuptools that breaks colcon on Humble (_distribution_fullname);
# removing it restores the apt version.
echo ">> removing pip setuptools (keeps colcon working)"
pip3 uninstall -y setuptools || true

echo ">> YOLO weights -> ~/.cache/team_perception/yolo11n.pt"
python3 - <<'EOF'
import os
import numpy
assert int(numpy.__version__.split('.')[0]) < 2, 'numpy must be < 2 for Humble cv_bridge'
d = os.path.expanduser('~/.cache/team_perception')
os.makedirs(d, exist_ok=True)
os.chdir(d)                      # ultralytics downloads bare names into the current folder
import torch
from ultralytics import YOLO
YOLO('yolo11n.pt')
print(f'OK: torch {torch.__version__}, CUDA available: {torch.cuda.is_available()}, '
      f'weights in {d}')
EOF
echo ">> done. Build: cd <workspace> && colcon build --packages-select team_perception_external"
