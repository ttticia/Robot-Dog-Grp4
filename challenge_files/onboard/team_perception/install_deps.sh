#!/usr/bin/env bash
# Install everything the ONBOARD package (qr_scanner, poster_detector, image_throttle) needs,
# on the robot computer (Ubuntu 22.04, ROS 2 Humble already installed).
#
#     bash install_deps.sh            # apt (asks for sudo) + pip
#     bash install_deps.sh --no-apt   # pip only (apt packages already installed)
#
# Then build from the workspace root:  colcon build --packages-select team_perception
set -euo pipefail
cd "$(dirname "$0")"

APT_PKGS=(
    ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
    ros-humble-launch-ros
    python3-opencv python3-numpy
    python3-pyzbar libzbar0       # QR fallback decoder (Humble's OpenCV has no QUIRC)
    python3-pip
)

if [[ "${1:-}" != "--no-apt" ]]; then
    echo ">> apt: ${APT_PKGS[*]}"
    sudo apt-get update
    sudo apt-get install -y "${APT_PKGS[@]}"
fi

echo ">> pip: requirements.txt"
pip3 install --user -r requirements.txt

echo ">> checking imports"
python3 - <<'EOF'
import cv2, numpy, zxingcpp
from pyzbar import pyzbar
assert int(numpy.__version__.split('.')[0]) < 2, 'numpy must be < 2 for Humble cv_bridge'
print(f'OK: opencv {cv2.__version__}, numpy {numpy.__version__}, zxing-cpp, pyzbar')
EOF
echo ">> done. Build: cd <workspace> && colcon build --packages-select team_perception"
