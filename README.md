# Robot-Dog-Grp4

This repo holds our ROS 2 code for the robot-dog challenge. So far it has one package:

| Package | What it does |
|---|---|
| [`team_perception`](team_perception/) | QR code scanner. It reads codes from the robot's camera (or a laptop webcam or a video) and publishes the text and the code's map position. |

The full guide for the scanner is **[team_perception/QR_test_guide.md](team_perception/QR_test_guide.md)**. It covers how the scanner works, its parameters, video testing and troubleshooting. This README covers getting set up and running.

---

## 1. What you need to install first

**Operating system:**
- Ubuntu 22.04 with **ROS 2 Humble**. It can be native Linux, or WSL2 on Windows 11.
- On WSL2, install Humble inside Ubuntu by following the [official instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) (`ros-humble-desktop`).

**System packages:**
```bash
sudo apt update
sudo apt install python3-pip python3-colcon-common-extensions \
                 python3-pyzbar python3-qrcode \
                 ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
                 ros-humble-image-transport-plugins ros-humble-rqt-image-view
```

**Python package.** This is the main QR decoder. It isn't in apt, so it comes from pip:
```bash
pip3 install --user -r ~/robot_challenge_workspace/src/team_perception/requirements.txt   # zxing-cpp==2.3.0
```
Run this after cloning, in step 2.

Why both decoders? The OpenCV that ships with Humble (4.5.4) **cannot decode QR codes**. We use:
- **zxing-cpp** as the main decoder: it's fast and reads angled codes
- **pyzbar** as a backup

**To test with a laptop webcam on Windows**, also install this in Windows PowerShell (not WSL):
```powershell
pip install opencv-python numpy
```

## 2. Get the code and build

This repo **is** the workspace's `src/` folder, so clone it as `src`:
```bash
mkdir -p ~/robot_challenge_workspace && cd ~/robot_challenge_workspace
git clone git@github.com:ttticia/Robot-Dog-Grp4.git src
pip3 install --user -r src/team_perception/requirements.txt

source /opt/ros/humble/setup.bash
colcon build --symlink-install          # always run colcon from ~/robot_challenge_workspace, never from src/
source install/setup.bash
```
If you'd rather not run the `source` lines in every new terminal, add them to `~/.bashrc`:
```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/robot_challenge_workspace/install/setup.bash' >> ~/.bashrc
```

**Check the install:**
```bash
python3 -c "from team_perception import qr_utils; print(qr_utils.BACKENDS)"   # should print: zxing-cpp + pyzbar
cd ~/robot_challenge_workspace/src/team_perception && python3 -m pytest test/test_qr_utils.py -q
```
The pytest run should end with no failures and **1 xfailed**. That one is an expected failure: a photo of a curved printout that no decoder can read.

## 3. Using the scanner

### On the robot
1. Find the camera topic names with `ros2 topic list | grep -i -e image -e camera_info -e depth`.
2. Start the scanner with those names:
```bash
ros2 run team_perception qr_scanner --ros-args \
  -p image_topic:=/camera/color/image_raw -p image_transport:=compressed \
  -p depth_topic:=/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info
```

### With a laptop webcam (WSL)
WSL can't see a built-in laptop camera directly. A script on Windows streams the camera into ROS instead. Each step below runs in its own terminal:
```bash
# 1. WSL: receive the stream
ros2 run team_perception webcam_bridge
```
```powershell
# 2. Windows PowerShell: send the camera (copy the script from team_perception/scripts/ to e.g. C:\qr\ first)
python C:\qr\webcam_sender_windows.py
```
```bash
# 3. WSL: run the scanner (no depth camera on a laptop)
ros2 run team_perception qr_scanner --ros-args -p image_transport:=compressed -p use_depth:=false
```

### With a video file
This uses the same bridge and scanner as the webcam setup. In place of the Windows sender, play the video from WSL:
```bash
python3 ~/robot_challenge_workspace/src/team_perception/scripts/webcam_sender_windows.py \
    --video /mnt/c/Users/<you>/Videos/qr_test.mp4 --loop
```
To get detection rates for a clip without ROS, run `python3 team_perception/scripts/eval_qr_video.py <video or frame folder>`.

### Seeing the results
```bash
ros2 run rqt_image_view rqt_image_view /qr/debug_image/compressed   # live video with boxes + decoded text
ros2 topic echo /qr/detections std_msgs/msg/String                  # one JSON message per new code
```
Each code is also logged to `~/qr_logs/`, as a CSV row plus an annotated snapshot.

For what each parameter does, the JSON format and common problems, see the [QR test guide](team_perception/QR_test_guide.md).

## 4. Working on the repo

```bash
cd ~/robot_challenge_workspace/src
git pull --rebase origin main      # get everyone else's changes first
# ... make changes, rebuild, run the tests ...
git add -A && git status           # check what you're committing
git commit -m "what you changed"
git push origin main
```
- **After pulling:** rebuild with `colcon build` from `~/robot_challenge_workspace`. If `requirements.txt` changed, re-run the pip install too.
- **Code style:** keep the code passing the flake8 and pep257 style checks, which `colcon test --packages-select team_perception` runs.
- **Never commit** `build/`, `install/`, `log/` or `__pycache__/`. `.gitignore` already excludes them.
