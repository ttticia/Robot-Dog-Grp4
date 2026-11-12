#!/usr/bin/env bash
# One entry point for everything detection-related. Sources ROS 2 Humble and the
# workspace by itself, so no `source ...` lines are needed in the terminal.
#
#   ./perception.sh help                 list all commands
#   ./perception.sh setup                pip installs + build + install check (first time)
#   ./perception.sh demo test/testing_data/QR.mp4     bridge + nodes + video in one go (Ctrl+C stops all)
#
# Tip: add an alias so it works from anywhere:
#   echo "alias perception='$HOME/robot_challenge_workspace/src/team_perception/scripts/perception.sh'" >> ~/.bashrc

# --- locations -------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"                 # src/team_perception
WS="${WS:-$(cd "$PKG_DIR/../.." && pwd)}"               # ~/robot_challenge_workspace
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"

# --- camera topics (override with environment variables) -------------------------------
# Defaults match the robot's Orbbec camera as recorded in the test_run bag. For the
# laptop webcam / video bridge use `--webcam` (or the demo/play commands), which switch
# to compressed transport and no depth.
IMAGE_TOPIC="${IMAGE_TOPIC:-/camera/color/image_raw}"
IMAGE_TRANSPORT="${IMAGE_TRANSPORT:-compressed}"
DEPTH_TOPIC="${DEPTH_TOPIC:-/camera/depth/image_raw}"    # must be ALIGNED to colour for poster
CAMERA_INFO_TOPIC="${CAMERA_INFO_TOPIC:-/camera/color/camera_info}"
MAP_TOPIC="${MAP_TOPIC:-/map}"

TORCH_VERSION="2.14.1"
TORCHVISION_VERSION="0.29.2"
APT_PACKAGES="python3-pip python3-colcon-common-extensions python3-pyzbar python3-qrcode \
ros-humble-cv-bridge ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
ros-humble-image-transport-plugins ros-humble-rqt-image-view"

# --- helpers ---------------------------------------------------------------------------
say()  { printf '\033[1;36m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mwarning: %s\033[0m\n' "$*" >&2; }
die()  { printf '\033[1;31merror: %s\033[0m\n' "$*" >&2; exit 1; }

source_ros() {
    [ -f "$ROS_SETUP" ] || die "ROS 2 Humble not found at $ROS_SETUP (set ROS_SETUP=...)"
    # ROS setup scripts reference unset variables, so never run them under `set -u`.
    # shellcheck disable=SC1090
    source "$ROS_SETUP"
}

source_ws() {
    source_ros
    if [ ! -f "$WS/install/setup.bash" ]; then
        warn "workspace not built yet; building now"
        cmd_build
    fi
    # shellcheck disable=SC1091
    source "$WS/install/setup.bash"
}

# Background processes started by this script, stopped together on exit / Ctrl+C.
PIDS=()
# Each one runs in its own process group (setsid), because `ros2 run` is only a wrapper:
# killing its pid leaves the real node running. Stopping the whole group gets both.
start_bg() {   # start_bg <label> <command...>
    local label="$1"; shift
    setsid "$@" &
    PIDS+=("$!")
    say "started $label (pid $!)"
}
stop_all() {
    trap - INT TERM EXIT
    [ "${#PIDS[@]}" -gt 0 ] || return 0
    say "stopping ${#PIDS[@]} process group(s)"
    local p
    for p in "${PIDS[@]}"; do kill -INT -- "-$p" 2>/dev/null; done   # polite: lets nodes log
    for _ in 1 2 3 4 5 6; do                                          # up to 3 s to exit
        local alive=0
        for p in "${PIDS[@]}"; do kill -0 -- "-$p" 2>/dev/null && alive=1; done
        [ $alive -eq 0 ] && break
        sleep 0.5
    done
    for p in "${PIDS[@]}"; do kill -KILL -- "-$p" 2>/dev/null; done  # anything still there
    wait 2>/dev/null
    PIDS=()
}
export PYTHONUNBUFFERED=1   # node/echo output appears immediately, also when piped to a file

robot_args() {   # common camera arguments for the detector nodes
    echo "-p image_topic:=$IMAGE_TOPIC -p image_transport:=$IMAGE_TRANSPORT"
}

# --- setup / build / check -------------------------------------------------------------
cmd_setup() {
    say "1/4 system packages (needs sudo; skipped if already installed)"
    local missing=""
    for p in $APT_PACKAGES; do
        dpkg -s "$p" >/dev/null 2>&1 || missing="$missing $p"
    done
    if [ -n "$missing" ]; then
        warn "missing apt packages:$missing"
        echo "    run:  sudo apt update && sudo apt install$missing"
    else
        echo "    all apt packages present"
    fi

    say "2/4 pip packages (user install)"
    if python3 -c "import torch" 2>/dev/null; then
        echo "    torch already installed: $(python3 -c 'import torch; print(torch.__version__)')"
    else
        # CPU build first, so ultralytics doesn't pull the multi-GB CUDA build
        pip3 install --user "torch==$TORCH_VERSION" "torchvision==$TORCHVISION_VERSION" \
            --index-url https://download.pytorch.org/whl/cpu || die "torch install failed"
    fi
    pip3 install --user -r "$PKG_DIR/requirements.txt" || die "pip install failed"
    # torch drags in a setuptools that breaks colcon on Humble; the apt one is fine
    if pip3 show setuptools 2>/dev/null | grep -q "Location: $HOME/.local"; then
        pip3 uninstall -y setuptools
    fi

    say "3/4 build"
    cmd_build
    say "4/4 check"
    cmd_check
}

cmd_build() {
    source_ros
    if [ "${1:-}" = "--clean" ]; then
        say "clean build: removing build/ and install/ for team_perception"
        rm -rf "$WS/build/team_perception" "$WS/install/team_perception"
    fi
    # colcon must run from the workspace root, never from src/
    (cd "$WS" && colcon build --symlink-install --packages-select team_perception) \
        || die "build failed (if the error mentions setuptools: pip3 uninstall -y setuptools)"
}

cmd_check() {
    source_ws
    python3 - <<'EOF'
import importlib
ok = True
def show(name, fn):
    global ok
    try:
        print(f'  {name:14s} {fn()}')
    except Exception as e:
        ok = False
        print(f'  {name:14s} MISSING ({e.__class__.__name__}: {e})')
show('QR decoders', lambda: __import__('team_perception.qr_utils', fromlist=['x']).BACKENDS)
show('ultralytics', lambda: importlib.import_module('ultralytics').__version__)
show('torch', lambda: importlib.import_module('torch').__version__)
show('numpy', lambda: importlib.import_module('numpy').__version__ + ('' if importlib.import_module('numpy').__version__.startswith('1.') else '  <-- must be < 2'))
show('opencv', lambda: importlib.import_module('cv2').__version__)
show('imageio-ffmpeg', lambda: importlib.import_module('imageio_ffmpeg').get_ffmpeg_version())
show('setuptools', lambda: importlib.import_module('setuptools').__version__ + '  (apt 59.x expected)')
import sys; sys.exit(0 if ok else 1)
EOF
    local rc=$?
    ros2 pkg executables team_perception | sed 's/^/  /'
    [ $rc -eq 0 ] && say "install looks good" || warn "something is missing: run ./perception.sh setup"
    return $rc
}

cmd_test() {
    source_ws
    (cd "$PKG_DIR" && python3 -m pytest test/ -q "$@")
}

# --- nodes -----------------------------------------------------------------------------
# Every node command takes extra ROS arguments, e.g.:  ./perception.sh qr -p use_depth:=false
cmd_bridge() { source_ws; exec ros2 run team_perception webcam_bridge --ros-args "$@"; }

cmd_qr() {
    source_ws
    # shellcheck disable=SC2046
    exec ros2 run team_perception qr_scanner --ros-args $(robot_args) \
        -p depth_topic:="$DEPTH_TOPIC" -p camera_info_topic:="$CAMERA_INFO_TOPIC" "$@"
}

cmd_human() {
    source_ws
    # shellcheck disable=SC2046
    exec ros2 run team_perception human_detector --ros-args $(robot_args) \
        -p depth_topic:="$DEPTH_TOPIC" -p camera_info_topic:="$CAMERA_INFO_TOPIC" "$@"
}

cmd_poster() {
    source_ws
    warn "poster_detector needs depth ALIGNED to colour ($DEPTH_TOPIC), localisation (TF to map) and $MAP_TOPIC"
    # shellcheck disable=SC2046
    exec ros2 run team_perception poster_detector --ros-args $(robot_args) \
        -p depth_topic:="$DEPTH_TOPIC" -p camera_info_topic:="$CAMERA_INFO_TOPIC" \
        -p map_topic:="$MAP_TOPIC" "$@"
}

cmd_robot() {   # robot [qr|human|poster|all] [--echo]: run detector nodes together on robot topics
    local which="all" echo_found=""
    for a in "$@"; do
        case "$a" in
            qr|human|poster|all) which="$a" ;;
            --echo) echo_found=1 ;;
            *) die "robot: expected qr|human|poster|all [--echo]" ;;
        esac
    done
    source_ws
    trap 'stop_all; exit 130' INT TERM; trap stop_all EXIT
    case "$which" in
        qr)     start_bg qr_scanner "$0" qr ;;
        human)  start_bg human_detector "$0" human ;;
        poster) start_bg human_detector "$0" human; start_bg poster_detector "$0" poster ;;
        all)    start_bg qr_scanner "$0" qr; start_bg human_detector "$0" human
                start_bg poster_detector "$0" poster ;;
    esac
    # debugging aid, only on request (the nodes log "QR found"/"Human found" themselves)
    [ -n "$echo_found" ] && start_bg "/found listener" ros2 topic echo /found std_msgs/msg/String --field data
    say "running; Ctrl+C stops everything"
    wait
}

# --- video / webcam input ----------------------------------------------------------------
cmd_play() {   # play <video|frame folder> [sender options, e.g. --loop --speed 0.5]
    [ -n "${1:-}" ] || die "play: give a video file or frame folder"
    local src="$1"; shift
    [ -e "$src" ] || die "no such file: $src"
    source_ws
    exec python3 "$SCRIPT_DIR/webcam_sender_windows.py" --video "$src" "$@"
}

wait_for_publisher() {   # wait_for_publisher <topic> <label> [timeout_s]: node is ready
    local topic="$1" label="$2" limit="${3:-40}" t=0
    until ros2 topic info "$topic" 2>/dev/null | grep -q "Publisher count: [1-9]"; do
        sleep 1; t=$((t + 1))
        [ $t -ge "$limit" ] && { warn "$label not ready after ${limit}s; continuing"; return 1; }
    done
    say "$label ready (${t}s)"
}

cmd_demo() {   # demo <video> [qr|human|both] [--loop] [--view] [--echo]: live pipeline from a video
    [ -n "${1:-}" ] || die "demo: give a video file, e.g. ./perception.sh demo test/testing_data/QR.mp4"
    local src; src="$(readlink -f "$1")"; shift
    [ -e "$src" ] || die "no such file: $src"
    local which="both" loop="" view="" echo_found=""
    for a in "$@"; do
        case "$a" in
            qr|human|both) which="$a" ;;
            --loop) loop="--loop" ;;
            --view) view=1 ;;           # open rqt debug-image windows
            --echo) echo_found=1 ;;     # print /found in its own listener
            *) die "demo: unknown option $a (expected qr|human|both --loop --view --echo)" ;;
        esac
    done
    source_ws
    trap 'stop_all; exit 130' INT TERM; trap stop_all EXIT
    local cam=(-p image_transport:=compressed -p use_depth:=false)
    start_bg webcam_bridge ros2 run team_perception webcam_bridge
    [ "$which" != human ] && start_bg qr_scanner ros2 run team_perception qr_scanner --ros-args "${cam[@]}"
    [ "$which" != qr ] && start_bg human_detector ros2 run team_perception human_detector --ros-args "${cam[@]}"
    # debugging tools: only when asked for
    if [ -n "$view" ]; then
        [ "$which" != human ] && start_bg "rqt (QR)" ros2 run rqt_image_view rqt_image_view /qr/debug_image/compressed
        [ "$which" != qr ] && start_bg "rqt (people)" ros2 run rqt_image_view rqt_image_view /humans/debug_image/compressed
    fi
    [ -n "$echo_found" ] && start_bg "/found listener" ros2 topic echo /found std_msgs/msg/String --field data
    # Start the video only once the nodes are listening (YOLO needs ~7 s to load and warm up),
    # otherwise the first seconds of the clip are never processed.
    [ "$which" != human ] && wait_for_publisher /qr/detections qr_scanner
    [ "$which" != qr ] && wait_for_publisher /humans/detections human_detector
    say "playing $src (Ctrl+C stops everything)"
    start_bg "video sender" python3 "$SCRIPT_DIR/webcam_sender_windows.py" --video "$src" $loop
    wait "${PIDS[-1]}"          # until the video ends (or Ctrl+C runs stop_all)
    say "video finished"
    sleep 2                     # let the last detections come through
}

# --- viewing / listening -------------------------------------------------------------------
cmd_view() {   # view <qr|human|poster> [raw]
    local which="${1:-qr}" suffix="/compressed"
    [ "${2:-}" = raw ] && suffix=""
    case "$which" in
        qr) topic=/qr/debug_image ;; human) topic=/humans/debug_image ;;
        poster) topic=/posters/debug_image ;; *) die "view: expected qr|human|poster" ;;
    esac
    source_ws
    if [ -n "$suffix" ] && ! ros2 run image_transport list_transports 2>/dev/null | grep -q compressed; then
        warn "compressed transport plugin missing (sudo apt install ros-humble-image-transport-plugins); showing raw topic"
        suffix=""
    fi
    exec ros2 run rqt_image_view rqt_image_view "$topic$suffix"
}

cmd_echo() {   # echo <found|qr|human|poster|classifications>
    local which="${1:-found}" topic
    case "$which" in
        found) topic=/found ;; qr) topic=/qr/detections ;; human) topic=/humans/detections ;;
        poster) topic=/posters/detections ;; classifications) topic=/posters/classifications ;;
        *) die "echo: expected found|qr|human|poster|classifications" ;;
    esac
    source_ws
    exec ros2 topic echo "$topic" std_msgs/msg/String --field data
}

# --- offline tools ---------------------------------------------------------------------------
# Runs in the caller's folder so relative input paths work; results still go to
# src/team_perception/test_runs/ (annotate finds it from its own install location).
cmd_annotate() { source_ws; exec ros2 run team_perception annotate "$@"; }

cmd_bag_check() {   # bag-check <bag folder>: is a recorded bag readable and what is in it?
    [ -n "${1:-}" ] || die "bag-check: give a bag folder"
    source_ros
    ros2 bag info "$1" || return 1
    for db in "$1"/*.db3; do
        [ -e "$db" ] || continue
        python3 - "$db" <<'EOF'
import sqlite3, sys
try:
    c = sqlite3.connect(f'file:{sys.argv[1]}?mode=ro', uri=True)
    print(f'{sys.argv[1]}: integrity {c.execute("pragma quick_check").fetchone()[0]}, '
          f'{c.execute("select count(*) from messages").fetchone()[0]} messages')
except Exception as e:
    print(f'{sys.argv[1]}: UNREADABLE ({e}) - still copying, or a broken copy/recording')
    sys.exit(1)
EOF
    done
}

cmd_help() {
    cat <<EOF
Usage: $(basename "$0") <command> [args]      (sources ROS + the workspace for you)

Setup
  setup                     pip installs (CPU torch, requirements), build, check; prints any apt command needed
  build [--clean]           colcon build from the workspace root (--clean: rebuild team_perception from scratch)
  check                     show decoder/YOLO/numpy/... versions and the package's executables
  test [pytest args]        run the unit + style tests

Live nodes (robot camera topics; override with IMAGE_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC, MAP_TOPIC)
  qr      [ros args]        QR scanner                    e.g. qr -p use_depth:=false
  human   [ros args]        YOLO person detector
  poster  [ros args]        poster-only detector (needs human, aligned depth, TF + /map)
  robot   [qr|human|poster|all] [--echo]   start several nodes together; Ctrl+C stops all
                            --echo also prints /found

Video / laptop webcam
  bridge                    receive the webcam/video stream into ROS
  play <video> [--loop] [--speed X]   send a video (or frame folder) to the bridge
  demo <video> [qr|human|both] [--loop] [--view] [--echo]   bridge + nodes + video, all in one
                            --view opens rqt debug windows, --echo prints /found (both off by default)

Viewing
  view <qr|human|poster> [raw]    rqt_image_view on the debug image
  echo <found|qr|human|poster|classifications>   print a topic's messages

Offline
  annotate -t <qr|human|both> <inputs...> [options]   annotated results into test_runs/<type>/
  bag-check <bag folder>    is a rosbag readable, and what topics does it hold?

Current topics: IMAGE_TOPIC=$IMAGE_TOPIC ($IMAGE_TRANSPORT)  DEPTH_TOPIC=$DEPTH_TOPIC
                CAMERA_INFO_TOPIC=$CAMERA_INFO_TOPIC  MAP_TOPIC=$MAP_TOPIC
Workspace: $WS
EOF
}

# --- dispatch --------------------------------------------------------------------------------
cmd="${1:-help}"
[ $# -gt 0 ] && shift
case "$cmd" in
    setup) cmd_setup "$@" ;;          build) cmd_build "$@" ;;
    check) cmd_check "$@" ;;          test) cmd_test "$@" ;;
    qr) cmd_qr "$@" ;;                human) cmd_human "$@" ;;
    poster) cmd_poster "$@" ;;        robot) cmd_robot "$@" ;;
    bridge) cmd_bridge "$@" ;;        play) cmd_play "$@" ;;
    demo) cmd_demo "$@" ;;            view) cmd_view "$@" ;;
    echo) cmd_echo "$@" ;;            annotate) cmd_annotate "$@" ;;
    bag-check) cmd_bag_check "$@" ;;
    help|-h|--help) cmd_help ;;
    *) cmd_help; die "unknown command: $cmd" ;;
esac
