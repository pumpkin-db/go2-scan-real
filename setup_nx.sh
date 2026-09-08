#!/usr/bin/env bash
# Prepare an Ubuntu 20.04 ARM64 NX from the sources and wheels in this tree.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKIP_APT=false
NON_INTERACTIVE=false
for arg in "$@"; do
  case "$arg" in
    --skip-apt) SKIP_APT=true ;;
    --non-interactive) NON_INTERACTIVE=true ;;
    *) echo "Unknown argument: $arg" >&2; exit 2 ;;
  esac
done

[ "$(uname -m)" = aarch64 ] || { echo '[FATAL] this release targets an ARM64 NX' >&2; exit 42; }
[ -r /opt/ros/noetic/setup.bash ] || { echo '[FATAL] ROS Noetic is required' >&2; exit 42; }

if [ "$SKIP_APT" = false ]; then
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake python3-dev python3-pip python3-venv python3-paramiko \
    libboost-all-dev libeigen3-dev libpcl-dev libopencv-dev libarmadillo-dev \
    libnlopt-dev libyaml-cpp-dev libgoogle-glog-dev libapr1-dev \
    ros-noetic-cv-bridge ros-noetic-octomap-server ros-noetic-pcl-ros \
    ros-noetic-robot-state-publisher ros-noetic-tf ros-noetic-tf2-ros \
    ros-noetic-topic-tools
fi

WHEELS="$ROOT/third_party/python_wheels"
VENV="$ROOT/.venv/ariadne"
[ -f "$WHEELS/torch-2.3.1-cp38-cp38-manylinux_2_17_aarch64.manylinux2014_aarch64.whl" ] \
  || { echo '[FATAL] bundled ARM64 PyTorch wheel is missing' >&2; exit 42; }
if [ ! -x "$VENV/bin/python" ]; then
  # --without-pip also works on minimal Ubuntu images where ensurepip is split
  # into python3.x-venv.  The system pip installed above populates this isolated
  # environment exclusively from the bundled wheel directory.
  python3 -m venv --without-pip "$VENV"
fi
SITE_PACKAGES="$("$VENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
python3 -m pip install --upgrade --no-index --find-links "$WHEELS" --target "$SITE_PACKAGES" \
  torch==2.3.1 numpy==1.24.4 scipy==1.10.1 scikit-image==0.21.0 \
  matplotlib==3.7.5 rospkg==1.6.0 PyYAML==6.0.3

if [ ! -f "$ROOT/config/local/board.json" ]; then
  if [ -f "$HOME/oksh.py" ]; then
    python3 "$ROOT/tools/configure_board.py" --import-oksh "$HOME/oksh.py"
  elif [ "$NON_INTERACTIVE" = false ]; then
    python3 "$ROOT/tools/configure_board.py"
  else
    echo '[WARN] board credentials not configured; run tools/configure_board.py before a real launch'
  fi
fi

echo '[SETUP] local Python environment and board configuration are ready'
echo '[SETUP] run ./build_all.sh next'
