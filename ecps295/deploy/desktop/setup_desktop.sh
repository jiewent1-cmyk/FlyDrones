#!/usr/bin/env bash
# ECPS295 desktop deployment: Ubuntu 24.04 + NVIDIA GPU -> the same Gazebo/SITL stack as ROG and the server.
#
#   Gazebo Harmonic (OSRF apt)  +  ArduPilot Copter-4.7.0 SITL with the 2 local patches (sitl/ardupilot_sitl.patch)
#   +  ardupilot_gazebo 082a0fe  +  conda envs flydrones / ardupilot (pinned, see requirements_*.txt)
#   +  ~/sim/FlyDrones (branch ecps295-sim)  +  hw tree ~/sim/hw/{ecps295,src}  +  launchers in ~/sim
#
# usage:  bash setup_desktop.sh                  all stages, in order
#         bash setup_desktop.sh apt conda ...     only these stages
#   stages: check driver apt conda flydrones ardupilot gzplugin hw launchers
#   FORCE=1            redo stages already marked done (markers in ~/sim/.deploy)
#   HW_TAR=file.tgz    hw tree packed from the server (pack_hw_tree.sh) instead of branch + hw_run_g4.patch
#   BRANCH / REPO      FlyDrones source (default ecps295-sim of github.com/jiewent1-cmyk/FlyDrones)
#   JOBS=n             build jobs (default nproc)
# Afterwards: bash ~/sim/verify_desktop.sh  (gates/anchors, lowbox_s0 v4 vs ROG/server, RTF with 1/3/4 instances).
# Only apt and the NVIDIA driver use sudo; everything else lives in $HOME (~/sim, ~/miniconda3).
set -euo pipefail

BRANCH=${BRANCH:-ecps295-sim}
REPO=${REPO:-https://github.com/jiewent1-cmyk/FlyDrones.git}
JOBS=${JOBS:-$(nproc)}
SIM=$HOME/sim
CONDA=$HOME/miniconda3  # same prefix as ROG: every launcher sources ~/miniconda3/etc/profile.d/conda.sh
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MARK=$SIM/.deploy
ALL=(check driver apt conda flydrones ardupilot gzplugin hw launchers)
mkdir -p "$SIM" "$MARK"

log() { printf '\n\033[1;34m== %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33mWARN: %s\033[0m\n' "$*" >&2; }
die() { printf '\033[1;31mERROR: %s\033[0m\n' "$*" >&2; exit 1; }
done_() { touch "$MARK/$1"; }
skip() { [ -z "${FORCE:-}" ] && [ -f "$MARK/$1" ] && { echo "stage $1 already done (FORCE=1 to redo)"; return 0; }; return 1; }

st_check() {
  log "check: OS, CPU, disk, sudo, network"
  . /etc/os-release
  [ "$ID" = ubuntu ] && [ "$VERSION_ID" = 24.04 ] || warn "tested on Ubuntu 24.04 (noble), this is $PRETTY_NAME"
  [ "$(uname -m)" = x86_64 ] || die "x86_64 only (ArduPilot SITL / OSRF packages)"
  echo "CPU: $(grep -m1 'model name' /proc/cpuinfo | cut -d: -f2) ($(nproc) threads), RAM $(free -g | awk '/Mem/ {print $2}') GB"
  free=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc 0-9)
  [ "$free" -ge 30 ] || die "need >= 30 GB free under $HOME (have $free GB)"
  sudo -v || die "sudo is needed for apt and the NVIDIA driver"
  curl -fsI https://github.com > /dev/null || die "no network (github.com)"
  done_ check
}

st_driver() {
  log "driver: NVIDIA (Gazebo renders the camera and the ToF on the GPU via EGL)"
  if nvidia-smi > /dev/null 2>&1; then
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader
    echo "kernel $(uname -r)"
    done_ driver
    return
  fi
  warn "nvidia-smi does not work: installing the recommended driver (RTX 50xx needs the open driver >= 570)"
  sudo apt-get update
  sudo apt-get install -y ubuntu-drivers-common
  ubuntu-drivers devices || true
  sudo ubuntu-drivers install
  echo
  echo ">>> Reboot now (with Secure Boot on, enrol the MOK key when asked), then run this script again."
  echo ">>> After every kernel update check nvidia-smi: on ROG a new kernel without NVIDIA modules broke Gazebo."
  exit 0
}

st_apt() {
  log "apt: build tools, Gazebo Harmonic (OSRF), ardupilot_gazebo build deps, gz python bindings for cam_bridge.py"
  sudo apt-get update
  sudo apt-get install -y curl wget git lsb-release gnupg ca-certificates build-essential cmake pkg-config ccache \
    rsync iproute2 python3-numpy python3-pil
  if [ ! -f /etc/apt/sources.list.d/gazebo-stable.list ]; then
    sudo curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
    sudo apt-get update
  fi
  sudo apt-get install -y gz-harmonic libgz-sim8-dev rapidjson-dev \
    python3-gz-transport13 python3-gz-msgs10 python3-gz-sim8 python3-gz-math7 \
    libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
  v=$(gz sim --version 2>/dev/null | grep -o '[0-9]*\.[0-9]*\.[0-9]*' | head -1)
  echo "Gazebo Sim $v"
  [ "$v" = 8.15.0 ] || warn "ROG runs gz-sim 8.15.0 (server 8.10): results are comparable, but note the version in reports"
  /usr/bin/python3 -c "from gz.transport13 import Node; from gz.msgs10.image_pb2 import Image; import numpy, PIL" \
    || die "system python cannot import gz.transport13 / gz.msgs10 / numpy / PIL (needed by cam_bridge.py)"
  done_ apt
}

st_conda() {
  log "conda: Miniforge in ~/miniconda3, envs flydrones + ardupilot (python 3.11, pinned)"
  if [ ! -x "$CONDA/bin/conda" ]; then
    [ -e "$CONDA" ] && die "$CONDA exists but has no conda; move it away first"
    f=$(mktemp --suffix=.sh)
    curl -fsSL -o "$f" https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
    bash "$f" -b -p "$CONDA"
    rm -f "$f"
  fi
  for env in flydrones ardupilot; do
    [ -x "$CONDA/envs/$env/bin/python" ] || "$CONDA/bin/conda" create -y -q -n $env python=3.11
    "$CONDA/envs/$env/bin/pip" install -q -r "$HERE/requirements_$env.txt"
  done
  "$CONDA/envs/flydrones/bin/python" -c "import numpy, scipy; print('flydrones env: numpy', numpy.__version__, 'scipy', scipy.__version__)"
  done_ conda
}

st_flydrones() {
  log "flydrones: ~/sim/FlyDrones ($BRANCH), editable install, MiniFly weights"
  if [ ! -d "$SIM/FlyDrones/.git" ]; then
    git clone --branch "$BRANCH" "$REPO" "$SIM/FlyDrones"
  elif [ -z "$(git -C "$SIM/FlyDrones" status --porcelain --untracked-files=no)" ]; then
    git -C "$SIM/FlyDrones" fetch -q origin && git -C "$SIM/FlyDrones" checkout -q "$BRANCH" && git -C "$SIM/FlyDrones" merge -q --ff-only "origin/$BRANCH"
  else
    warn "~/sim/FlyDrones has local changes: not updated"
  fi
  echo "FlyDrones at $(git -C "$SIM/FlyDrones" log --oneline -1)"
  "$CONDA/envs/flydrones/bin/pip" install -q --no-deps -e "$SIM/FlyDrones"
  cd "$SIM/FlyDrones/ecps295"
  for v in v2 v3 v4; do
    [ -f minifly/minifly_$v.npz ] || "$CONDA/envs/flydrones/bin/python" my_minifly.py $v --out minifly/minifly_$v.npz
  done
  sha256sum minifly/*.npz
  done_ flydrones
}

st_ardupilot() {
  log "ardupilot: Copter-4.7.0 + sitl/ardupilot_sitl.patch (JSON rangefinder bitmask, flow over obstacles), SITL build"
  local P=$SIM/FlyDrones/ecps295/sitl/ardupilot_sitl.patch
  [ -f "$P" ] || die "run the flydrones stage first (needs $P)"
  if [ ! -d "$SIM/ardupilot/.git" ]; then
    git clone --branch Copter-4.7.0 --depth 1 https://github.com/ArduPilot/ardupilot.git "$SIM/ardupilot"
  fi
  cd "$SIM/ardupilot"
  git submodule update --init --recursive --depth 1 || git submodule update --init --recursive
  if git apply --check -R "$P" 2> /dev/null; then
    echo "patch already applied"
  else
    git apply "$P"
  fi
  git diff --stat
  PATH=$CONDA/envs/ardupilot/bin:$PATH ./waf configure --board sitl > "$SIM/waf_configure.log" 2>&1 || { tail -20 "$SIM/waf_configure.log"; die "waf configure failed"; }
  PATH=$CONDA/envs/ardupilot/bin:$PATH ./waf copter -j"$JOBS" > "$SIM/waf_build.log" 2>&1 || { tail -30 "$SIM/waf_build.log"; die "waf copter failed"; }
  [ -x build/sitl/bin/arducopter ] || die "build/sitl/bin/arducopter missing"
  echo "built $(ls -la build/sitl/bin/arducopter)"
  done_ ardupilot
}

st_gzplugin() {
  log "gzplugin: ardupilot_gazebo 082a0fe (ArduPilotPlugin, JSON link to SITL)"
  [ -d "$SIM/ardupilot_gazebo/.git" ] || git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$SIM/ardupilot_gazebo"
  cd "$SIM/ardupilot_gazebo"
  git fetch -q origin && git checkout -q 082a0fe
  mkdir -p build && cd build
  GZ_VERSION=harmonic cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo > "$SIM/gzplugin_cmake.log" 2>&1 || { tail -20 "$SIM/gzplugin_cmake.log"; die "cmake failed"; }
  make -j"$JOBS" > "$SIM/gzplugin_build.log" 2>&1 || { tail -30 "$SIM/gzplugin_build.log"; die "make failed"; }
  [ -f libArduPilotPlugin.so ] || die "libArduPilotPlugin.so missing"
  done_ gzplugin
}

st_hw() {
  log "hw: hardware-aligned run tree ~/sim/hw/{ecps295,src} (what the server's gz_batch.py --hw uses)"
  local HW=$SIM/hw PY=$CONDA/envs/flydrones/bin/python
  rm -rf "$HW.new"
  mkdir -p "$HW.new"
  if [ -n "${HW_TAR:-}" ]; then
    tar xzf "$HW_TAR" -C "$HW.new"
    [ -f "$HW.new/ecps295/run_g4.py" ] && [ -d "$HW.new/src/flydrones" ] || die "$HW_TAR is not an hw tree (ecps295/ + src/)"
    echo "source: $(basename "$HW_TAR") sha256 $(sha256sum "$HW_TAR" | cut -c1-16)" > "$HW.new/DEPLOY_SOURCE.txt"
  else
    # branch tree + the server's two env-gated run_g4.py changes (ECPS_SIMCLOCK, ECPS_ARM; both off by default)
    rsync -a --exclude __pycache__ --exclude results "$SIM/FlyDrones/ecps295" "$HW.new/"
    rsync -a --exclude __pycache__ "$SIM/FlyDrones/src" "$HW.new/"
    cp "$HW.new/ecps295/rl/server/simclock.py" "$HW.new/ecps295/"
    patch -s -p1 -d "$HW.new/ecps295" < "$HERE/hw_run_g4.patch"
    echo "source: FlyDrones $(git -C "$SIM/FlyDrones" rev-parse --short HEAD) + hw_run_g4.patch" > "$HW.new/DEPLOY_SOURCE.txt"
  fi
  for v in v2 v3 v4; do  # weights are not in git; regenerate with the tree's own code if the source had none
    [ -f "$HW.new/ecps295/minifly/minifly_$v.npz" ] && continue
    (cd "$HW.new/ecps295" && PYTHONPATH=$HW.new/src:. "$PY" my_minifly.py $v --out minifly/minifly_$v.npz)
  done
  if [ -d "$HW" ]; then
    rm -rf "$HW.old"
    mv "$HW" "$HW.old"
    echo "previous tree kept as $HW.old"
  fi
  mv "$HW.new" "$HW"
  cat "$HW/DEPLOY_SOURCE.txt"
  done_ hw
}

st_launchers() {
  log "launchers: ~/sim/{gz_g4,run_matrix,run_matrix_par,run_ablation,run_resume,verify_desktop}.sh"
  local S=$SIM/FlyDrones/ecps295/scripts T='${ECPS_TREE:-$HOME/sim/hw/ecps295}'
  # same scripts as ROG; the run tree is ECPS_TREE (default the hw tree), and run_g4 sees that tree's src/ first
  sed -e "s#^E=~/sim/FlyDrones/ecps295\$#E=$T#" -e 's#PYTHONPATH=\$E MPLBACKEND#PYTHONPATH=$E:$(dirname $E)/src MPLBACKEND#' \
    "$S/gz_g4.sh" > "$SIM/gz_g4.sh"
  sed -e "s#^M=~/sim/FlyDrones/ecps295/minifly\$#M=$T/minifly#" "$S/run_matrix.sh" > "$SIM/run_matrix.sh"
  for f in run_ablation.sh run_resume.sh; do sed -e "s#\$HOME/sim/FlyDrones/ecps295#$T#g" "$S/$f" > "$SIM/$f"; done
  cp "$S/run_matrix_par.sh" "$SIM/"
  cp "$HERE/verify_desktop.sh" "$HERE/reference.json" "$SIM/"
  grep -q ECPS_TREE "$SIM/gz_g4.sh" && grep -q 'dirname $E)/src' "$SIM/gz_g4.sh" || die "gz_g4.sh rewrite failed (upstream script changed?)"
  grep -q ECPS_TREE "$SIM/run_matrix.sh" || die "run_matrix.sh rewrite failed"
  mkdir -p "$SIM/runs"
  done_ launchers
}

stages=("$@")
[ ${#stages[@]} -eq 0 ] && stages=("${ALL[@]}")
for s in "${stages[@]}"; do
  [[ " ${ALL[*]} " == *" $s "* ]] || die "unknown stage $s (stages: ${ALL[*]})"
  skip "$s" || "st_$s"
done
log "done. Next: bash ~/sim/verify_desktop.sh"
