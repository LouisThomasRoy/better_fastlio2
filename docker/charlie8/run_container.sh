#!/usr/bin/env bash
# Start (or re-enter) the better_fastlio2 + GNSS container.
#
# The STUDENT_GUIDE walks through the `docker run` command this replaces --
# learn it there first, then use this to save typing. Run it again from another
# terminal and it attaches a second shell to the SAME running container, which
# is how you play a bag while the mapping node is running.
#
#   ./docker/charlie8/run_container.sh
#
# Override any of these from the environment:
#   CATKIN_WS  host catkin workspace   (default: two levels above this repo)
#   DATASETS   host folder of bags     (default: $HOME/slam/datasets)
#   OUTPUT     host folder for results (default: $HOME/slam/output)
#   IMAGE      image tag               (default: bfl2-gnss:noetic)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CATKIN_WS="${CATKIN_WS:-$(cd "$REPO/../.." && pwd)}"
DATASETS="${DATASETS:-$HOME/slam/datasets}"
OUTPUT="${OUTPUT:-$HOME/slam/output}"
IMAGE="${IMAGE:-bfl2-gnss:noetic}"
NAME="${NAME:-bfl2}"

# Second and later terminals: attach to the container that is already up.
if docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
    echo "[attaching a new shell to the running '$NAME' container]"
    exec docker exec -it "$NAME" bash
fi

[[ -d "$CATKIN_WS/src" ]] || { echo "no $CATKIN_WS/src -- is the repo cloned into <catkin_ws>/src/?"; exit 1; }
mkdir -p "$OUTPUT"
[[ -d "$DATASETS" ]] || { echo "no dataset folder at $DATASETS (set DATASETS=...)"; exit 1; }

# Let the container talk to the host X server, for rviz.
xhost +local:docker >/dev/null 2>&1 || echo "NOTE: 'xhost' unavailable; rviz may not open a window."

# Hardware-accelerated rviz if the host exposes a GPU render node, and membership
# of the groups that own it. Falls back to software rendering (llvmpipe), which
# is slower but works everywhere -- export LIBGL_ALWAYS_SOFTWARE=1 to force it.
gpu_args=()
if [[ -e /dev/dri ]]; then
    gpu_args+=(--device /dev/dri:/dev/dri)
    for node in /dev/dri/renderD* /dev/dri/card*; do
        [[ -e "$node" ]] && gpu_args+=(--group-add "$(stat -c '%g' "$node")")
    done
fi

exec docker run -it --rm \
    --name "$NAME" \
    --net=host \
    --shm-size=2g \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$CATKIN_WS":/catkin_ws \
    -v "$DATASETS":/datasets:ro \
    -v "$OUTPUT":/output \
    "${gpu_args[@]}" \
    "$IMAGE"
