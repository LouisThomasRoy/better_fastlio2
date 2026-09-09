#!/usr/bin/env bash
# Sourced into every shell the container starts. Three workspaces chain here:
#   /opt/ros/noetic   ROS itself
#   /opt/deps_ws      livox_ros_driver + darknet_ros_msgs, built into the image
#   /catkin_ws        yours, bind-mounted from the host (may not be built yet)
set -e
source /opt/ros/noetic/setup.bash
source /opt/deps_ws/devel/setup.bash
if [[ -f /catkin_ws/devel/setup.bash ]]; then
    source /catkin_ws/devel/setup.bash
fi

# Same three lines for interactive shells opened later with `docker exec`.
if [[ ! -f "$HOME/.bashrc" ]] || ! grep -q deps_ws "$HOME/.bashrc" 2>/dev/null; then
    cat >> "$HOME/.bashrc" <<'RC'
source /opt/ros/noetic/setup.bash
source /opt/deps_ws/devel/setup.bash
[[ -f /catkin_ws/devel/setup.bash ]] && source /catkin_ws/devel/setup.bash
RC
fi

if [[ -t 1 && "$1" == "bash" ]]; then
    cat <<'BANNER'
------------------------------------------------------------------
 better_fastlio2 + GNSS  --  ROS Noetic
   /catkin_ws   your workspace   (bind-mounted from the host)
   /datasets    the bags         (read-only)
   /output      results          (bind-mounted, writable)

   build:  cd /catkin_ws && catkin build fast_lio_sam -j$(nproc)
   run:    roslaunch fast_lio_sam mapping_charlie8.launch seq:=field
   play:   rosbag play /datasets/field.bag        (in a second terminal)
   save:   rosservice call /save_map "{resolution: 0.0, destination: ''}"
------------------------------------------------------------------
BANNER
fi

exec "$@"
