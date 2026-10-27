# source ~/humanoid_sdk/env.sh        -- on the robot computer and on the PC alike
#
# ROS 2 Humble (or TogetheROS on the RDK X5), the description workspace if one
# is built separately, this workspace, and the ROS_DOMAIN_ID both machines must
# share. Override with BHL_ROS_DOMAIN_ID (the RDK X5 pairing in
# Berkeley-humanoid-lite/onboard_x5 uses 77; DexHand MoveIt sits on 0).
_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
elif [ -f /opt/tros/humble/setup.bash ]; then
  source /opt/tros/humble/setup.bash
else
  echo "env.sh: no ROS 2 Humble found (/opt/ros/humble or /opt/tros/humble)" >&2
fi
# URDF + meshes (berkeley_humanoid_lite_description): either a separate
# Berkeley-Humanoid-Lite checkout built at BHL_DESCRIPTION_WS, or imported into
# this workspace with humanoid.repos (then this line finds nothing and is a no-op).
BHL_DESCRIPTION_WS="${BHL_DESCRIPTION_WS:-$HOME/Berkeley-Humanoid-Lite/ros2_ws}"
[ -f "$BHL_DESCRIPTION_WS/install/setup.bash" ] && source "$BHL_DESCRIPTION_WS/install/setup.bash"
[ -f "$_ws/ros2_ws/install/setup.bash" ] && source "$_ws/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="${BHL_ROS_DOMAIN_ID:-92}"
export ROS_LOCALHOST_ONLY=0
unset _ws
