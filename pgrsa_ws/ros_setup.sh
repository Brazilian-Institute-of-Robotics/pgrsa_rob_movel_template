#!/bin/bash

# ===== ROS 2 =====
source /opt/ros/humble/setup.bash

# ===== Workspace principal =====
if [ -f ~/pgrsa_rob_movel_template/pgrsa_ws/install/setup.bash ]; then
    source ~/pgrsa_rob_movel_template/pgrsa_ws/install/setup.bash
fi

# ===== Outros workspaces (opcional) =====
# source ~/outro_ws/install/setup.bash
source /opt/ros/humble/setup.bash
source ~/ugv_nav4d/build/install/env.sh
# ===== QoL =====
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0

echo "ROS environment loaded ✅"
