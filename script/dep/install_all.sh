#! /bin/bash

set -e

zsibot_CURRENT_PREFIX=$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

bash ${zsibot_CURRENT_PREFIX}/gazebo_dep.sh
bash ${zsibot_CURRENT_PREFIX}/lcm_dep.sh
bash ${zsibot_CURRENT_PREFIX}/osqp_dep.sh
bash ${zsibot_CURRENT_PREFIX}/unitree_sdk_dep.sh
bash ${zsibot_CURRENT_PREFIX}/livox_sdk_dep.sh
bash ${zsibot_CURRENT_PREFIX}/ros2_dep.sh
