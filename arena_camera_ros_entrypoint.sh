#!/bin/bash

set -e
cd /triton_cam_driver 
#rosdep fix-permissions
#rosdep update
rosdep install --from-paths src --ignore-src -r -y;
colcon build --symlink-install
source install/local_setup.bash

exec "$@"
