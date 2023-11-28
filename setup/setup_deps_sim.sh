#!/usr/bin/env bash
#
# NOT TESTED
#
SCRIPT_DIR=$(realpath $(dirname $0))

# remove any obsolete rosinstall configuration
cd $SCRIPT_DIR/../..
rm -rf .rosinstall .rosinstall.bak

cd $SCRIPT_DIR
./setup_deps_common.sh

# remove previous rosinstall configuration
cd $SCRIPT_DIR/../..
rm -rf .rosinstall .rosinstall.bak

rosinstall -n . drl_vo_nav/repos_sim.rosinstall

# Get rid of some packages that were already installed by the `common` script:
SIM_PKG_DIR="drl_vo_sim"

# NOTE: `turtlebot2_noetic_packages` metapackage contains only `kobuki_gazebo_plugins`, 3 `yocs_` packages
# and multiple submodules without working URLs. `yocs_` were already installed by the `common` script
mkdir -p ${SIM_PKG_DIR}/kobuki
mv \
    ${SIM_PKG_DIR}/turtlebot2_noetic_packages/kobuki_gazebo_plugins \
    ${SIM_PKG_DIR}/kobuki/
rm -rf ${SIM_PKG_DIR}/turtlebot2_noetic_packages

# NOTE: `pedsim_msgs` package is also embedded into the `pedsim_ros_with_gazebo` repository
rm -rf ${SIM_PKG_DIR}/pedsim_ros_gazebo/pedsim_msgs

# ignore hardware-specific packages for simulation
touch ${SIM_PKG_DIR}/kobuki/kobuki_core/kobuki_ftdi/CATKIN_IGNORE
# this one also relies on some ROS2 packages (https://github.com/stonier/ecl_core/tree/devel/ecl_mobile_robot)
touch ${SIM_PKG_DIR}/kobuki/kobuki_core/kobuki_driver/CATKIN_IGNORE
# below rely on the previous entry
touch ${SIM_PKG_DIR}/kobuki/kobuki/kobuki_node/CATKIN_IGNORE
touch ${SIM_PKG_DIR}/kobuki/kobuki/kobuki_testsuite/CATKIN_IGNORE

# delete rosinstall that is not needed at that point
rm .rosinstall
