#!/usr/bin/env bash
#
SCRIPT_DIR=$(realpath $(dirname $0))
cd $SCRIPT_DIR/../..

sudo apt-get install -y ros-melodic-sophus
rosinstall -n . drl_vo_nav/repos_common.rosinstall

# get rid of some packages - those are necessary only when this repo is not used:
# https://github.com/zzuxzt/turtlebot2_noetic_packages/tree/master
YUJIN_OCS_DIR="drl_vo_common/yujin_ocs"
mv \
    ${YUJIN_OCS_DIR}/yocs_cmd_vel_mux \
    ${YUJIN_OCS_DIR}/yocs_controllers \
    ${YUJIN_OCS_DIR}/yocs_velocity_smoother/ \
    drl_vo_common/
rm -rf ${YUJIN_OCS_DIR}/

# only certain subpackages from `pedsim_ros_with_gazebo` can be used in non-simulation setup
PEDSIM_DIR="drl_vo_common/pedsim_ros_gazebo"
mv \
    ${PEDSIM_DIR}/pedsim_msgs \
    drl_vo_common/
rm -rf ${PEDSIM_DIR}/
