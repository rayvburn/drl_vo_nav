# DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles

Implementation code for our paper ["DRL-VO: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles"](
https://doi.org/10.1109/TRO.2023.3257549
)([arXiv](https://arxiv.org/pdf/2301.06512.pdf)) in IEEE Transactions on Robotics (T-RO) 2023. 
This repository contains our DRL-VO code for training and testing the DRL-VO control policy in its [3D human-robot interaction Gazebo simulator](https://github.com/TempleRAIL/pedsim_ros_with_gazebo).
Video demos can be found at [multimedia demonstrations](https://www.youtube.com/watch?v=KneELRT8GzU&list=PLouWbAcP4zIvPgaARrV223lf2eiSR-eSS&index=2&ab_channel=PhilipDames).
Here are two GIFs showing our DRL-VO control policy for navigating in the simulation and real world. 
* Simulation:
![simulation_demo](demos/1.simulation_demo.gif "simulation_demo") 
* Real world:
![hardware_demo](demos/2.hardware_demo.gif "hardware_demo") 

## Introduction:
Our DRL-VO control policy is a novel learning-based control policy with strong generalizability to new environments that enables a mobile robot to navigate autonomously through spaces filled with both static obstacles and dense crowds of pedestrians. The policy uses a unique combination of input data to generate the desired steering angle and forward velocity: a short history of lidar data, kinematic data about nearby pedestrians, and a sub-goal point. The policy is trained in a reinforcement learning setting using a reward function that contains a novel term based on velocity obstacles to guide the robot to actively avoid pedestrians and move towards the goal. This DRL-VO control policy is tested in a series of 3D simulated experiments with up to 55 pedestrians and an extensive series of hardware experiments using a turtlebot2 robot with a 2D Hokuyo lidar and a ZED stereo camera. In addition, our DRL-VO control policy ranked 1st in the simulated competition and 3rd in the final physical competition of the ICRA 2022 BARN Challenge, which is tested in highly constrained static environments using a Jackal robot. The deployment code for ICRA 2022 BARN Challenge can be found at ["nav-competition-icra2022-drl-vo"](https://github.com/TempleRAIL/nav-competition-icra2022-drl-vo).

## Requirements:
* Ubuntu 20.04
* ROS-Noetic
* Python 3.8.5
* Pytorch 1.7.1
* Tensorboard 2.4.1
* Gym 0.18.0
* Stable-baseline3 1.1.0

## Installation:
This package requires these packages: 
* [robot_gazebo](https://github.com/TempleRAIL/robot_gazebo): contains our custom configuration files and maps for turtlebot2 navigation.
* [pedsim_ros_with_gazebo](https://github.com/TempleRAIL/pedsim_ros_with_gazebo): our customized 3D human-robot interaction Gazebo simulator based on [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros).
* [turtlebot2 packages](https://github.com/zzuxzt/turtlebot2_noetic_packages): turtlebot2 packages on ROS noetic.

We provide two ways to install our DRL-VO navigation packages on Ubuntu 20.04:
1) independently install them on your PC;
2) use a [pre-created singularity container](https://doi.org/10.5281/zenodo.7679658) directly (no need to configure the environment).

### 1) Independent installation on PC:
1. install ROS Noetic by following [ROS installation document](http://wiki.ros.org/noetic/Installation/Ubuntu). 
2. install required learning-based packages:
```
pip install torch==1.7.1+cu110 -f https://download.pytorch.org/whl/torch_stable.html
pip install gym==0.18.0 pandas==1.2.1
pip install stable-baselines3==1.1.0
pip install tensorboard psutil cloudpickle
```
3. install Turtlebot2 ROS packages:
```
sudo apt-get install ros-noetic-move-base*
sudo apt-get install ros-noetic-map-server*
sudo apt-get install ros-noetic-amcl*
sudo apt-get install ros-noetic-navigation*
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
wget https://raw.githubusercontent.com/zzuxzt/turtlebot2_noetic_packages/master/turtlebot2_noetic_install.sh 
sudo sh turtlebot2_noetic_install.sh 
```
4. install DRL-VO ROS navigation packages:
```
cd ~/catkin_ws/src
git clone https://github.com/TempleRAIL/robot_gazebo.git
git clone https://github.com/TempleRAIL/pedsim_ros_with_gazebo.git
git clone https://github.com/TempleRAIL/drl_vo_nav.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.sh
```

### 2) Using singularity container: all required packages are installed
1. install singularity software:
```
cd ~
wget https://github.com/sylabs/singularity/releases/download/v3.9.7/singularity-ce_3.9.7-bionic_amd64.deb
sudo apt install ./singularity-ce_3.9.7-bionic_amd64.deb
```
2. download pre-created ["drl_vo_container.sif"](https://doi.org/10.5281/zenodo.7679658) to the home directory.

3. install DRL-VO ROS navigation packages:
```
cd ~
singularity shell --nv drl_vo_container.sif
source /etc/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/TempleRAIL/robot_gazebo.git
git clone https://github.com/TempleRAIL/pedsim_ros_with_gazebo.git
git clone https://github.com/TempleRAIL/drl_vo_nav.git
cd ..
catkin_make
source ~/catkin_ws/devel/setup.sh
```

4. ctrl + D to exit the singularity container.

## Usage:
### Running on PC:
*  train on desktop (with a GUI): the trained models and log files will be stored in "~/drl_vo_runs"
```
roscd drl_vo_nav
cd ..
sh run_drl_vo_policy_training_desktop.sh ~/drl_vo_runs
```
*  train on server (without a GUI): the trained models and log files will be stored in "~/drl_vo_runs"
```
roscd drl_vo_nav
cd ..
sh run_drl_vo_policy_training_server.sh ~/drl_vo_runs
```
*  inference on desktop (navigation):
```
roscd drl_vo_nav
cd ..
sh run_drl_vo_navigation_demo.sh
```
You can then use the "2D Nav Goal" button on Rviz to set a random goal for the robot, as shown below:
![sending_goal_demo](demos/3.sending_goal_demo.gif "sending_goal_demo") 

### Running on singularity container:
*  train on desktop (with a GUI): the trained models and log files will be stored in "~/drl_vo_runs"
```
cd ~
singularity shell --nv drl_vo_container.sif
source /etc/.bashrc
source ~/catkin_ws/devel/setup.sh
roscd drl_vo_nav
cd ..
sh run_drl_vo_policy_training_desktop.sh ~/drl_vo_runs
```
*  train on server (without a GUI): the trained models and log files will be stored in "~/drl_vo_runs"
```
cd ~
singularity shell --nv drl_vo_container.sif
source /etc/.bashrc
source ~/catkin_ws/devel/setup.sh
roscd drl_vo_nav
cd ..
sh run_drl_vo_policy_training_server.sh ~/drl_vo_runs
```
*  inference on desktop (navigation):
```
cd ~
singularity shell --nv drl_vo_container.sif
source /etc/.bashrc
source ~/catkin_ws/devel/setup.sh
roscd drl_vo_nav
cd ..
sh run_drl_vo_navigation_demo.sh
```
You can then use the "2D Nav Goal" button on Rviz to set a random goal for the robot, as shown below:
![sending_goal_demo](demos/3.sending_goal_demo.gif "sending_goal_demo") 

## Citation
```
@article{xie2023drl,
  author={Xie, Zhanteng and Dames, Philip},
  journal={IEEE Transactions on Robotics}, 
  title={{DRL-VO}: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles}, 
  year={2023},
  volume={39},
  number={4},
  pages={2700-2719},
  doi={10.1109/TRO.2023.3257549}}

@article{xie2023drl,
  title={{DRL-VO}: Learning to Navigate Through Crowded Dynamic Scenes Using Velocity Obstacles},
  author={Xie, Zhanteng and Dames, Philip},
  journal={arXiv preprint arXiv:2301.06512},
  year={2023}
}

```

## Fork information

Main features of this fork:

- resolved `requirements` appropriate for Ubuntu 18 and ROS Melodic,
- prepared scripts to install the package dependencies (`common` for a real-world operation, and `sim` for a simulated setup referenced by the authors)
  - the aim was to run the navigation locally (without a Docker or so) on Ubuntu 18 and ROS Melodic,
- `CMakeLists` modified to automatically install the required Python version and create the virtual environment during the first building (`catkin build`),
- scripts and launch files were parameterized to run the algorithm on a different robotic system.

Introduced changes might not be compatible with launching the system in `train` mode.

Prerequisites:

```sh
sudo apt-get install -y \
  wget \
  ros-melodic-sophus
```

Once installed, simply run the all-in-one, parameterized launch:

```sh
roslaunch drl_vo_nav drl_vo_run_policy_agent.launch
```

adding the required remappings/parameters (see the source code).
