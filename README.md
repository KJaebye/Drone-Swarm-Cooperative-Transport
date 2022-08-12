# Introduction
It is the github repo for the paper: Decentralised aerial swarm for adaptive and energy efficient transport of unknown loads. https://doi.org/10.1016/j.swevo.2021.100957

Cooperative transport by a swarm of Quadcopters offers more flexibility and performance when carrying loads that are complex in structural profile and mass. Ensuring that team members of the swarm are optimally placed on these loads as well as able to resist disturbances from the environment during transport are current research challenges. In this paper, we present a decentralized behaviour based subsumption architecture for enabling a swarm of Quadcopters to explore an unfamiliar area, find a load and transport it to a target location cooperatively. In the architecture, three behaviours were used: an obstacle avoidance behaviour to avoid collisions with objects in the environment, a flocking behaviour to ensure swarm structure and a bacterium behaviour for exploration of the environment and to adapt to the mass profile of various detected loads.

By adapting to the mass profile of a detected load, we show that our architecture ensures even energy distribution among Quadcopters while achieving robustness to disturbances from the environment. Our results show that a mass adapting swarm is able to conserve energy during payload transportation when compared to a swarm that does not adapt to a load’s profile. Furthermore, we do not use explicit communication between team members but instead rely on data from visual sensors attached to the Quadcopters. We experiment with simulations in a physics informed robot simulator called CoppeliaSim and demonstrate the effectiveness of our architecture when utilized for cooperative transport of irregular loads.

# Environment Configuration
## 1. Set up ROS Noetic
## 1.1 Configure your Ubuntu repositories
The correspnding ubuntu version of ROS Noetic is Ubuntu20.04.

Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the Ubuntu guide for instructions on doing this.
### 1.2 Setup your sources.list
Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
### 1.3 Set up your keys
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
### 1.4 Installation
First, make sure your Debian package index is up-to-date:
```
sudo apt update
```
Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages
```
sudo apt install ros-noetic-desktop-full
```
### 1.5 Environment setup
You must source this script in every bash terminal you use ROS in.
```
source /opt/ros/noetic/setup.bash
```
It can be convenient to automatically source this script every time a new shell is launched. We can add this command into ~/.bashrc
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### 1.6 Check installation
Execute roscore in terminal and the result should be like this:
```
kjaebye@kjaebye-XPS-15-7590:~$ roscore
... logging to /home/kjaebye/.ros/log/eb8e0844-1a08-11ed-add9-2b4676a435c8/roslaunch-kjaebye-XPS-15-7590-127559.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://kjaebye-XPS-15-7590:39887/
ros_comm version 1.15.14


SUMMARY
========

PARAMETERS
 * /rosdistro: noetic
 * /rosversion: 1.15.14

NODES

auto-starting new master
process[master]: started with pid [127567]
ROS_MASTER_URI=http://kjaebye-XPS-15-7590:11311/

setting /run_id to eb8e0844-1a08-11ed-add9-2b4676a435c8
process[rosout-1]: started with pid [127577]
started core service [/rosout]
^C[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```
### 1.7 Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:
```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
### 1.7.1 Initialize rosdep
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
```
sudo apt install python3-rosdep
```
With the following, you can initialize rosdep.
```
sudo rosdep init
rosdep update
```
## 2. Create a ROS workspace
### 2.1 Install catkin_tools by apt-install
First you must have the ROS repositories which contain the .deb for catkin_tools:
```
sudo sh \
    -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" \
        > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
```
Once you have added that repository, run these commands to install catkin_tools:
```
sudo apt-get update
sudo apt-get install python3-catkin-tools
```
### 2.2 Create a ROS Workspace
Let's create and build a catkin workspace:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin build
```
The 'catkin build' command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder.

Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder. Inside the 'devel' folder you can see that there are now several setup.*sh files. Sourcing any of these files will overlay this workspace on top of your environment. To understand more about this see the general catkin documentation: catkin. Before continuing source your new setup.*sh file:
```
source devel/setup.bash
```
To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.
```
kjaebye@kjaebye-XPS-15-7590:~/catkin_ws$ echo $ROS_PACKAGE_PATH
/home/kjaebye/catkin_ws/src:/opt/ros/noetic/share
```
If your workspapce path /home/kjaebye/catkin_ws/src is not in envorinment path, export the path:
```
export ROS_PACKAGE_PATH=/home/kjaebye/catkin_ws/src:$ROS_PACKAGE_PATH
```
Make sure this env path exit in all your terminals that runs ROS. For convinience, add this command into ~/.bashrc
```
echo "export ROS_PACKAGE_PATH=/home/kjaebye/catkin_ws/src:$ROS_PACKAGE_PATH" >> ~/.bashrc
source ~/.bashrc
```
### 2.3 Install requirements
```
sudo apt install ros-noetic-geographic-msgs
sudo apt install ros-noetic-octomap-msgs
sudo apt install ros-noetic-mavlink
sudo apt install autoconf
sudo apt install ros-noetic-tf2-sensor-msgs
sudo apt install libgeographic-dev
sudo apt install ros-noetic-control-toolbox
```
#### 2.3.1 Check Eigen3
Check ubuntu has Eigen3 or not:
```
sudo apt install mlocate
sudo updatedb
```
If there are eigen3 directions but no path '/usr/include/Eigen', it means eigen3 has been installed but cannot be detected in a right position, because the Eigen lib is installed to /usr/include/eigen3/Eigen by default. We need to map it to /usr/include/Eigen:
```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```
If Eigen lib is not installed, install by:
```
sudo apt install libeigen3-dev
```
### 2.4 Build the project
This project is based on three dependencies: glog_catkin, catkin_simple, mav_comm.
Make sure the package mav_comm is the PX4 version.
Download these packages at /catkin_ws/src/, and put the package drone_controller.
```
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/PX4/mav_comm.git

# download and put the package drone_controller into /src
git clone 
```
do 'catkin build' at /catkin_ws
```
cd ~/catkin_ws/
catkin build
source devel/setup.bash
```
## 3. Build CoppeliaSim
### 3.1 Download CoppeliaSim Edu 4.2 from official
Download file from home page. To open CoppeliaSim:
```
cd CoppeliaSim_Edu_V4_2_0_Ubuntu20_04/
./coppeliaSim.sh
```
If you find this: "This release will now stop working." and CoppeliaSim cannot open, simply add following in the system/usrset.txt file:
```
allowOldEduRelease=7501 # for CoppeliaSim Edu V4.1.0
allowOldEduRelease=7775 # for CoppeliaSim Edu V4.2.0
```
Edit: this issue affects only CoppeliaSim V4.1 and V4.2. CoppeliaSim V4.3 and later do not implement this limitation anymore.

### 3.2 Build ROS interface plugin for CoppeliaSim
Please follow the instructions from CoppeliaSim official user manual

Download interface package into /catkin_ws/src
```
git clone --recursive https://github.com/CoppeliaRobotics/simExtROS.git sim_ros_interface
```
**Note: Because we use an external msg 'mav_msgs/CommandMotorSpeed' in this project, we need to add this to /sim_ros_interface/meta/messages.txt**

Also, add package name "mav_msgs" into sim_ros_interface CMakeLists.text and package.xml

In order to build the packages, navigate to the catkin_ws folder and type:
```
export COPPELIASIM_ROOT_DIR=~/path/to/coppeliaSim/folder
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Build this package can generate a library named libsimExtROS.so, located at ~/catkin_ws/devel/lib/. Copy this file and replace the file of the same name at CoppeliaSim root directory. This library is the communication interface between CoppeliaSim and ROS.

### 3.3 Check ROS plugin is loaded
**Note:
Before open the CoppeliaSim, ros should be launched firstly!!!**
```
roscore
```
Then in another terminal open the CoppeliaSim. In logging text we should find:
```
[CoppeliaSim:loadinfo]   plugin 'ROS': loading...
[CoppeliaSim:loadinfo]   plugin 'ROS': load succeeded.
```

# Citation
We are glad to share our ideas with you! Fork or use this work please use this citation:
@article{huang2021decentralised,
  title={Decentralised aerial swarm for adaptive and energy efficient transport of unknown loads},
  author={Huang, Kangyao and Chen, Jingyu and Oyekan, John},
  journal={Swarm and Evolutionary Computation},
  volume={67},
  pages={100957},
  year={2021},
  publisher={Elsevier}
}
