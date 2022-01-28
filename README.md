# ORB-SLAM 2 Workspace
The development environment for the UGV


This repo contains submodules (forked from the original open source repos)
Run the following command after cloning, and whenever you want the submodules update

git submodule update --init --recursive

To learn more about submodules, this link may help: https://git-scm.com/book/en/v2/Git-Tools-Submodules

# TO Run

`git clone https://github.com/Capstone-W3/orb_slam2_workspace`

`cd orb_slam2_workspace`

`git submodule update --init --recursive`

`cd orb_slam_ws`

`catkin init`

`cd src/ORB-SLAM2_ROS/ORB_SLAM2`

`sudo chmod +x build*`

`./build_catkin.sh`
(All errors this throws should be due to missing packages)

`cd ../..` (to orb_slam_ws)

`catkin build`

`source devel/setup.bash`

````
roslaunch diff_drive_mapping_robot robot_offline.launch
rosbag play ${PATH TO THE BAG FILE}
roslaunch orb_slam2_ros raspicam_mono_wide.launch
````






# Packaged Needed 

### For all of ORB-SLAM

`sudo apt-get install ros-kinetic-desktop-full`

`sudo apt-get install ros-kinetic-catkin python-catkin-tools` 

### For only robot example

`sudo apt-get install ros-kinetic-move-base-msgs`

