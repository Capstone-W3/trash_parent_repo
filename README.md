# Terrestrial Roving Autonomous Scrap Harvester (TRASH)

Welcome to the home repository for Northeastern's EECE Capstone team named TRASH. The TRASH system is a robotics project designed to map out and pick up trash in a particular area. This file serves as a full guide to getting our codebase set up on your own machine.


# README Outline

1. [INTRODUCTION](#1-introduction)
2. [EXPECTED KNOWLDGE](#2-expected-knowldege)
    - Ubuntu 16.04
    - ROS (Kintetic)
3. [Project Setup Instructions](#3-setup-instructions)
    - Initial Downloads
      1. [Clone This Repository](#clone-this-repository)
      2. [General Dependencies](#general-dependencies)
    - [Speed Setup (Just Copy & Paste)](#speed-setup)
    - [Full Instructions](#full-instructions)
      1. [Complete Project](#)
      2. [Just ORB-SLAM2](#)
      3. [Just YOLOv4](#)
4. [Project Run Instructions](#)
    - ORB-SLAM2
    - YOLOv4
    - Full Project

5. [Contact Us](#contact)



# 1. Introduction

TRASH 


[Final Project Document](Capstone_Project_Writeup.pdf)


#### Software Overview

The TRASH system ’s software has three distinct stages (as seen below). In the first stage, the TRASH system maps its environment using a customized version of ORBSlam, an open source Monocular SLAM solution which can accurately create point clouds given RGB and depth camera input. We trained YOLO, an image identification CNN, with our own dataset, then using it to identify and mark the locations of trash clusters on the map. We then render the 3D point cloud down to a 2D occupancy grid using a custom implementation of Octomap. It is from this 2D occupancy grid that the robot can navigate around an environment.
Once the mapping stage is over, the TRASH system enters its second stage: general naviga- tion.The Turtlebot uses adaptive Monte Carlo localization (AMCL) to locate itself in the built map. That way it can identify obstacles and path plan around them to the trash cluster points labeled in that map.
Upon reaching a cluster, the third stage begins. YOLO identifies trash in the RGB image from the Realsense Camera, and from the coordinates of the trash detection and the distance measurement received from the depth camera, calculates the position of the trash relative to the robot. Once it identifies the relative position of the trash, the robot turns towards its target, starts the collection mechanism motor, and moves towards it, picking up the trash piece. Upon successful collection, it returns to the general navigation stage, repeating on until all trash clusters have been visited.



## 2. Expected Knowldege

words

#### THIS PROJECT ASSUMES WORKING ON UBUNTU 16.04 WITH ROS-KINETIC AND PYTHON3.5, THERE ARE MANY FIXES IMPLEMENTED THIS CONFIGURATION THAT WOULD NOT BE REQUIRED ON OTHER SYSTEMS

This repo contains submodules (forked from the original open source repos)
Run the following command after cloning, and whenever you want the submodules update

git submodule update --init --recursive

To learn more about submodules, this link may help: https://git-scm.com/book/en/v2/Git-Tools-Submodules

---

## 3. Setup Instructions

words

**Setup Instructions Outline**

1. [Clone This Repository](#clone-this-repository)
2. [General Dependencies](#general-dependencies)
    1. [pip3](#pip3)
    2.  [ROS](#ros)

## Clone This Repository

`git clone --recurse-submodules https://github.com/Capstone-W3/trash_parent_repo.git`

Then need to go through and checkout to the remote tracking branch in each

`git submodule update --recursive --remote`

OR if git version is 2.23 or greater

`git clone --recurse-submodules --remote-submodules https://github.com/Capstone-W3/trash_parent_repo.git`


## General Dependencies

### pip3

`sudo apt-get install python3-pip`

DO NOT run the suggested update command, pip > 21.0 does not support python3.5 and will break the entire system. This happened to me and took a long time to fix because as of Jan 2022 it seems to be impossible to upgrade python versions on Ubuntu 16.04 ([issue](https://github.com/deadsnakes/issues/issues/195)). 

To update versions without breaking everything:

`pip3 install -U "pip<21.0" setuptools`

**NOTE:** If you have already broken it like me, you have to install pip from another source as seen below, otherwise don't run this command

````
curl -sSL https://bootstrap.pypa.io/pip/3.5/get-pip.py -o get-pip.py
python3 get-pip.py "pip < 21.0"
````

### ROS

Install ros-kinetic

http://wiki.ros.org/kinetic/Installation/Ubuntu

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt install curl`

`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`

`sudo apt-get update`

`sudo reboot`

````
sudo apt-get install ros-kinetic-desktop-full

sudo apt-get install ros-kinetic-catkin python-catkin-tools

sudo apt-get install ros-kinetic-catkin python-controller-manager

sudo apt-get install ros-kinetic-pid
````

**DO NOT** run the suggested command from ros that adds `source /opt/ros/kinetic/setup.bash` to your `~/.bashrc`. This will cause the `ros` version of any package to be the system defaults (opencv will be sourced from the ros version not the one you install)

To check whether this has been done on pre-installed ros, open `~/.bashrc` (`code ~/.bashrc` for VSCode) and remove this line if already there.

For any terminal shell that you wish to run ros commands from you must now run

`source /opt/ros/kinetic/setup.bash` or for this specific project's catkin workspace `source ~/trash_parent_repo/catkin_ws/devel/setup.bash`



&nbsp; 

---

&nbsp; 



## ORB-SLAM2

#### ORB-SLAM2 Outline

- [Dependency Installs](#orb-slam2-dependencies)
    1. CMAKE
    2. Eigen3
    3. Pangolin
    4. OpenCV
- Building ORB-SLAM2

If desired you can jump directly to [running ORB-SLAM2](running-orb-slam2)

### ORB-SLAM2 Dependencies

#### CMake

Requires updated version of CMake, but many way of doing this update will break a pre-installed ros implementation

````
cd $HOME
wget https://github.com/Kitware/CMake/releases/download/v3.22.2/cmake-3.22.2.tar.gz
tar xzf cmake-3.22.2.tar.gz
rm -rf cmake-3.22.2.tar.gz
cd cmake-3.22.2
./configure
make
sudo make install
echo 'export PATH=$HOME/cmake-3.22.2/bin:$PATH' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$HOME/cmake-3.22.2:$CMAKE_PREFIX_PATH' >> ~/.bashrc
````

Check that everything worked using `cmake --version`

#### Eigen3

Install version 3.3.9 of Eigen3 because it doesn't recognize the base version, [as seen here](https://apolo-docs.readthedocs.io/en/latest/software/scientific_libraries/eigen/eigen-3.3.7/index.html)

````
cd $HOME
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -xzvf eigen-3.3.9.tar.gz 
rm eigen-3.3.9.tar.gz
cd eigen-3.3.9

mkdir build && cd build
cmake ..
sudo make install
````

#### Pangolin

````
cd ~/trash_parent_repo/Pangolin
./scripts/install_prerequisites.sh recommended
git checkout v0.6

mkdir -p build && cd build
cmake .. -DEigen3_DIR=$HOME/eigen-3.3.9/build
cmake --build .
````

The checkout to `v0.6` resolves issue [#715](https://github.com/stevenlovegrove/Pangolin/pull/715) from Pangolin

The `-DEigen3_DIR` flag must point to where you put eigen3, I set it up so this will default to the home directory but can be changed

#### OpenCV

````
cd ~/trash_parent_repo/opencv
git -C opencv checkout 3.4
mkdir -p build && cd build
cmake .. -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local
sudo make install
````

make sure that when you `cmake` that you see both versions of python

````
--   Python 2:
--     Interpreter:                 /usr/bin/python2.7 (ver 2.7.6)
--     Libraries:                   /usr/lib/x86_64-linux-gnu/libpython2.7.so (ver 2.7.6)
--     numpy:                       /usr/lib/python2.7/dist-packages/numpy/core/include (ver 1.8.2)
--     packages path:               lib/python2.7/dist-packages
--
--   Python 3:
--     Interpreter:                 /usr/bin/python3.4 (ver 3.4.3)
--     Libraries:                   /usr/lib/x86_64-linux-gnu/libpython3.4m.so (ver 3.4.3)
--     numpy:                       /usr/lib/python3/dist-packages/numpy/core/include (ver 1.8.2)
--     packages path:               lib/python3.4/dist-packages
````

If having issues with `make -j4` freezing the machine, remove the parallel option and just run `make`

https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html

https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html



If OpenCV is only needed for the dependencies from other repos you can just install through `pip3`

`pip3 install opencv-python==3.4.6.27`




#### Downloading datasets

For EuRoC datasets go to http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/ and download

If using http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/, download the zip file and unzip it in downloads

#### Run sample data

Then from `ORB_SLAM3` try

````
cd Examples
--------
````

### orb_slam3_ros_wrapper ([repo](https://github.com/thien94/orb_slam3_ros_wrapper))

must be contained within the catkin workspace

If the parent repo was not cloned to `$HOME`, follow instructions from above to update CMakeLists.txt with correct path

~~git clone https://github.com/thien94/orb_slam3_ros_wrapper.git~~

Make sure to have run:
`source /opt/ros/kinetic/setup.bash` but don't add to ~/ .bashrc

````
cd catkin_ws
catkin init

cd src/orb_slam3_ros_wrapper
git checkout master
cd ../..

catkin build
````

The steps described [here](https://github.com/thien94/orb_slam3_ros_wrapper#2-orb_slam3_ros_wrapper-this-package) for setting directories of ORB_SLAM and Vocabulary have already been done for the default location and structure of this repo, `$HOME/trash_parent_repo`

Install dependency for their publishers 

`sudo apt-get install ros-kinetic-hector-trajectory-server`

#### To run with gridmap

`roslaunch orb_slam3_ros_wrapper orb_slam3_mono_gridmap.launch`

and run with bag downloaded from [EuRoC](http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/)

`rosbag play V1_01_easy.bag`








&nbsp; 

&nbsp; 

&nbsp; 


**TRAIN YOLO USING GOOGLE COLAB AND ROBOFLOW FOR DATASET SETUP**

- https://colab.research.google.com/drive/1DRbdWh7pWrKC28hN1RmgkOhKlOxhPm9X#scrollTo=NjKzw2TvZrOQ
- https://blog.roboflow.com/train-yolov4-tiny-on-custom-data-lighting-fast-detection/
- https://app.roboflow.com/jack-fenton/trash-uavvaste/1

- Try to get YOLO outputs meshing with SLAM
  - This could be an issue because YOLO will label every single time the object is seen but we want 1 location per object
  - Need to somehow merge them
  - Look for papers using YOLO with slam and see if they have solutions
  - [This uses YOLO with ORB-SLAM2 for a new SLAM algorithm](https://link.springer.com/article/10.1007/s00521-021-06764-3)



## YOLO ROS: Real-Time Object Detection for ROS ([repo](https://github.com/leggedrobotics/darknet_ros/tree/1.1.5))

### How to train YOLO

Login Credentials for our Google and Roboflow accounts can be found [here](https://github.com/Capstone-W3/Trash-Access-Tokens)

https://colab.research.google.com/drive/1DRbdWh7pWrKC28hN1RmgkOhKlOxhPm9X#scrollTo=NjKzw2TvZrOQ

Using Roboflow for image storage and annotations. Starting with images from [UAVVaste fork](https://github.com/Capstone-W3/UAVVaste) (which was forked from [original](https://github.com/UAVVaste/UAVVaste), and uploading to Roboflow project [TRASH+UAVVaste](https://app.roboflow.com/trash-northeastern/trash-uavvaste). More images can be uploaded and added for our specific use. A Google-Colab Jupyter Notebook ([original](https://colab.research.google.com/drive/1mzL6WyY9BRx4xX476eQdhKDnd_eixBlG) and [ours](https://colab.research.google.com/drive/1kx2XSdisVOBt_QT3J2TVJZ3M8XsQMy_k)) is being used for the actual training of the model to use free GPUs for training that doesn't take days.

For deeper information check out the actual [Darknet](https://github.com/AlexeyAB/darknet) repo and its instructions on how to [train to detect custom objects](https://github.com/AlexeyAB/darknet#how-to-train-to-detect-your-custom-objects)

Following Roboflow general instructions for [Training YOLOv4 on a Custom Dataset](https://blog.roboflow.com/training-yolov4-on-a-custom-dataset/), but due to our limited processing power using [YOLOv4-tiny instructions](https://blog.roboflow.com/train-yolov4-tiny-on-custom-data-lighting-fast-detection/)

We have already modified the Jupyter Notebook [here](https://colab.research.google.com/drive/1kx2XSdisVOBt_QT3J2TVJZ3M8XsQMy_k) 

Changes:
- In Roboflow go to [Versions](https://app.roboflow.com/trash-northeastern/trash-uavvaste/1), Export the Download Code for "YOLO Darknet" and paste it into the Jupyter notebook [here](https://colab.research.google.com/drive/1kx2XSdisVOBt_QT3J2TVJZ3M8XsQMy_k#scrollTo=Cdj4tmT5Cmdl&line=1&uniqifier=1)
- In the [cell for obj.data](https://colab.research.google.com/drive/1kx2XSdisVOBt_QT3J2TVJZ3M8XsQMy_k#scrollTo=KiCILEbs1NII&line=4&uniqifier=1), change the line with `out.write(classes = ` to the number of classes you are training for. For us this is probably just 1 (`rubbish`)
- In the [cell for custom-yolov4-tiny-detector.cfg](https://colab.research.google.com/drive/1kx2XSdisVOBt_QT3J2TVJZ3M8XsQMy_k#scrollTo=U_WJcqHhpeVr&line=5&uniqifier=1), change the line `max_batches` to (`classes*2000` but not less than number of training images, and not less than `6000`), f.e. `max_batches=6000` if you train for `3` classes

Then should be good to run each cell in the notebook to train, this will take a while

After this finishes there should files `custom-yolov4-tiny-detector_best.weights`, `custom-yolov4-tiny-detector_final.weights`, and `custom-yolov4-tiny-detector_last.weights`; save the `_best.weights` to be used when running Darkent with ROS.


### How to use trained YOLO model in ROS

````

cd catkin_workspace/src
git checkout 1.1.5
catkin build
````

Getting what I have found to be a segmentation fault as soon as first image is read in [here](https://answers.ros.org/question/300753/how-to-read-ros-log-file/) and [here](https://github.com/leggedrobotics/darknet_ros/issues/205)

````
[darknet_ros-1] process has died [pid 2876, exit code -11, cmd /home/mllax8/trash_parent_repo/catkin_ws/devel/lib/darknet_ros/darknet_ros camera/rgb/image_raw:=/ardrone/front/image_raw __name:=darknet_ros __log:=/home/mllax8/.ros/log/33c5db96-9b65-11ec-b94b-000c29e37f7d/darknet_ros-1.log].
log file: /home/mllax8/.ros/log/33c5db96-9b65-11ec-b94b-000c29e37f7d/darknet_ros-1*.log
````

https://github.com/leggedrobotics/darknet_ros/issues/93#issuecomment-520524269


#### YOLO with SLAM

https://link.springer.com/article/10.1007/s00521-021-06764-3






&nbsp;

&nbsp;

&nbsp;


## Contact

For any questions, reach out either to the group using our [team email](mailto:TrashTeamNEU@gmail.com) or [Jack's personal email](mailto:TrashTeamNEU@gmail.com)

