# Terrestrial Roving Autonomous Scrap Harvester (TRASH)

Welcome to the home repository for Northeastern's EECE Capstone team named TRASH. The TRASH system is a robotics project designed to map out and pick up trash in a particular area. This file serves as a full guide to getting our codebase set up on your own machine.

![Image](TRASH_SystemMockup.png)

# README Outline

1. [INTRODUCTION](#1-introduction)
2. [EXPECTED KNOWLDGE](#2-expected-knowldege)
    - Ubuntu 16.04
    - ROS (Kintetic)
3. [Project Setup Instructions](#3-setup-instructions)
    - Initial Downloads
      1. [Clone This Repository](#clone-this-repository)
      2. [General Dependencies](#general-dependencies)
    - [Full Instructions](#full-instructions)
      1. [Complete Project](#)
      2. [Just ORB-SLAM2](#)
      3. [Just YOLOv4](#)
    - [Speed Setup (Just Copy & Paste)](#speed-setup)
4. [Project Run Instructions](#)
    - ORB-SLAM2
    - YOLOv4
    - Full Project

5. [Contact Us](#contact)



# 1. Introduction

TRASH was designed as an application of robotics to the ever-present problem of trash collection. We focused our approach on litter pickup, particularly in spaces like highway shoulders and parking lots. Our approach focused on creating a modular solution that was not dependent on a particular implementation. We first map a space and identify the trash and then pick it up, which could be done by a heterogeneous two-robot system (like a mapping drone and a ground collection robot). However, since this is just a proof of concept we are only using a single robot. We use the TurtleBot2, which is essentially a Roomba for research, and custom designed a collection mechanism fastened to the front.

Take a look at the [Final Project Document](Capstone_Project_Writeup.pdf) for a full understanding of the scope, goals, approach taken, and results of this project.



### Software Overview

Each one of the software packages that make up different parts of the project are connected using ROS. Robot Operating System (ROS) is an open-source robotics middleware suite that handles communication between "nodes". Each node is able to publish data of different types that any other node in the network can subscribe to.

The TRASH system’s software has three distinct stages (as seen below).

In the first stage, the TRASH system maps its environment using a customized version of ORB-SLAM2, an open source Visual SLAM solution which can accurately create point clouds given RGB and depth camera input. We trained YOLO, an image identification CNN, with our own dataset, then using it to identify and mark the locations of trash clusters on the map. We then render the 3D point cloud down to a 2D occupancy grid using a custom implementation of Octomap. It is from this 2D occupancy grid that the robot can navigate around an environment.

Once the mapping stage is over, the TRASH system enters its second stage: general navigation.The Turtlebot uses adaptive Monte Carlo localization (AMCL) to locate itself in the built map. That way it can identify obstacles and path plan around them to the trash cluster points labeled in that map.
Upon reaching a cluster, the third stage begins. YOLO identifies trash in the RGB image from the Realsense Camera, and from the coordinates of the trash detection and the distance measurement received from the depth camera, calculates the position of the trash relative to the robot. Once it identifies the relative position of the trash, the robot turns towards its target, starts the collection mechanism motor, and moves towards it, picking up the trash piece. Upon successful collection, it returns to the general navigation stage, repeating on until all trash clusters have been visited.

![Image](TRASH_SoftwareOverview.png)
Each ROS node in our network is represented by their own bubble


## 2. Expected Knowldege


This project uses a number of advanced technical solutions, but most of these do not need to be understood at least while setting up the code base.

1.  This project must be run on the Linux operating system Ubuntu (specifically 16.04). This project expects that you have some familiarity with Unix operating systems. If you do not already have Ubuntu 16.04 set up, you will need to create a virtual machine. I suggest [VMware Fusion](https://www.vmware.com/products/fusion.html) to run your [Ubuntu image](https://releases.ubuntu.com/16.04/). For more expansive directions look [here](https://graspingtech.com/vmware-fusion-ubuntu-18.04/). **You MUST use Ubuntu 16.04**

2.  Ubuntu 16.04 is end-of-life and no longer supported but any recent software updates, but our robot was unable to use anything newer. Most of the issues we ran into on the software front of this project stem from this fact. We were unable to update ROS beyond ros-kinetic and Python3 beyong Python3.5, which each cause a number of issues in themselves. 

3.  ROS is essential to how the components of this project fit together. If you have never heard of or used ROS before, it may be worth reading up about it or doing some of the tutorials found on the [ROS website](http://wiki.ros.org/ROS/Introduction). However you must use [these instructions] to install ros-kinetic; DO NOT install ROS using the instructions from their website.

4.  As is already evidenced by the format of this document, Github is an essential tool in setting up this project. We use git submodules, which allow for multiple git repositories to be linked whtin a single one. We use this parent repository to automatically structure all the dependent repositories for you. Look [here](https://git-scm.com/book/en/v2/Git-Tools-Submodules) to learn more about submodules.

5.  Though not essential to getting the codebase set up, knowledge of robotics techniques SLAM and Computer Vision would be very helpful understanding what is being done in this project. To learn more about these respective topics you can read their papers: [ORB-SLAM2](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf) and [YOLO](https://arxiv.org/pdf/1506.02640.pdf) (or [YOLOv4](https://arxiv.org/pdf/2004.10934.pdf)).

6.  All code snippets shown in this guide (`visible here`) are meant to be run in a command-line window



---


## 3. Setup Instructions

After installing Ubuntu 16.04 as discussed in [2](#2-expected-knowldege), continue by following the instructions from each section in this guide

For the ease of consistency across each install of this project, we will assume that it is being installed directly in the home directory. If you are not doing this, you will need to fix the directory of installs whereever an absolute path is given (if you do not know what this means, don't worry and just run as instructed).

**Setup Instructions Outline**

1. [Clone This Repository](#clone-this-repository)
2. [General Dependencies](#general-dependencies)
    1. [pip3](#pip3)
    2.  [ROS](#ros)

## Clone This Repository

Open a command line window and then

`git clone --recurse-submodules --remote-submodules https://github.com/Capstone-W3/trash_parent_repo.git`

OR if git version is less than 2.23

`git clone --recurse-submodules https://github.com/Capstone-W3/trash_parent_repo.git`

Then need to go through and checkout to the remote tracking branch in each submodule

`git submodule update --recursive --remote`



## General Dependencies

### pip3

Open a new command line window and then

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


Open a new command line window

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



# ORB-SLAM2

### ORB-SLAM2 Outline

- [Dependency Installs](#orb-slam2-dependencies)
    1. [CMAKE](#cmake)
    2. [Eigen3](#eigen3)
    3. [Pangolin](#pangolin)
    4. [OpenCV](#opencv)
- [Building ORB-SLAM2](#ORB-SLAM2 Iteself)

If desired you can jump directly to [running ORB-SLAM2](running-orb-slam2)

## ORB-SLAM2 Dependencies

### CMake

Requires updated version of CMake, but many way of doing this update will break a pre-installed ros implementation


Open a new command line window (you do not want to use a window that has already sourced ROS)

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

### Eigen3

Install version 3.3.9 of Eigen3 because it doesn't recognize the base version, [as seen here](https://apolo-docs.readthedocs.io/en/latest/software/scientific_libraries/eigen/eigen-3.3.7/index.html)


Open a new command line window

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

### Pangolin


Open a new command line window

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

### OpenCV


Open a new command line window

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





## ORB-SLAM2 Itself
###### using [instal instructions](#https://github.com/Capstone-W3/ORB-SLAM2_ROS/tree/no_loop_close#3-installation-example) from [source repository](https://github.com/Capstone-W3/ORB-SLAM2_ROS/tree/no_loop_close)

````
cd ~/trash-parent-repo/catkin_ws/src
cd ORB-SLAM2_ROS
git checkout no_loop_close
````

Create execution privileges for all installation scripts:
````
cd ORB-SLAM2_ROS/ORB_SLAM2
sudo chmod +x build*
````

Then run the main build script:
````
./build_catkin.sh
````

Then either [install YOLO](#) or [run ORB-SLAM2](#)





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

