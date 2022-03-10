https://github.com/appliedAI-Initiative/orb_slam_2_ros

### For Jared and Katie
- [Instructions](#how-to-train-yolo)

# Google drive with rosbags and training data
https://drive.google.com/drive/folders/1bLo9Ca3X3cnFm1wuKJpPVZGTskb2vgef?usp=sharing

# TODO


https://gist.github.com/rethink-imcmahon/77a1a4d5506258f3dc1f?permalink_comment_id=3577587#gistcomment-3577587

https://github.com/leggedrobotics/darknet_ros/issues/205

https://drive.google.com/drive/folders/1bLo9Ca3X3cnFm1wuKJpPVZGTskb2vgef

http://tacodataset.org/

https://github.com/IeiuniumLux/Visual-SLAM

https://answers.ros.org/question/355158/roslaunch-dies-with-exit-code-11-after-it-stops-receiving-topic-from-bag-file/

https://answers.ros.org/question/188823/segmentation-fault/

https://github.com/leggedrobotics/darknet_ros/issues/345

https://drive.google.com/drive/folders/1bLo9Ca3X3cnFm1wuKJpPVZGTskb2vgef?usp=sharing

Setup of sensors
- Install Realsense2 ros packages
  - `sudo apt-get install ros-kinetic-realsense2-camera`
  - This doesn't work out of box, follow instructions for adding udev rules for camera [here](https://github.com/IntelRealSense/realsense-ros/issues/1408#issuecomment-698128999)
- All 3 of our cameras need callibration (drone front and bottom and turtlebot)
  - How to callibrate cameras [using OpenCV](https://docs.opencv.org/3.4/d6/d55/tutorial_table_of_content_calib3d.html), which we already have in the project
  - Need an orbslam camera yaml file for front facing in [this form](https://github.com/Capstone-W3/ORB_SLAM3/blob/v0.3-beta/Examples/Monocular-Inertial/EuRoC.yaml)
   - AR Drone front can be found [here](https://github.com/jwangjie/Mapping-ARDrone/blob/master/orb_slam2/Examples/Monocular/ardrone.yaml)
   - RealSense camera setup file [exists](https://github.com/Capstone-W3/ORB_SLAM3/tree/v1.0-release/Examples/Monocular-Inertial) but in their new format, need to revert to old format
- To use Monocular-Intertial (camera and imu gives better results), need to get the IMU callibrations on both robots
- Turtlebot needs gyro and odometry callibration for navigation [here](http://wiki.ros.org/turtlebot_calibration/Tutorials/Calibrate%20Odometry%20and%20Gyro)

Translate ORB-SLAM output from point clouds to octree with Octomap for path planning
- [ORB_SLAM3_Grid_Mapping](https://github.com/MrPicklesGG/ORB_SLAM3_Grid_Mapping)
  - [adapted from ORB-SLAM2 version repo](https://github.com/abhineet123/ORB_SLAM2)
  - [paper of ORB-SLAM2 version](https://github.com/abhineet123/ORB_SLAM2/blob/master/2d-grid-mapping.pdf)
- Potential Recources
- https://github.com/charbelias24/3dMapSLAM#24-octomap
- https://github.com/OctoMap/octomap_mapping/tree/kinetic-devel 
- https://github.com/OctoMap/octomap_ros/tree/kinetic-devel (the above is probably better than this)


YOLO for our application and SLAM 

- ~~Get YOLO Darknet setup ([repo](https://github.com/UZ-SLAMLab/ORB_SLAM3))~~
  - ~~Should be easy since this version is specifically for Kinetic and Ubunutu 16.04~~
- Run YOLO with UAVVaste dataset ([git-repo](https://github.com/UAVVaste/UAVVaste))
  - This could be super easy but unsure because they almost certainly used a different version of YOLO
  - [How to train to detect custom objects](https://github.com/AlexeyAB/darknet#how-to-train-to-detect-your-custom-objects)
  - 
  - [how-to-improve-object-detection](https://github.com/AlexeyAB/darknet#how-to-improve-object-detection)
  - [Convert COCO style annotations to YOLO format](https://github.com/qwirky-yuzu/COCO-to-YOLO) for images from [UAVVaste](https://github.com/UAVVaste/UAVVaste)

**TRAIN YOLO USING GOOGLE COLAB AND ROBOFLOW FOR DATASET SETUP**

- https://colab.research.google.com/drive/1DRbdWh7pWrKC28hN1RmgkOhKlOxhPm9X#scrollTo=NjKzw2TvZrOQ
- https://blog.roboflow.com/train-yolov4-tiny-on-custom-data-lighting-fast-detection/
- https://app.roboflow.com/jack-fenton/trash-uavvaste/1

- Try to get YOLO outputs meshing with SLAM
  - This could be an issue because YOLO will label every single time the object is seen but we want 1 location per object
  - Need to somehow merge them
  - Look for papers using YOLO with slam and see if they have solutions
  - [This uses YOLO with ORB-SLAM2 for a new SLAM algorithm](https://link.springer.com/article/10.1007/s00521-021-06764-3)

# Clone this repo and its submodules

`git clone --recurse-submodules https://github.com/Capstone-W3/trash_parent_repo.git`

Then need to go through and checkout to the remote tracking branch in each

`git submodule update --recursive --remote`

OR if git version is 2.23 or greater

`git clone --recurse-submodules --remote-submodules https://github.com/Capstone-W3/trash_parent_repo.git`

#### THIS PROJECT ASSUMES WORKING ON UBUNTU 16.04 WITH ROS-KINETIC AND PYTHON3.5, THERE ARE MANY FIXES IMPLEMENTED THIS CONFIGURATION THAT WOULD NOT BE REQUIRED ON OTHER SYSTEMS

## General requirements

### pip3

`sudo apt-get install python3-pip`

DO NOT run the suggested update command, pip > 21.0 does not support python3.5 and will break the entire system. This happened to me and took a long time to fix because as of Jan 2022 it seems to be impossible to upgrade python versions on Ubuntu 16.04 ([issue](https://github.com/deadsnakes/issues/issues/195)). 

To update versions without breaking everything: (actually I think it should be `pip3 install...`)

`pip install -U "pip<21.0" setuptools`

**NOTE:** If you have already broken it like me, you have to install pip from another source, otherwise don't run this command

````
curl -sSL https://bootstrap.pypa.io/pip/3.5/get-pip.py -o get-pip.py
python3 get-pip.py "pip < 21.0"
````

### ROS

Install ros-kinetic

http://wiki.ros.org/kinetic/Installation/Ubuntu

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt install curl` # if you haven't already installed curl

`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`

`sudo apt-get update`

`sudo reboot`

`sudo apt-get install ros-kinetic-desktop-full`

`sudo apt-get install ros-kinetic-catkin python-catkin-tools`

`sudo apt-get install ros-kinetic-catkin python-controller-manager`

`sudo apt-get install ros-kinetic-pid`

**DO NOT** run the suggested command from ros that adds `source /opt/ros/kinetic/setup.bash` to your `~/.bashrc`. This will cause the `ros` version of any package to be the system defaults (opencv will be sourced from the ros version not the one you install)

To check whether this has been done on pre-installed ros, open `~/.bashrc` (`code ~/.bashrc` for VSCode) and remove this line if already there.

For any terminal shell that you wish to run ros commands from you must now run

`source /opt/ros/kinetic/setup.bash` or for this specific project's catkin workspace `source ~/trash_parent_repo/catkin_ws/devel/setup.bash`


# ORB-SLAM3 and its ros wrapper

## ORB-SLAM3

### Dependencies and their dependencies

#### Pangolin

##### CMake

Requires updated version of CMake, but many way of doing this will break a pre-installed ros implementation

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

##### Eigen3

Install version 3.3.9 of Eigen3 because it doesn't recognize the base version https://apolo-docs.readthedocs.io/en/latest/software/scientific_libraries/eigen/eigen-3.3.7/index.html

````
cd $HOME
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -xzvf eigen-3.3.9.tar.gz 
cd eigen-3.3.9

mkdir build && cd build
cmake ..
sudo make install
````

##### Pangolin itself

The checkout to `v0.6` resolves issue [#715](https://github.com/stevenlovegrove/Pangolin/pull/715) from Pangolin. Waiting to see if this can be resolved and no longer needed

The `-DEigen3_DIR` flag must point to where you put eigen3, I set it up so this will default to the home directory but can be changed

~~git clone --recursive https://github.com/stevenlovegrove/Pangolin.git~~
````
cd Pangolin
./scripts/install_prerequisites.sh recommended
git checkout v0.6

mkdir -p build && cd build
cmake .. -DEigen3_DIR=$HOME/eigen-3.3.9/build
cmake --build .
````

#### OpenCV

If OpenCV is only needed for the dependencies from other repos you can just install through `pip3`

`pip3 install opencv-python==3.4.6.27`

If needing to perform camera callibrations or other things specifically within OpenCV, see other OpenCV section towards the bottom of this repo (should link to it)


### ORB_SLAM3 ([repo](https://github.com/UZ-SLAMLab/ORB_SLAM3))

(need to remove all the `-j*` flags in `build.sh` on our forked repo)

We are using `v0.3` due to limitations from the ros wrapper but otherwise we could use `v0.4` (can't remember if v1.0 works but think they phased out Ubuntu 16.04 and ros-kinetic)

~~git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3~~

~~git -C ORB_SLAM3 checkout v0.3~~
````
cd ORB_SLAM3
git checkout trash

chmod +x build.sh
./build.sh
````

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


#### UAVVaste

````
git clone https://github.com/Capstone-W3/UAVVaste
cd UAVVaste
pip3 install -r requirements.txt
python3 main.py
````
This script is written using f-strings (which are not supported in python3.5) so lines 15, 18, and 22 need to be changed to not use this format.

To translate from their COCO-like format to YOLO format 

`python3 coco_to_yolo.py`


&nbsp; 

&nbsp; 

&nbsp; 

&nbsp;

&nbsp;

&nbsp;


## Trying different approach and tracking required steps

### Using regular ORB-SLAM2

#### Dependencies and their dependencies

###### Pangolin

Requires updated version of CMake, but many way of doing this will break a pre-installed ros implementation

From https://askubuntu.com/a/976700. Download cmake (https://cmake.org/download/) I used cmake-3.22.2.tar.gz

````
cd $CMAKE_DOWNLOAD_PATH
./configure
make
sudo make install
echo 'export PATH=$HOME/cmake-install/bin:$PATH' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$HOME/cmake-install:$CMAKE_PREFIX_PATH' >> ~/.bashrc
````
Check that everything worked using `cmake --version`

Should be able to follow these for full command line (but have not tested yet)

````
wget https://cmake.org/files/v3.22/cmake-3.22.2-Linux-x86_64.tar.gz
tar xzf cmake-3.22.2-Linux-x86_64.tar.gz
rm -rf cmake-3.22.2-Linux-x86_64.tar.gz
cd cmake-3.22.3-Linux-x86_64
./configure
make
sudo make install
echo 'export PATH=$HOME/cmake-install/bin:$PATH' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=$HOME/cmake-install:$CMAKE_PREFIX_PATH' >> ~/.bashrc
````

Then install version 3.3.9 of Eigen3 because it doesn't recognize the base version https://apolo-docs.readthedocs.io/en/latest/software/scientific_libraries/eigen/eigen-3.3.7/index.html
````
cd ~
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz
tar -xzvf eigen-3.3.9.tar.gz 
cd eigen-3.3.9

mkdir build && cd build
cmake ..
sudo make install
````


Now can install Pangolin (though locating eigen is not currently working)

````
# Get Pangolin
cd ~/your_fav_code_directory
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin 

# Install dependencies (as described above, or your preferred method)
./scripts/install_prerequisites.sh recommended

# Configure and build
mkdir -p build && cd build
cmake .. -DEigen3_DIR=$HOME/eigen-3.3.9/build
cmake --build .

# GIVEME THE PYTHON STUFF!!!! (Check the output to verify selected python version)
cmake --build . -t pypangolin_pip_install

# Run me some tests! (Requires Catch2 which must be manually installed on Ubuntu.)
ctest
````

Currently trying to resolve issue https://github.com/stevenlovegrove/Pangolin/issues/714

Circumvented problem by changing branches for building (not doing it right away so we can still use the `install_prerequisites.sh` script)

````
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended
git checkout v0.6

mkdir -p build && cd build
cmake .. -DEigen3_DIR=$HOME/eigen-3.3.9/build
cmake --build .
````

###### OpenCV

https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html

https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html

(if computer is capable of handling multi-thread run with something like `sudo make -j4 install`)

This will take 20+ minutes so be ready to let it run

git clone https://github.com/opencv/opencv

git -C opencv checkout 3.4
````
cd opencv
mkdir -p build && cd build
cmake .. -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local
sudo make install
````

make sure that when you `cmake` that you see both versions of python~~

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

#### Build ORB-SLAM2

Inside the catkin workspace

`git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2`

For base ORB-SLAM2 

````
cd ORB_SLAM2
chmod +x build.sh
./build.sh
````

Then for ROS compatability
`./build_ros.sh`

Then to run https://github.com/raulmur/ORB_SLAM2#running-monocular-node

`rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE`

`rosrun ORB_SLAM2 Mono Vocabulary/ORBvoc.txt Examples/Monocular/TUM1.yaml`


##### Issues 

###### For base

If failing due to do `‘usleep’ was not declared`

https://github.com/raulmur/ORB_SLAM2/pull/932 and https://github.com/raulmur/ORB_SLAM2/pull/824

If failing due to internal compiler error

https://github.com/raulmur/ORB_SLAM2/issues/305

Just remove the `-j` options from all `make` instructions in `build.sh` and in `build_ros.sh`

###### For ros compatability

if getting
````
Consolidate compiler generated dependencies of target Stereo
[ 66%] Linking CXX executable ../Stereo
/usr/bin/ld: CMakeFiles/Stereo.dir/src/ros_stereo.cc.o: undefined reference to symbol '_ZN5boost6system15system_categoryEv'
/usr/lib/x86_64-linux-gnu/libboost_system.so: error adding symbols: DSO missing from command line
collect2: error: ld returned 1 exit status
CMakeFiles/Stereo.dir/build.make:229: recipe for target '../Stereo' failed
make[2]: *** [../Stereo] Error 1
CMakeFiles/Makefile2:717: recipe for target 'CMakeFiles/Stereo.dir/all' failed
make[1]: *** [CMakeFiles/Stereo.dir/all] Error 2
Makefile:135: recipe for target 'all' failed
make: *** [all] Error 2
````

https://github.com/raulmur/ORB_SLAM2/issues/494

Final fix: in `ORB_SLAM2/Examples/ROS/ORB_SLAM2/CMakeList.txt` add `-lboost_system` to `set` so that it looks like

````
set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
-lboost_system
)
````



To change image data from compressed to raw https://answers.ros.org/question/35183/compressed-image-to-image/


In pure ORB-SLAM3

`./Monocular/mono_euroc ../Vocabulary/ORBvoc.txt ./Monocular/EuRoC.yaml ~/Downloads/mav0 ./Monocular/EuRoC_TimeStamps/V101.txt dataset-V101_mono`





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

If these last 3 fail, try them in another order

````
roslaunch diff_drive_mapping_robot robot_offline.launch
rosbag play ${PATH TO THE BAG FILE}
roslaunch orb_slam2_ros raspicam_mono_wide.launch
````






# Packaged Needed 

### For all of ORB-SLAM


### For only robot example

`sudo apt-get install ros-kinetic-move-base-msgs`

`sudo apt-get install ros-kinetic-octomap-rviz-plugins`

`sudo apt-get install ros-kinetic-diff-drive-controller`

````
git clone https://github.com/WiringPi/WiringPi
cd WiringPi
./build
````

