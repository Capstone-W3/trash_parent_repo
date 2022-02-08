
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

Then install version 3.3.9 of Eigen3 because it doesn't recognize the base version
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
mkdir build && cd build
cmake .. -DEigen3_DIR=$HOME/eigen-3.3.9/build
cmake --build .

# GIVEME THE PYTHON STUFF!!!! (Check the output to verify selected python version)
cmake --build . -t pypangolin_pip_install

# Run me some tests! (Requires Catch2 which must be manually installed on Ubuntu.)
ctest
````

###### OpenCV

https://docs.opencv.org/4.x/d0/d3d/tutorial_general_install.html

````
git clone https://github.com/opencv/opencv
git -C opencv checkout 3.4









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

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

`sudo apt-get install ros-kinetic-desktop-full`

`sudo apt-get install ros-kinetic-catkin python-catkin-tools`

`sudo apt-get install ros-kinetic-catkin python-controller-manager`

`sudo apt-get install ros-kinetic-pid`

### For only robot example

`sudo apt-get install ros-kinetic-move-base-msgs`

`sudo apt-get install ros-kinetic-octomap-rviz-plugins`

`sudo apt-get install ros-kinetic-diff-drive-controller`

````
git clone https://github.com/WiringPi/WiringPi
cd WiringPi
./build
````

