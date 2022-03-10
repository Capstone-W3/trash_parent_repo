source /opt/ros/kinetic/setup.bash
rosbag record /camera/color/image_rect_color /camera/color/image_raw /camera/depth/image_rect_raw /camera/depth/camera_info /odom -O imgsOdomDepth.bag

