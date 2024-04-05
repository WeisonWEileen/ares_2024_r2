# ros2 launch rc_serial_driver serial_driver.launch.py
# ros2 launch armor_detector detector.launch.py
# ros2 run v4l2_camera v4l2_camera_node 
# # ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/cam0
# ros2 run rqt_image_view rqt_image_view
# ros2 launch yolov8_bringup yolov8.launch.py





# #查看摄像头信息
# #查看长宽是640/480
# v4l2-ctl -d  /dev/video0 --all



# cd ..

cmds=(
    "ros2 run v4l2_camera v4l2_camera_node"
	"ros2 launch yolov8_bringup yolov8.launch.py"
    "ros2 topic echo /yolo/tracking"
)

for cmd in "${cmds[@]}"
do 
    echo Current CMD : "$cmd"
    gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
    sleep 0.2
done
