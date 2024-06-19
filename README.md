### 第一步，以对其深度图的方式启动realsense D435
'''
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
'''
### 第二步，启动 yolov8 启动节点
'''
ros2 run armor_detector rc_armor_detector_node 
'''
## 第三步，启动投影节点
'''
ros2 run armor_detector rc_armor_projector_node
'''