# 球部坐标定位
- ```inferencer_node```文件：球部识别的节点文件
- ```inferencer.cpp```文件：onnx加速下的yolov8核心文件
- ```include/yolov8.hpp```文件：TensorRT加速下的yolov8核心文件
---
# 不同平台运行注意事项
- 注意在CMakeLists.txt中指定要```CUDA```架构和加速类型
- jetson上的realsense的话题是namespace是```/camera/camera```,而x86架构上的```/camera```


