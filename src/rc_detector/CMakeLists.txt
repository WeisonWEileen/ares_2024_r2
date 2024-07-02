cmake_minimum_required(VERSION 3.10)

set(CMAKE_SYSTEM_PROCESSOR "x86_64")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message(STATUS "Onnx  Host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
  option(USE_ONNX "Enable ONNX support" ON)
  option(USE_TENSORRT "Enable TensorRT support" OFF)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message(STATUS "Tensorrt Host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
  option(USE_TENSORRT "Enable TensorRT support" OFF)
  option(USE_TENSORRT "Enable TensorRT support" ON)
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()


if(USE_TENSORRT)
    set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
endif()

project(rc_detector)

# control whether to use onnx or tensorrt accelrate must be seletected
if(USE_ONNX)
  add_definitions(-DONNX)
  message(STATUS "USE ONNX")

  # CUDA
  set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
  find_package(CUDA 11 REQUIRED)
  set(CMAKE_CUDA_STANDARD 11)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)

  ## Use C++14
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  ## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
  ## enforcing cleaner code.
  add_definitions(-Wall -Werror)

  ## Export compile commands for clangd
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  SET(CMAKE_BUILD_TYPE "Debug")
  SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
  SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

  #######################
  ## Find dependencies ##
  #######################

  find_package(ament_cmake_auto REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  # find_package(OpenMP REQUIRED)
  ament_auto_find_build_dependencies()

  ###########
  ## Build ##
  ###########

  ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
  )

  target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


elseif(USE_TENSORRT)
  message(STATUS "USE TENSORRT")

  add_definitions(-DTENSORRT)

  # 记得选择CUDA架构，tensorrt必备
    ## Use C++14
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

  ## Export compile commands for clangd
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

  #######################
  ## Find dependencies ##
  #######################

  find_package(ament_cmake_auto REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(CUDA REQUIRED)
  find_package(Eigen3 REQUIRED)
  get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
  # find_package(OpenMP REQUIRED)
  ament_auto_find_build_dependencies()

  #  TensorRT
  set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
  set(TensorRT_LIB_DIR  /usr/lib/aarch64-linux-gnu)

  ###########
  ## Build ##
  ###########

  ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
  )

  target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${TensorRT_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
  target_link_directories(${PROJECT_NAME} PUBLIC ${CUDA_LIB_DIR} ${TensorRT_LIB_DIR})
  target_link_libraries(${PROJECT_NAME} nvinfer nvinfer_plugin ${OpenCV_LIBRARIES} ${TensorRT_LIBRARIES} ${CUDA_LIBRARIES})
endif()


# ADD EXECUTABLE NODES
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc_detector::InferencerNode
  EXECUTABLE rc_detector_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc_detector::ProjectorNode
  EXECUTABLE rc_projector_node
)

add_executable(rc_carry_state_node src/carry_state_node.cpp)
ament_target_dependencies(rc_carry_state_node
  rclcpp
  sensor_msgs
  OpenCV
  cv_bridge
)

install(TARGETS
  rc_carry_state_node
  DESTINATION lib/${PROJECT_NAME}
)

# Shared Objects to be called
ament_auto_package(
  INSTALL_TO_SHARE
  include
  launch
  model
  config
)