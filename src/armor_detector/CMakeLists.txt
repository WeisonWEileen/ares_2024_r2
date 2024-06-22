cmake_minimum_required(VERSION 3.10)

option(USE_ONNX "Enable ONNX support" OFF)
option(USE_TENSORRT "Enable TensorRT support" ON)



if(USE_TENSORRT)
    set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
endif()

project(armor_detector)

# control whether to use onnx or tensorrt accelrate must be seletected
if(USE_ONNX)
  add_definitions(-DONNX)
  message(STATUS "USE ONNX")


  # CUDA PATH
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
  # find_package(OpenMP REQUIRED)
  ament_auto_find_build_dependencies()

  ###########
  ## Build ##
  ###########

  ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
  )

  target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


elseif(USE_TENSORRT)
  message(STATUS "USE TENSORRT")

  add_definitions(-DTENSORRT)

  # 记得选择CUDA架构，tensorrt必备
  option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

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
  find_package(CUDA REQUIRED)
  get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
  # find_package(OpenMP REQUIRED)
  ament_auto_find_build_dependencies()

  #  TensorRT
  set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
  set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)



  ###########
  ## Build ##
  ###########

  ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
  )

  target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS} ${TensorRT_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${TensorRT_LIBRARIES} ${CUDA_LIB_DIR})
endif()


# ADD EXECUTABLE NODES
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc_auto_aim::InferencerNode
  EXECUTABLE rc_armor_detector_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc_auto_aim::ProjectorNode
  EXECUTABLE rc_armor_projector_node
)

# Shared Objects to be called
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  model
)