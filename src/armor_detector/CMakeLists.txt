cmake_minimum_required(VERSION 3.10)
project(armor_detector)

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


rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc_auto_aim::InferencerNode
  EXECUTABLE rc_armor_detector_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rc_auto_aim::ProjectorNode
  EXECUTABLE rc_armor_projector_node
)



#############
## Testing ##
#############

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   list(APPEND AMENT_LINT_AUTO_EXCLUDE
#     ament_cmake_copyright
#     ament_cmake_uncrustify
#     ament_cmake_cpplint
#   )
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest)
#   ament_add_gtest(test_node_startup test/test_node_startup.cpp)
#   target_link_libraries(test_node_startup ${PROJECT_NAME})

#   ament_add_gtest(test_number_cls test/test_number_cls.cpp)
#   target_link_libraries(test_number_cls ${PROJECT_NAME})

# endif()

#############
## Install ##
#############

#暂时没有模型，先就那样
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  model
)
