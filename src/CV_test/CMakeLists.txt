# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(ImageShow)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)




# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_BUILD_TYPE Debug)

# Declare the executable target built from your sources
add_executable(ImageShow main.cpp)
add_executable(Test cv_split.cpp)
add_executable(Test02 testsplitmax.cpp)
add_executable(SIMD simd.cpp)
add_executable(SIMD02 simd02.cpp)





# Link your application with OpenCV libraries
target_link_libraries(ImageShow PRIVATE ${OpenCV_LIBS})
target_link_libraries(Test PRIVATE ${OpenCV_LIBS})
target_link_libraries(Test02 PRIVATE ${OpenCV_LIBS})
target_link_libraries(SIMD PRIVATE ${OpenCV_LIBS})
target_link_libraries(SIMD02 PRIVATE ${OpenCV_LIBS})


#ysq_cpp
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message("OpenMP found.")
    target_link_libraries(Test02 PUBLIC OpenMP::OpenMP_CXX)
endif()