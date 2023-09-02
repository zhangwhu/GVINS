cmake_minimum_required(VERSION 3.0.2)
project(data_to_rosbag)
set(CMAKE_BUILD_TYPE "Debug")

add_definitions(-DENAGLO -DENACMP -DENAGAL -DTRACE -D_DEBUG -D_CRT_SECURE_NO_DEPRECATE)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  cv_bridge
  gnss_comm
)

find_package(OpenCV 4.0 REQUIRED)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /media/zhuhang/D/mGNSS/include/
)

add_executable(data2bag src/data2bag.cc)
target_link_libraries(data2bag ${catkin_LIBRARIES} ${OpenCV_LIBS} /media/zhuhang/D/mGNSS/lib/libRTKLIB.so)

add_executable(receive src/receive.cc)
target_link_libraries(receive ${catkin_LIBRARIES} ${OpenCV_LIBS} /media/zhuhang/D/mGNSS/lib/libRTKLIB.so)
