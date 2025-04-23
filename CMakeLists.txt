cmake_minimum_required(VERSION 3.0.2)
project(uart_com)

link_directories(/opt/ros/noetic/lib)
# set(TENSORRT_YOLO_DIR /home/rts/catkin_ws/devel/share/tensorrt_yolo/cmake)
find_package(tf2_ros REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2_ros
  message_generation
  custom_msgs
  # tensorrt_yolo
  # livox_ros_driver2
)
find_package(Boost REQUIRED COMPONENTS 
  system 
  thread
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uart_com
  CATKIN_DEPENDS roscpp rospy serial std_msgs message_runtime
)

include_directories(
    ${catkin_INCLUDE_DIRS}  
    ${CMAKE_SOURCE_DIR}
    include
)


add_executable(send
  src/ros_32.cpp
  src/send.cpp
)

target_link_libraries(send
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  tf2
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
# cmake_minimum_required(VERSION 3.0.2)
# project(uart_com)

# # 1. 初始化 catkin 并声明所有 ROS 依赖
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   rospy
#   std_msgs
#   geometry_msgs
#   tf2_ros
#   message_generation
#   custom_msgs
#   # tensorrt_yolo
# )

# # 2. 其他非 ROS 依赖
# find_package(Boost REQUIRED COMPONENTS system thread)

# # 3. 导出包信息
# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES uart_com
#   CATKIN_DEPENDS 
#     roscpp 
#     rospy 
#     std_msgs 
#     geometry_msgs 
#     tf2_ros 
#     message_generation 
#     custom_msgs 
#     # tensorrt_yolo
# )

# # 4. 包含目录
# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )

# # 5. 生成可执行文件
# add_executable(send
#   src/ros_32.cpp
#   src/send.cpp
# )

# # 6. 链接库
# target_link_libraries(send
#   ${catkin_LIBRARIES}  # 包含 tensorrt_yolo
#   ${Boost_LIBRARIES}
#   tf2_ros
# )