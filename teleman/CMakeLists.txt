cmake_minimum_required(VERSION 3.0.2)
project(teleman)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
    std_msgs
    actionlib_msgs
    control_msgs
    dynamic_reconfigure
    roscpp
    rospy
    message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs geometry_msgs # Or other packages containing msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    actionlib_msgs
    control_msgs
    roscpp
    rospy
    std_msgs
    message_runtime

  DEPENDS
    EIGEN3
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# ################################################################################################################################
# Build ##
# ################################################################################################################################
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(telemanipulation src/telemanipulation.cpp)
target_link_libraries(telemanipulation ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(home src/home.cpp)
target_link_libraries(home ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Specify additional locations of header files Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(
  PROGRAMS script/telemanipulation.py script/test_publisher.py script/Ks_teleop.py script/teleop2.py script/Kd_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

