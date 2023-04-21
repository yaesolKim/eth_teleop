cmake_minimum_required(VERSION 3.0.2)
project(eth)

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
    actionlib_msgs
    control_msgs
    dynamic_reconfigure
    franka_msgs
    roscpp
    rospy
    message_generation
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)


generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs trajectory_msgs control_msgs  # Or other packages containing msgs
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

add_executable(tutorial src/tutorial.cpp)
target_link_libraries(tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(haptic src/haptic.cpp)
target_link_libraries(haptic ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(home src/home.cpp)
target_link_libraries(home ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(error_recovery_node src/error_recovery_node.cpp)
target_link_libraries(error_recovery_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Specify additional locations of header files Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(
  PROGRAMS script/haptic.py script/haptic_handle.py script/haptic_plain.py script/Ks_eth.py script/Kd_eth.py script/telemanipulation.py script/test_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

