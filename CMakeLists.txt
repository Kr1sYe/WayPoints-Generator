cmake_minimum_required(VERSION 2.8.3)
project(coveragepaintingpathplanning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  interactive_markers
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  octomap_msgs
	rospy
	std_msgs
	rostime 
	sensor_msgs 
  message_filters 
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(OCTOMAP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(PCL REQUIRED)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
    octomap_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR} 
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include 

)

# the below cpp document is for coverage painting paths planning
add_executable(paintingpaths_planning_test src/paintingpaths_planning_test.cpp)
target_link_libraries(paintingpaths_planning_test ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(paintingpaths_planning src/paintingpaths_planning.cpp)
target_link_libraries(paintingpaths_planning ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(paintingpaths_planning_multi_thread src/paintingpaths_planning_multi_thread.cpp)
target_link_libraries(paintingpaths_planning_multi_thread ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)

add_executable(paintingpaths_visualization src/paintingpaths_visualization.cpp)
target_link_libraries(paintingpaths_visualization ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} ${PCL_LIBRARIES} ${Boost_INCLUDE_DIR} jsoncpp)