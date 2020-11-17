
cmake_minimum_required(VERSION 2.8.3)

project(tf2_pointcloud)

find_package(catkin REQUIRED COMPONENTS roscpp tf2 sensor_msgs pcl_conversions tf2_eigen geometry_msgs tf2_ros)
find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

add_executable(tf2_pointcloud src/tf2_pointcloud.cpp)
target_link_libraries(tf2_pointcloud ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS tf2_pointcloud
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
