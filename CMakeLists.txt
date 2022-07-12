cmake_minimum_required(VERSION 2.8.3)
project(aloam_velodyne)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		nav_msgs
		sensor_msgs
		odom_stream_msgs
		cloud_msgs
  		roscpp
  		rospy
  		rosbag
  		std_msgs
  		image_transport
  		cv_bridge
  		tf)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(fmt)
find_package(yaml-cpp)

Find_Package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
	find_package(cmake_modules REQUIRED)
	find_package(Eigen REQUIRED)
	set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
	set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
else()
	set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs odom_stream_msgs cloud_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)

include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS}
		${CERES_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
		${OpenCV_INCLUDE_DIRS}
)


link_directories(
		${CERES_LIBRARIES}
		include
		${OpenCV_LIBRARY_DIRS}
		${PCL_LIBRARY_DIRS}
		${CERES_INCLUDE_DIRS}
		${EIGEN3_INCLUDE_DIR}
)

add_executable(pcl2RangeImgNpole src/pcl2RangeImgNpole.cpp)
add_dependencies(pcl2RangeImgNpole ${catkin_EXPORTED_TARGETS} odom_stream_msgs_gencpp cloud_msgs_gencpp)
target_link_libraries(pcl2RangeImgNpole ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})




