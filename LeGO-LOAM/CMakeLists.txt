cmake_minimum_required(VERSION 2.8.3)
project(lego_loam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cloud_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cloud_msgs
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(featureAssociation src/featureAssociation.cpp)
add_dependencies(featureAssociation ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(featureAssociation ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(mapOptmization src/mapOptmization.cpp)
target_link_libraries(mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(transformFusion src/transformFusion.cpp)
target_link_libraries(transformFusion ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(pointPublish src/pcd2bag.cpp)
target_link_libraries(pointPublish ${catkin_LIBRARIES} ${PCL_LIBRARIES}  ${OpenCV_LIBRARIES})

add_executable(getOdometry
  src/getOdometry.cpp
)

add_dependencies(getOdometry ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(getOdometry
  ${catkin_LIBRARIES}
)

#add_library(zheda_sac_model_registration src/zheda_sac_model_registration.cpp)
#target_link_libraries(zheda_sac_model_registration
 # ${catkin_LIBRARIES}
  #${PCL_LIBRARIES}
#)


add_library(zhedaransac src/zhedaransac.cpp)
target_link_libraries(zhedaransac
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
