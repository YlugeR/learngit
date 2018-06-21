cmake_minimum_required( VERSION 2.8.3 )
project(label_data_serilize)

SET( CMAKE_BUILD_TYPE "Release" )

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
		roscpp
		rospy
		pcl_ros
		cv_bridge
		)

#find_package(Eigen3 REQUIRED)
find_package( PCL REQUIRED )
find_package( OpenCV REQUIRED )

link_directories(lib)

include_directories(
		include
		${catkin_INCLUDE_DIRS}
		${PCL_INCLUDE_DIRS})

catkin_package(
		DEPENDS EIGEN3 PCL OpenCV
		INCLUDE_DIRS include
		LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
		src/dirManager.cpp
		src/parse.cpp
		src/seg_extract_based_box.cpp
		src/labelDataManager.cpp
		src/robo_calcfeature.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} common module_manager robosense tracking classification detection preprocessing)

add_executable(serilize_demo
                demo/serilize.cpp)
add_dependencies(serilize_demo ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME})
target_link_libraries(serilize_demo common module_manager robosense tracking classification detection preprocessing ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PROJECT_NAME})



