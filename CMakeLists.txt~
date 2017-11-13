cmake_minimum_required(VERSION 2.8.3)
project(ibeo_scala_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PCL REQUIRED)

find_package(MRPT 1.5 EXACT REQUIRED
        base
        slam
        gui
        opengl
        maps
        obs
        )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
	include/ibeo_scala_node
	/home/guolindong/CyberC3/IbeoSDK5.0.4/src
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ibeo_scala_node.cpp
# )

add_executable(scala_pointcloud
		src/scala_pointcloud.cpp
)

link_directories(
    ${catkin_LIB_DIRS}  
		/home/guolindong/CyberC3/IbeoSDK5.0.4/libs/
		/usr/lib/
)

target_link_libraries(scala_pointcloud
		libibeosdk_d.so
		${catkin_LIBRARIES} 
		${MRPT_LIBS}
		${PCL_LIBRARIES}
)


