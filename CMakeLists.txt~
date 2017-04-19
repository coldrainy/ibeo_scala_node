cmake_minimum_required(VERSION 2.8.3)
project(ibeo_scala_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
	include/ibeo_scala_node
	/home/guolindong/Downloads/IbeoSDK5.0.4/src
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ibeo_scala_node.cpp
# )

add_executable(${PROJECT_NAME} src/ibeo_scala_node.cpp
															 include/ibeo_scala_node/ibeo_scala_node.h)

link_directories(
    ${catkin_LIB_DIRS}  /home/guolindong/Downloads/IbeoSDK5.0.4/lib/)

target_link_libraries(ibeo_scala_node 
	${catkin_LIBRARIES} libibeosdk_d.so)


