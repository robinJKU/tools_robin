cmake_minimum_required(VERSION 2.8.3)
project(video_player)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  gscam
)

##  services in the 'srv' folder
add_service_files(
   FILES
   PlayVideoSrv.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES gscam
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation gscam
#  DEPENDS system_lib
)

## Declare a cpp executable
add_executable(video_player_node src/video_player_node.cpp)
target_link_libraries(
	video_player_node
	${catkin_LIBRARIES}
	${GSTREAMER_LIBRARIES}
	${GST_APP_LIBRARIES}
)
add_dependencies(video_player_node  ${catkin_EXPORTED_TARGETS})

add_executable(test_client src/test_client.cpp)
target_link_libraries(
	test_client
	${catkin_LIBRARIES}
	${GSTREAMER_LIBRARIES}
	${GST_APP_LIBRARIES}
)
add_dependencies(test_client  ${catkin_EXPORTED_TARGETS})

## build talker and listener
include_directories(
	${catkin_INCLUDE_DIRS}
	${GLIB_INCLUDE_DIRS}
    	${GST_APP_INCLUDE_DIRS}
)
