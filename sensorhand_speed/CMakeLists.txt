cmake_minimum_required(VERSION 2.8.3)
project(sensorhand_speed)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cereal_port
  message_generation
)

## Generate messages in the 'msg' folder
 add_message_files(FILES
   hand_dump.msg
 )

## Generate services in the 'srv' folder
add_service_files(FILES
   hand_cmd.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hex_serial
  CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(hand_node src/hand_node.cpp)

add_dependencies(hand_node ${PROJECT_NAME}_gencpp)

target_link_libraries(hand_node
  cereal_port
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS sensorhand_speed sensorhand_speed_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sensorhand_speed.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
