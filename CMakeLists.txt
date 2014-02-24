cmake_minimum_required(VERSION 2.8.3)
project(scr_proto)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp
                  tf
                  rospy
                  std_msgs
                  genmsg
                  message_generation
                  geometry_msgs
                  april_tags
                )

find_package(eigen REQUIRED)

## System dependencies are found with CMake's conventions


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
    DiffCommand.msg
    SpeedCommand.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
#   FILES AddTwoInts.srv
# )

# Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   AprilTagAction.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES april_tags_lib
  CATKIN_DEPENDS roscpp rospy rosserial_arduino april_tags std_msgs message_runtime
  DEPENDS eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

#set(SER_LIB /usr/lib/libserial.so.0.0.0)

## Declare a cpp library
#add_library(serial_interface src/serial_interface.cpp)
#target_link_libraries(serial_interface ${catkin_LIBRARIES} ${SER_LIB})


## Declare a cpp executable
add_executable(odom_test src/Odom_Tester.cpp)
target_link_libraries(odom_test ${catkin_LIBRARIES})

add_executable(odom_broadcaster_node src/OdometryBroadcaster.cpp)
target_link_libraries(odom_broadcaster_node ${catkin_LIBRARIES})

add_executable(april_tags_processor src/AprilTagsProcessor.cpp)
target_link_libraries(april_tags_processor ${catkin_LIBRARIES})

## Declare a cpp executable

#add_executable(motor_node src/motor_node.cpp)
#target_link_libraries(motor_node serial_interface ${catkin_LIBRARIES} ${SER_LIB})
#add_dependencies(motor_node scr_proto_generate_messages_cpp)

add_executable(speed_controller_node src/speed_controller_node.cpp)
target_link_libraries(speed_controller_node ${catkin_LIBRARIES})
add_dependencies(speed_controller_node scr_proto_generate_messages_cpp)

add_executable(base_controller_node src/base_controller_node.cpp)
target_link_libraries(base_controller_node ${catkin_LIBRARIES})
add_dependencies(base_controller_node scr_proto_generate_messages_cpp)

add_executable(remote_control_node src/remote_control_node.cpp)
target_link_libraries(remote_control_node ${catkin_LIBRARIES})


##add_executable(nav_goals_node src/nav_goals_node.cpp)
#target_link_libraries(nav_goals_node ${catkin_LIBRARIES})


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(beginner_tutorials_node beginner_tutorials_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(beginner_tutorials_node
#   ${catkin_LIBRARIES}
# )

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
# install(TARGETS beginner_tutorials beginner_tutorials_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_beginner_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
