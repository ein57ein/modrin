cmake_minimum_required(VERSION 2.8.3)
project(epos2_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs bondcpp tf message_generation)
#move_base_msgs

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#set the Enviroment for epos2 lib
set(LD_LIBRARY_PATH /opt/lib:${PROJECT_SOURCE_DIR}:$LD_LIBRARY_PATH)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   velocity.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
#   DEPENDENCIES
#   geometry_msgs#   move_base_msgs#   nav_msgs#   sensor_msgs#   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include 
#  LIBRARIES epos2_control
   CATKIN_DEPENDS actionlib geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf bondcpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(epos2_control
#   src/${PROJECT_NAME}/epos2_control.cpp
# )

## Declare a cpp executable
# add_executable(epos2_control_node src/epos2_control_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(epos2_control_node epos2_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(epos2_control_node
#   ${catkin_LIBRARIES}
# )

add_executable(epos2_talker src/epos2_talker.cpp)
add_executable(epos2_driver src/epos2_driver.cpp)
add_executable(epos2_stateOutput src/epos2_stateOutput.cpp)

#include the MotorController-class for the driver
add_library(Epos2MotorController src/Epos2MotorController.cpp)
add_library(TankSteering src/TankSteering.cpp)
add_dependencies(TankSteering ${PROJECT_NAME}_gencpp)
target_link_libraries(Epos2MotorController /opt/lib/libEposCmd.so)
target_link_libraries(epos2_driver Epos2MotorController)
target_link_libraries(epos2_driver TankSteering)
#linking the ros-Libaries
target_link_libraries(epos2_driver ${catkin_LIBRARIES})

#include the SampleMoving-class for the talker
add_library(SampleMoving src/SampleMoving.cpp)
add_dependencies(SampleMoving ${PROJECT_NAME}_gencpp)
target_link_libraries(epos2_talker SampleMoving)
target_link_libraries(epos2_talker ${catkin_LIBRARIES})

#include the stateOutput
#add_dependencies(epos2_stateOutput ${PROJECT_NAME}_gencpp)
target_link_libraries(epos2_stateOutput ${catkin_LIBRARIES})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS epos2_control epos2_control_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_epos2_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)