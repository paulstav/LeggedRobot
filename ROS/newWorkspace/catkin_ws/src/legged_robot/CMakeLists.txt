cmake_minimum_required(VERSION 2.8.3)
project(legged_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   PWM.msg
   Encoder.msg
   Position.msg
   AccGyro.msg
   Force.msg
   RobotAngles.msg
   RobotInertiaMeasurements.msg
   RobotForces.msg
   LegCommand.msg
   LegEncoders.msg
   LegPosition.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES legged_robot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(legged_robot
#   src/${PROJECT_NAME}/legged_robot.cpp
# )

add_executable(FL_motor_interface src/FL_motor_interface.cpp)
target_link_libraries(FL_motor_interface ${catkin_LIBRARIES})
add_dependencies(FL_motor_interface legged_robot_generate_messages_cpp)
add_executable(FL_motor_pid src/FL_motor_pid.cpp)
target_link_libraries(FL_motor_pid ${catkin_LIBRARIES})
add_dependencies(FL_motor_pid legged_robot_generate_messages_cpp)
add_executable(FL_leg_control src/FL_leg_control.cpp)
target_link_libraries(FL_leg_control ${catkin_LIBRARIES})
add_dependencies(FL_leg_control legged_robot_generate_messages_cpp)

add_executable(FR_motor_interface src/FR_motor_interface.cpp)
target_link_libraries(FR_motor_interface ${catkin_LIBRARIES})
add_dependencies(FR_motor_interface legged_robot_generate_messages_cpp)
add_executable(FR_motor_pid src/FR_motor_pid.cpp)
target_link_libraries(FR_motor_pid ${catkin_LIBRARIES})
add_dependencies(FR_motor_pid legged_robot_generate_messages_cpp)
add_executable(FR_leg_control src/FR_leg_control.cpp)
target_link_libraries(FR_leg_control ${catkin_LIBRARIES})
add_dependencies(FR_leg_control legged_robot_generate_messages_cpp)

add_executable(BL_motor_interface src/BL_motor_interface.cpp)
target_link_libraries(BL_motor_interface ${catkin_LIBRARIES})
add_dependencies(BL_motor_interface legged_robot_generate_messages_cpp)
add_executable(BL_motor_pid src/BL_motor_pid.cpp)
target_link_libraries(BL_motor_pid ${catkin_LIBRARIES})
add_dependencies(BL_motor_pid legged_robot_generate_messages_cpp)
add_executable(BL_leg_control src/BL_leg_control.cpp)
target_link_libraries(BL_leg_control ${catkin_LIBRARIES})
add_dependencies(BL_leg_control legged_robot_generate_messages_cpp)

add_executable(BR_motor_interface src/BR_motor_interface.cpp)
target_link_libraries(BR_motor_interface ${catkin_LIBRARIES})
add_dependencies(BR_motor_interface legged_robot_generate_messages_cpp)
add_executable(BR_motor_pid src/BR_motor_pid.cpp)
target_link_libraries(BR_motor_pid ${catkin_LIBRARIES})
add_dependencies(BR_motor_pid legged_robot_generate_messages_cpp)
add_executable(BR_leg_control src/BR_leg_control.cpp)
target_link_libraries(BR_leg_control ${catkin_LIBRARIES})
add_dependencies(BR_leg_control legged_robot_generate_messages_cpp)

add_executable(IMU_interface src/IMU_interface.cpp)
target_link_libraries(IMU_interface ${catkin_LIBRARIES})
add_dependencies(IMU_interface legged_robot_generate_messages_cpp)

add_executable(Force_interface src/Force_interface.cpp)
target_link_libraries(Force_interface ${catkin_LIBRARIES})
add_dependencies(Force_interface legged_robot_generate_messages_cpp)

add_executable(Read_Position src/Read_Position.cpp)
target_link_libraries(Read_Position ${catkin_LIBRARIES})
add_dependencies(Read_Position legged_robot_generate_messages_cpp)

add_executable(RobotSensorPreprocess src/RobotSensorPreprocess.cpp)
target_link_libraries(RobotSensorPreprocess ${catkin_LIBRARIES})
add_dependencies(RobotSensorPreprocess legged_robot_generate_messages_cpp)

## Declare a cpp executable
# add_executable(legged_robot_node src/legged_robot_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(legged_robot_node legged_robot_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(legged_robot_node
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
# install(TARGETS legged_robot legged_robot_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_legged_robot.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
