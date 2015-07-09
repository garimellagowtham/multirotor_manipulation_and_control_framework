cmake_minimum_required(VERSION 2.8.3)
project(multirotor_manipulation_and_control_framework)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rqt_gui
  rqt_gui_cpp
  libmavconn
  sensor_msgs
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(GCOP REQUIRED COMPONENTS systems est)
find_package(TinyXML REQUIRED)

message("TINYXML: ${TINYXML_LIBRARIES}")

include(EnableCXX11)
include(MavrosMavlink)


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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

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
#  LIBRARIES multirotor_manipulation_and_control_framework
#  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_cpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
 include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GCOP_INCLUDE_DIRS}
  ${TINYXML_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
)

#set(USB2AX ON)

if(USB2AX)
   set(DXL_HAL_SRC include/multirotor_manipulation_and_control_framework/dynamixelsdk/dxl_hal_usb2ax.C)
 else(USB2AX)
   set(DXL_HAL_SRC include/multirotor_manipulation_and_control_framework/dynamixelsdk/dxl_hal_usb2dynamixel.C)
 endif(USB2AX)


## Declare a cpp library
 add_library(${PROJECT_NAME}
   include/multirotor_manipulation_and_control_framework/trajectory_tracking_control.cpp
   include/multirotor_manipulation_and_control_framework/pixhawk_parser.cpp
   include/multirotor_manipulation_and_control_framework/point3d_estimator.cpp
   include/multirotor_manipulation_and_control_framework/simple_arm.cpp
   include/multirotor_manipulation_and_control_framework/generic_arm.cpp
   include/multirotor_manipulation_and_control_framework/dynamixelsdk/dynamixel.C
   ${DXL_HAL_SRC}
 )

target_link_libraries(${PROJECT_NAME} ${GCOP_LIBRARIES} ${TINYXML_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a cpp executable
 add_executable(simple_arm_example src/arm_examples/simple_arm_example.cpp)
 add_executable(cyton_arm_example src/arm_examples/cyton_arm_example.cpp)

 add_executable(pixhawk_parser_example src/quadcopter_parser_examples/pixhawk_parser_example.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(multirotor_manipulation_and_control_framework_node multirotor_manipulation_and_control_framework_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(simple_arm_example
   ${PROJECT_NAME}
 )

 target_link_libraries(cyton_arm_example
   ${PROJECT_NAME}
 )

 target_link_libraries(cyton_arm_example
   ${PROJECT_NAME}
 )

 target_link_libraries(pixhawk_parser_example
 ${PROJECT_NAME}
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
# install(TARGETS multirotor_manipulation_and_control_framework multirotor_manipulation_and_control_framework_node
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
 catkin_add_gtest(${PROJECT_NAME}-test test/utest.cpp)
 if(TARGET ${PROJECT_NAME}-test)
   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
 endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)