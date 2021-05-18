cmake_minimum_required(VERSION 2.8.3)
project(drone_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  glog_catkin
  mav_msgs
  planning_msgs
  roscpp
  sensor_msgs
  tf
  cv_bridge
  image_transport
)

find_package(OpenCV #[[3.3.1]] REQUIRED)
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
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
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
#   geometry_msgs#   mav_msgs#   planning_msgs#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES drone_control_loop controller_utility vacuum
  CATKIN_DEPENDS cmake_modules geometry_msgs glog_catkin mav_msgs planning_msgs roscpp sensor_msgs tf image_transport cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include include/drone_control include/controller_utility include/vacuum include/vision_sensor include/other_sensors
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OPENCV_INCLUDE_DIR})

## Declare a C++ library
add_library(drone_control
  src/drone_control/attitude_control_loop.cpp
  src/drone_control/position_control_loop.cpp
  src/drone_control/velocity_control_loop.cpp
  src/drone_control/acceleration_control_loop.cpp
)

add_library(controller_utility
  src/controller_utility/controller_utility.cpp
  src/controller_utility/PID.cpp
  src/controller_utility/flocking.cpp
  src/controller_utility/random_wp_flying.cpp
  src/controller_utility/random_velocity_flying.cpp
  src/controller_utility/obstacle_avoidance.cpp
)
add_library(vacuum
  src/vacuum/vacuum.cpp)

add_library(vision_sensor
  #src/vision_sensor/vision_sensor_perspective.cpp
  #src/vision_sensor/vision_sensor_stitching360.cpp
  #src/vision_sensor/vision_sensor_down.cpp
  src/vision_sensor/vision_sensor_panoramic.cpp
  )

add_library(other_sensors
  src/other_sensors/proximity_sensor.cpp
  src/other_sensors/ultrasonic_sensor.cpp
  )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/drone_controller_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

add_executable(drone src/drone.cpp)
add_dependencies(drone ${drone_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(drone 
  drone_control
  controller_utility
  vacuum
  vision_sensor
  other_sensors
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(flocking_cmd_generator src/flocking_cmd_generator.cpp)
add_dependencies(flocking_cmd_generator ${flocking_cmd_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(flocking_cmd_generator 
  drone_control
  controller_utility
  vacuum
  vision_sensor
  other_sensors
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(bacterium_cmd_generator src/bacterium_cmd_generator.cpp)
add_dependencies(bacterium_cmd_generator ${bacterium_cmd_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bacterium_cmd_generator 
  drone_control
  controller_utility
  vacuum
  vision_sensor
  other_sensors
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES})

add_executable(vacuum_cmd_generator src/vacuum_cmd_generator.cpp)
add_dependencies(vacuum_cmd_generator ${vacuum_cmd_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vacuum_cmd_generator 
  drone_control
  controller_utility
  vacuum
  vision_sensor
  other_sensors
  ${catkin_LIBRARIES}
)

add_executable(lift_wp_planning src/lift_wp_planning.cpp)
add_dependencies(lift_wp_planning ${lift_wp_planning_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lift_wp_planning 
  drone_control
  controller_utility
  vacuum
  vision_sensor
  other_sensors
  ${catkin_LIBRARIES}
)

add_executable(land_wp_planning src/land_wp_planning.cpp)
add_dependencies(land_wp_planning ${land_wp_planning_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(land_wp_planning 
  drone_control
  controller_utility
  vacuum
  vision_sensor
  other_sensors
  ${catkin_LIBRARIES}
)

add_executable(destination_cmd_generator src/destination_cmd_generator.cpp)
add_dependencies(destination_cmd_generator ${destination_cmd_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(destination_cmd_generator 
  drone_control
  controller_utility
  vacuum
  vision_sensor
  other_sensors
  ${catkin_LIBRARIES}
)

add_executable(uniform_cmd_generator src/uniform_cmd_generator.cpp)
add_dependencies(uniform_cmd_generator ${uniform_cmd_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(uniform_cmd_generator 
  ${catkin_LIBRARIES}
)
add_executable(nonuniform_cmd_generator src/nonuniform_cmd_generator.cpp)
add_dependencies(nonuniform_cmd_generator ${nonuniform_cmd_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nonuniform_cmd_generator 
  ${catkin_LIBRARIES}
)
add_executable(navigator_carrier_pose_generator src/navigator_carrier_pose_generator.cpp)
add_dependencies(navigator_carrier_pose_generator ${navigator_carrier_pose_generator_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigator_carrier_pose_generator 
  ${catkin_LIBRARIES}
)
#add_executable(suction_action src/suction.cpp)
#add_dependencies(suction_action ${suction_action_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_drone_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
