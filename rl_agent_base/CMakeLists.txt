cmake_minimum_required(VERSION 2.8.3)
project(rl_agent_base)

add_definitions(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  class_loader
  cmake_modules
  control_toolbox
  geometry_msgs
#  orocos_kdl
  kdl_parser
  realtime_tools
  roscpp
  roslib
  rospy
  urdf
  sensor_msgs
  std_msgs
  tf

  rl_msgs
  mii_foundation
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)

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
#  LIBRARIES rl_agent
  CATKIN_DEPENDS  class_loader cmake_modules control_toolbox geometry_msgs kdl_parser
                  realtime_tools roscpp roslib rospy sensor_msgs std_msgs tf urdf rl_msgs
  DEPENDS orocos_kdl
)

###########
## Build ##
###########
set(SRC_ROOT            src)
set(SRC_TEST_ROOT       example/src)

set(SRC_SAMPLE          ${SRC_ROOT}/sample)
set(SRC_SENSOR          ${SRC_ROOT}/sensor)
set(SRC_CONTROLLER      ${SRC_ROOT}/controller)


set(PLUGIN_FILES        # ${SRC_ROOT}/robot_plugin.cpp
                        ${SRC_ROOT}/robot_plugin2.cpp
                        ${SRC_SAMPLE}/sample.cpp
)

set(SENSOR_FILES        ${SRC_SENSOR}/sensor.cpp
                        ${SRC_SENSOR}/chain_joint_encoder.cpp
)

set(CONTROLLER_FILES    ${SRC_CONTROLLER}/trial_controller.cpp
                        ${SRC_CONTROLLER}/proportional.cpp
                        ${SRC_CONTROLLER}/position_controller.cpp
                        ${SRC_CONTROLLER}/lqr.cpp
)

set(EX_FILES            ${SRC_TEST_ROOT}/plugin_example.cpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${PLUGIN_FILES}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_sensor
  ${SENSOR_FILES}
)

add_library(${PROJECT_NAME}_controller
  ${CONTROLLER_FILES}
)

add_library(${PROJECT_NAME}_example
  ${EX_FILES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(rl_agent ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(rl_agent_node src/rl_agent_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(rl_agent_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
  mii_foundation
  ${PROJECT_NAME}_sensor
  ${PROJECT_NAME}_controller
)

target_link_libraries(${PROJECT_NAME}_sensor
  ${catkin_LIBRARIES}
  ${orocos_kdl_LIBRARIES}
  ${kdl_parser_LIBRARIES}
  ${boost_LIBRARIES}
  mii_foundation
  mii_foundation_repository
)

target_link_libraries(${PROJECT_NAME}_controller
  ${catkin_LIBRARIES}
  ${boost_LIBRARIES}
  mii_foundation
  mii_foundation_toolbox
)

target_link_libraries(${PROJECT_NAME}_example
  ${PROJECT_NAME}
)


#############
## Install ##
#############

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
                ${PROJECT_NAME}_sensor
                ${PROJECT_NAME}_controller
                ${PROJECT_NAME}_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rl_agent.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
