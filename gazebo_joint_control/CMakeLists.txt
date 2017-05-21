cmake_minimum_required(VERSION 2.8.3)
project(gazebo_joint_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  sensor_msgs
  joint_trajectory_execution
  roslint
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
    # can check with if (TARGET doc) whether the target 'doc' was already used for another package.
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(gazebo_joint_control_doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
endif(DOXYGEN_FOUND)

# required for Gazebo 6 support
add_definitions(-std=c++11)

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
   INCLUDE_DIRS include gz2_support
   LIBRARIES gazebo_joint_control
   CATKIN_DEPENDS gazebo_ros roscpp sensor_msgs
   DEPENDS gazebo Boost
)

###########
## Build ##
###########
  
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  gz2_support
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(JOINT_CONTROL_SRC
   src/JointController.cpp
   gz2_support/src/JointControllerNewImpl.cpp
   src/GazeboJointControl.cpp
   src/GazeboJointControlLocalImpl.cpp
   src/GazeboJointControlLocalImplRegistration.cpp)

add_library(gazebo_joint_control
    ${JOINT_CONTROL_SRC} 
)

add_library(gazebo_joint_state_client
   src/JointController.cpp
   gz2_support/src/JointControllerNewImpl.cpp
   src/GazeboJointStateClient.cpp
)

add_library(gazebo_joint_state_publisher
   src/GazeboJointStatePublisher.cpp
)

add_library(gazebo_joint_trajectory_server
   src/GazeboJointTrajectoryServer.cpp
)


add_dependencies(gazebo_joint_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gazebo_joint_state_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gazebo_joint_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(gazebo_joint_trajectory_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
)

set (DEP_LIBS
   ${GAZEBO_LIBRARIES}   
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}   
)

target_link_libraries(gazebo_joint_control
   ${DEP_LIBS}
)

target_link_libraries(gazebo_joint_state_client
   ${DEP_LIBS}
)

target_link_libraries(gazebo_joint_state_publisher
   ${DEP_LIBS}
)

target_link_libraries(gazebo_joint_trajectory_server
   ${DEP_LIBS}
)


# style checking
roslint_cpp()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS gazebo_joint_control gazebo_joint_state_client
        gazebo_joint_state_publisher gazebo_joint_trajectory_server 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)

## Mark cpp header files for installation
install(FILES gz2_support/gazebo_joint_control/gz2_support/JointControllerNewImpl.h
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/gz2_support/
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY config 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.yaml"
)

if (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)
    install(DIRECTORY 
        ${CMAKE_CURRENT_BINARY_DIR}/docs/html 
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/doc
)
endif (EXISTS ${CMAKE_CURRENT_BINARY_DIR}/docs/html)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
