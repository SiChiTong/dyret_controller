cmake_minimum_required(VERSION 2.8.3)
project(dyret_controller)

set(ROS_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -ltbb -D SFERES_XML_WRITE")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

set(Boost_USE_STATIC_LIBS OFF)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  genmsg
  dyret_common
  dyret_hardware 
  tonnesfn_experiments
)

add_message_files(
  DIRECTORY msg
  FILES
  PositionCommand.msg
  ActionMessage.msg
  DistAngCommand.msg
  DistAngMeasurement.msg
  GaitControllerStatus.msg
  GaitConfiguration.msg
  GaitControllerCommand.msg
)

add_service_files(
  FILES
  GetGaitControllerStatus.srv
  GetGaitEvaluation.srv
  SendPositionCommand.srv
  ConfigureGait.srv
  GetInferredPosition.srv
  GaitControllerCommandService.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS 
  program_options
  serialization
  graph
)

find_package(TBB REQUIRED)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

add_executable(gaitController
               src/gaitController.cpp
               src/gait/BSplineGait.cpp
               src/kinematics/kinematicFunctions.cpp
               src/kinematics/forwardKinematics.cpp
               src/kinematics/inverseKinematics.cpp
               src/kinematics/interpolation.cpp
               src/kinematics/IncPoseAdjuster.cpp
               src/kinematics/movementFunctions.cpp
              )
target_link_libraries(gaitController ${catkin_LIBRARIES})
add_dependencies(gaitController
                 ${catkin_EXPORTED_TARGETS}
                 dyret_controller_generate_messages_cpp)

add_executable(gaitEvaluator
               src/gaitEvaluator.cpp
              )
target_link_libraries(gaitEvaluator ${catkin_LIBRARIES})
add_dependencies(gaitEvaluator
                 ${catkin_EXPORTED_TARGETS}
                 dyret_controller_generate_messages_cpp)

add_executable(inverseKinematicsAdapter
	src/inverseKinematicsAdapter.cpp
	src/kinematics/inverseKinematics.cpp
)
target_link_libraries(inverseKinematicsAdapter ${catkin_LIBRARIES})
add_dependencies(inverseKinematicsAdapter
                 ${catkin_EXPORTED_TARGETS}
                 dyret_controller_generate_messages_cpp)
