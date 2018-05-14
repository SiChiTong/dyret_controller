cmake_minimum_required(VERSION 2.8.3)
project(dyret_controller)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -ltbb -D SFERES_XML_WRITE")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(message_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  genmsg
  dynamic_reconfigure
  dyret_common
  dyret_hardware
)

find_package(Boost REQUIRED COMPONENTS 
  program_options
  serialization
  graph
)

find_package(TBB REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/gaitControllerParams.cfg
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
)

add_executable(inverseKinematicsService src/inverseKinematicsService.cpp)
target_link_libraries(inverseKinematicsService 
	${catkin_LIBRARIES})
add_dependencies(inverseKinematicsService 
	${catkin_EXPORTED_TARGETS})

add_executable(gaitController 
               src/gaitController.cpp 
               src/gait/gait.cpp
               src/gait/GaitSegment.cpp
               src/gait/BSplineGait.cpp
               src/kinematics/kinematicFunctions.cpp
               src/kinematics/forwardKinematics.cpp
               src/kinematics/interpolation.cpp
               src/kinematics/IncPoseAdjuster.cpp
               src/kinematics/movementFunctions.cpp
              )
target_link_libraries(gaitController ${catkin_LIBRARIES})
add_dependencies(gaitController 
  ${catkin_EXPORTED_TARGETS})

add_executable(trajectoryController
               src/trajectoryController.cpp
              )
target_link_libraries(trajectoryController ${catkin_LIBRARIES})
add_dependencies(trajectoryController 
	${catkin_EXPORTED_TARGETS})

add_executable(gaitEvaluator
               src/gaitEvaluator.cpp
              )
target_link_libraries(gaitEvaluator ${catkin_LIBRARIES})
add_dependencies(gaitEvaluator 
	${catkin_EXPORTED_TARGETS})

add_executable(testKinematics
               src/kinematics/testKinematics.cpp
               src/kinematics/forwardKinematics.cpp
              )
target_link_libraries(testKinematics ${catkin_LIBRARIES})
add_dependencies(testKinematics 
	${catkin_EXPORTED_TARGETS})
