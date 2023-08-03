cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_path_scheduling)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  turtlesim
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(turtle_space_generator_scheduler src/turtle_space_generator_scheduler.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star)

add_executable(turtle_follower src/turtle_follower.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star)

add_executable(algorithm_test src/main.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star)

target_link_libraries(turtle_follower ${catkin_LIBRARIES})
target_link_libraries(turtle_space_generator_scheduler ${catkin_LIBRARIES})
target_link_libraries(algorithm_test ${catkin_LIBRARIES})

if(DEFINED ENV{ROOTSYS})
  list(APPEND CMAKE_PREFIX_PATH $ENV{ROOTSYS})
elseif(DEFINED ROOTSYS)
  list(APPEND CMAKE_PREFIX_PATH ${ROOTSYS})
elseif(DEFINED ROOT_CMAKE_DIR)
  list(APPEND CMAKE_PREFIX_PATH ${ROOT_CMAKE_DIR})
# else()
#   message(
#     STATUS
#       "Please consider to set ROOTSYS or use -DROOTSYS=..."
#       "If still there is a problem, point to the directory which includes FindROOT.cmake using -DROOT_CMAKE_DIR=..."
# )
endif(DEFINED ENV{ROOTSYS})

set(CMAKE_PREFIX_PATH /home/milvus/root ${CMAKE_PREFIX_PATH})

find_package(ROOT REQUIRED COMPONENTS Net Core RIO Hist Graf Graf3d Gpad Tree Rint)

include_directories(${ROOT_INCLUDE_DIRS})

target_link_libraries(turtle_follower ${ROOT_LIBRARIES})
target_link_libraries(turtle_space_generator_scheduler ${ROOT_LIBRARIES})
target_link_libraries(algorithm_test ${ROOT_LIBRARIES})