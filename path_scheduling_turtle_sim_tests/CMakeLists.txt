cmake_minimum_required(VERSION 3.0.2)
project(path_scheduling_turtle_sim_tests)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  turtlesim
  geometry_msgs
  tf
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES 
  velocities_durations_orientations.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_executable(turtle_subscriber src/turtle_subscriber.cpp src/velocity_scheduler.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star src/physical_path include/physical_path)

add_executable(turtle_publisher src/turtle_publisher.cpp src/velocity_scheduler.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star src/physical_path include/physical_path)

add_executable(turtle_teleporter_publisher src/turtle_teleporter_publisher.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star src/physical_path include/physical_path)

add_executable(turtle_teleporter_subscriber src/turtle_teleporter_subscriber.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star src/physical_path include/physical_path)

add_executable(algorithm_test src/algorithm_test.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star src/physical_path include/physical_path)

target_link_libraries(turtle_subscriber ${catkin_LIBRARIES})
target_link_libraries(turtle_publisher ${catkin_LIBRARIES})
target_link_libraries(turtle_teleporter_publisher ${catkin_LIBRARIES})
target_link_libraries(turtle_teleporter_subscriber ${catkin_LIBRARIES})
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

find_package(ROOT REQUIRED COMPONENTS Net Core RIO Hist Graf Graf3d Gpad Tree Rint)

include_directories(${ROOT_INCLUDE_DIRS})

target_link_libraries(turtle_subscriber ${ROOT_LIBRARIES})
target_link_libraries(turtle_publisher ${ROOT_LIBRARIES})
target_link_libraries(turtle_teleporter_publisher ${ROOT_LIBRARIES})
target_link_libraries(turtle_teleporter_subscriber ${ROOT_LIBRARIES})
target_link_libraries(algorithm_test ${ROOT_LIBRARIES})

set (CMAKE_CXX_FLAGS "-g")

catkin_package(
  CATKIN_DEPENDS message_runtime
    )
