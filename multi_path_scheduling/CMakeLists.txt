cmake_minimum_required(VERSION 3.0.2)
project(multi_path_scheduling)

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

add_executable(turtle_restrained_subscriber src/turtle_restrained_scheduler/turtle_restrained_subscriber.cpp src/turtle_restrained_scheduler/node_schedule.cpp include/node_schedule.h src/turtle_restrained_scheduler/coordinated_motion_profiler.cpp include/coordinated_motion_profiler.h src/shared/project_utilities.cpp include/project_utilities.h src/shared/agv_collision_space.cpp include/agv_collision_space.h src/shared/rrt.cpp include/rrt.h src/shared/coordination_visualization.cpp include/coordination_visualization.h src/shared/rrt_star include/rrt_star src/shared/physical_path include/physical_path)

add_executable(turtle_restrained_publisher src/turtle_restrained_scheduler/turtle_restrained_publisher.cpp src/turtle_restrained_scheduler/node_schedule.cpp include/node_schedule.h src/turtle_restrained_scheduler/coordinated_motion_profiler.cpp include/coordinated_motion_profiler.h src/shared/project_utilities.cpp include/project_utilities.h src/shared/agv_collision_space.cpp include/agv_collision_space.h src/shared/rrt.cpp include/rrt.h src/shared/coordination_visualization.cpp include/coordination_visualization.h src/shared/rrt_star include/rrt_star src/shared/physical_path include/physical_path src/shared/connected_rrt_star include/connected_rrt_star)

add_executable(turtle_unrestrained_subscriber src/turtle_unrestrained_scheduler/turtle_unrestrained_subscriber.cpp src/turtle_unrestrained_scheduler/unrestrained_velocity_scheduler.cpp include/unrestrained_velocity_scheduler.h src/shared/project_utilities.cpp include/project_utilities.h src/shared/agv_collision_space.cpp include/agv_collision_space.h src/shared/rrt.cpp include/rrt.h src/shared/coordination_visualization.cpp include/coordination_visualization.h src/shared/rrt_star include/rrt_star src/shared/physical_path include/physical_path)

add_executable(turtle_unrestrained_publisher src/turtle_unrestrained_scheduler/turtle_unrestrained_publisher.cpp src/turtle_unrestrained_scheduler/unrestrained_velocity_scheduler.cpp include/unrestrained_velocity_scheduler.h src/shared/project_utilities.cpp include/project_utilities.h src/shared/agv_collision_space.cpp include/agv_collision_space.h src/shared/rrt.cpp include/rrt.h src/shared/coordination_visualization.cpp include/coordination_visualization.h src/shared/rrt_star include/rrt_star src/shared/physical_path include/physical_path)

add_executable(turtle_teleporter_publisher src/turtle_teleporter/turtle_teleporter_publisher.cpp src/shared/project_utilities.cpp include/project_utilities.h src/shared/agv_collision_space.cpp include/agv_collision_space.h src/shared/rrt.cpp include/rrt.h src/shared/coordination_visualization.cpp include/coordination_visualization.h src/shared/rrt_star include/rrt_star src/shared/physical_path include/physical_path)

add_executable(turtle_teleporter_subscriber src/turtle_teleporter/turtle_teleporter_subscriber.cpp src/shared/project_utilities.cpp include/project_utilities.h src/shared/agv_collision_space.cpp include/agv_collision_space.h src/shared/rrt.cpp include/rrt.h src/shared/coordination_visualization.cpp include/coordination_visualization.h src/shared/rrt_star include/rrt_star src/shared/physical_path include/physical_path)

add_executable(algorithm_test src/algorithm_tests/algorithm_test.cpp src/shared/project_utilities.cpp include/project_utilities.h src/shared/agv_collision_space.cpp include/agv_collision_space.h src/shared/rrt.cpp include/rrt.h src/shared/coordination_visualization.cpp include/coordination_visualization.h src/shared/rrt_star include/rrt_star src/shared/physical_path include/physical_path include/connected_rrt_star 
src/shared/connected_rrt_star)

find_package(Eigen3 REQUIRED)

target_link_libraries(turtle_restrained_subscriber ${catkin_LIBRARIES})
target_link_libraries(turtle_restrained_publisher ${catkin_LIBRARIES})
target_link_libraries(turtle_unrestrained_subscriber ${catkin_LIBRARIES})
target_link_libraries(turtle_unrestrained_publisher ${catkin_LIBRARIES})
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

target_link_libraries(turtle_restrained_subscriber ${ROOT_LIBRARIES})
target_link_libraries(turtle_restrained_publisher ${ROOT_LIBRARIES})
target_link_libraries(turtle_unrestrained_subscriber ${ROOT_LIBRARIES})
target_link_libraries(turtle_unrestrained_publisher ${ROOT_LIBRARIES})
target_link_libraries(turtle_teleporter_publisher ${ROOT_LIBRARIES})
target_link_libraries(turtle_teleporter_subscriber ${ROOT_LIBRARIES})
target_link_libraries(algorithm_test ${ROOT_LIBRARIES})

set (CMAKE_CXX_FLAGS "-g")

catkin_package(
  CATKIN_DEPENDS message_runtime
    )
