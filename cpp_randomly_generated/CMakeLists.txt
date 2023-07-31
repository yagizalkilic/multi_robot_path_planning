cmake_minimum_required(VERSION 3.10.2)
project(PathFinding)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the main.cpp file to the project
add_executable(PathFinding src/main.cpp src/project_utilities.cpp include/project_utilities.h src/agv_collision_space.cpp include/agv_collision_space.h src/rrt.cpp include/rrt.h src/coordination_visualization.cpp include/coordination_visualization.h src/rrt_star include/rrt_star)

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

target_link_libraries(PathFinding ${ROOT_LIBRARIES})
