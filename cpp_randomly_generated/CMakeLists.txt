cmake_minimum_required(VERSION 3.10.2)
project(PathFinding)

# Set C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the main.cpp file to the project
add_executable(PathFinding n_dimension_path_finding.cpp project_utilities.cpp project_utilities.h agv_collision_space.cpp agv_collision_space.h rrt.cpp rrt.h coordination_visualization.cpp coordination_visualization.h)

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
