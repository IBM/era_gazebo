cmake_minimum_required(VERSION 2.8.3)
project(era_gazebo)

# Added by Augusto Vega #############
macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

use_cxx11()
#####################################

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_ros
  tf
  nav_msgs
  message_generation
)

# Depend on system install of Gazebo
#find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_message_files(
  FILES
  ERAOccupancyGrid.msg
)

generate_messages(
  DEPENDENCIES
  nav_msgs
)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    tf
    message_runtime
)

## 'Map Fuser' node
add_executable(map_fuser src/map_fuser.cpp)
target_link_libraries(map_fuser ${catkin_LIBRARIES})
add_dependencies(map_fuser era_gazebo_generate_messages_eus)

## 'Message Generator' node
add_executable(message_generator src/message_generator.cpp)
target_link_libraries(message_generator ${catkin_LIBRARIES})
add_dependencies(message_generator era_gazebo_generate_messages_eus)

