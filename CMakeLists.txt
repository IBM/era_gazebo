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

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)


#####################################

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  gazebo_ros
  tf
  nav_msgs
  gazebo_msgs
  message_generation
  image_transport 
  cv_bridge
  image_geometry
  pcl_ros
)

#link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}  ${LZ4_INCLUDE_DIR} include)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_message_files(
  FILES
  ERAMsg.msg
  DetectionBox.msg
  DetectionBoxList.msg
)

add_service_files(
   FILES
   SceneBBox.srv
 )

generate_messages(
  DEPENDENCIES
  nav_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    tf
    message_runtime
)

## 'Map Fuser' node
add_executable(map_fuser src/map_fuser.cpp src/combine_grids.cpp src/coordinate_conversions.cpp)
target_link_libraries(map_fuser ${catkin_LIBRARIES})
add_dependencies(map_fuser era_gazebo_generate_messages_eus)

add_executable(ERAmsgBuilder_node src/ERAmsgBuilder.cpp)
add_dependencies(ERAmsgBuilder_node era_gazebo_generate_messages_cpp)
target_link_libraries(ERAmsgBuilder_node
  ${catkin_LIBRARIES}
)

add_executable(ERAmsgInterpreter_node src/ERAmsgInterpreter.cpp)
add_dependencies(ERAmsgInterpreter_node era_gazebo_generate_messages_cpp)
target_link_libraries(ERAmsgInterpreter_node
  ${catkin_LIBRARIES}
)

add_Executable(og_labeler src/occupancy_grid_labeler.cpp)
add_dependencies(og_labeler era_gazebo_generate_messages_cpp)
target_link_libraries(og_labeler
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
)

add_Executable(fake_laser src/fake_laser.cpp)
add_dependencies(fake_laser era_gazebo_generate_messages_cpp)
target_link_libraries(fake_laser
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
)

add_Executable(occgrid_node src/occgrid_node.cpp src/occgrid.c)
add_dependencies(occgrid_node era_gazebo_generate_messages_cpp)
target_link_libraries(occgrid_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
)