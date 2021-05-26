cmake_minimum_required(VERSION 3.0.2)
project(tresc3_scan_algorithm_gl)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

if(NOT TARGET serial)
file(GLOB serial_src "include/serial/*.cc")
add_library(serial SHARED ${serial_src})
endif()

if(NOT TARGET gl_driver)
add_library(gl_driver
  include/gl_driver.cpp
)
endif()

add_executable(tresc3_scan_node src/tresc3_scan_node.cpp)
target_link_libraries(tresc3_scan_node ${catkin_LIBRARIES} gl_driver serial)
add_dependencies(tresc3_scan_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})