cmake_minimum_required(VERSION 2.8.3)
project(nord_planning)
set(CMAKE_CXX_FLAGS "--std=c++14 -Wall -Wno-reorder -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nord_planning src/nord_planning.cpp src/graph/graph.cpp src/dijkstra/map.cpp src/dijkstra/point.cpp src/dijkstra/path.cpp)
add_executable(initial_path src/tabu/initial_path.cpp src/tabu/map.cpp src/tabu/cone.cpp)

add_dependencies(nord_planning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(initial_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(nord_planning
  ${catkin_LIBRARIES}
)
target_link_libraries(initial_path
  ${catkin_LIBRARIES}
)






