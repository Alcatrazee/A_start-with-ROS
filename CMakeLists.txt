cmake_minimum_required(VERSION 2.8.3)
project(map_test)


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  geometry_msgs
)


catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(map_decode_node src/map_decode.cpp)
target_link_libraries(map_decode_node ${catkin_LIBRARIES})


add_executable(Path_puber src/Path_puber.cpp)
target_link_libraries(Path_puber ${catkin_LIBRARIES})

add_executable(path_gen src/path_gen.cpp)
target_link_libraries(path_gen ${catkin_LIBRARIES})