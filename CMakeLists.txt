cmake_minimum_required(VERSION 2.8)

project(dual_manipulation_planner)


find_package(catkin REQUIRED COMPONENTS roscpp)
catkin_package(
INCLUDE_DIRS "include" 
)
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(src/lemon-1.3.1)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/lemon-1.3.1/)

add_subdirectory(src/lemon-1.3.1)

add_subdirectory(src/gml2lgf)

add_executable(dual_manipulation_planner src/graph_creator.cpp src/main.cpp)
target_link_libraries(dual_manipulation_planner ${catkin_LIBRARIES} lemon)
# ${CMAKE_BINARY_DIR}/lemon-1.3.1/lemon/liblemon.a)

install(TARGETS dual_manipulation_planner RUNTIME DESTINATION bin)
#install(FILE ${CMAKE_CURRENT_BINARY_DIR}/src/lemon-1.3.1/lemon/config.h DESTINATION include)