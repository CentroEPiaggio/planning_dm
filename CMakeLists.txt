cmake_minimum_required(VERSION 2.8)

project(dual_manip_planning)

include_directories(lemon-1.3.1)
include_directories(${CMAKE_BINARY_DIR}/lemon-1.3.1)

add_subdirectory(lemon-1.3.1)

add_subdirectory(gml2lgf)


add_executable(dual_manip_planning graph_creator.cpp main.cpp)
target_link_libraries(dual_manip_planning ${CMAKE_BINARY_DIR}/lemon-1.3.1/lemon/liblemon.a)

install(TARGETS dual_manip_planning RUNTIME DESTINATION bin)
