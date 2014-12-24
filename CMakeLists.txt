cmake_minimum_required(VERSION 2.8)

project(dual_manipulation_planner)
ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS roscpp dual_manipulation_shared)
catkin_package(
INCLUDE_DIRS "include" 
)

message(WARNING ${catkin_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

include_directories(src/lemon-1.3.1)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src/lemon-1.3.1/)

add_subdirectory(src/lemon-1.3.1)

add_subdirectory(src/gml2lgf)

add_executable(dual_manipulation_planner src/graph_creator.cpp src/ros_server.cpp src/main.cpp)
target_link_libraries(dual_manipulation_planner ${catkin_LIBRARIES} lemon)
# ${CMAKE_BINARY_DIR}/lemon-1.3.1/lemon/liblemon.a)

install(TARGETS dual_manipulation_planner RUNTIME DESTINATION bin)
#install(FILE ${CMAKE_CURRENT_BINARY_DIR}/src/lemon-1.3.1/lemon/config.h DESTINATION include)