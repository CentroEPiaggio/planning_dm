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


find_package(catkin REQUIRED COMPONENTS roscpp roslib dual_manipulation_shared)
catkin_package(
	INCLUDE_DIRS include
	LIBRARIES planning_lib
	CATKIN_DEPENDS dual_manipulation_shared
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_INSTALL_PREFIX}/include) #this line is to find lemon include files!!
link_directories(${catkin_LIBRARY_DIRS})

add_subdirectory(src/gml2lgf)

add_executable(dual_manipulation_planner src/graph_creator.cpp src/ros_server.cpp src/main.cpp)
add_dependencies(dual_manipulation_planner dual_manipulation_shared_generate_messages_cpp dual_manipulation_shared lemon)
target_link_libraries(dual_manipulation_planner ${catkin_LIBRARIES} lemon)
# add_dependencies(dual_manipulation_planner dual_manipulation_shared_gencpp)

add_library(planning_lib SHARED src/graph_creator.cpp src/planner_lib.cpp)
target_link_libraries(planning_lib ${catkin_LIBRARIES} lemon)
add_dependencies(planning_lib dual_manipulation_shared_generate_messages_cpp dual_manipulation_shared lemon)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS planning_lib
         EXPORT planning_lib
        ARCHIVE DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT lib
#         RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}" COMPONENT bin
         LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}" COMPONENT shlib
)
install(TARGETS dual_manipulation_planner RUNTIME DESTINATION bin)

add_executable(test_planning test/test_planning.cpp src/planner_lib.cpp src/graph_creator.cpp)
add_dependencies(test_planning dual_manipulation_shared_generate_messages_cpp dual_manipulation_shared lemon)
target_link_libraries(test_planning ${catkin_LIBRARIES})

add_executable(single_grasp_test test/single_grasp_planning.cpp)
add_dependencies(single_grasp_test dual_manipulation_shared_generate_messages_cpp dual_manipulation_shared)
target_link_libraries(single_grasp_test ${catkin_LIBRARIES})


# add_dependencies(test_planning dual_manipulation_shared_gencpp)
