cmake_minimum_required(VERSION 2.8)

project(gmlreader)

add_executable(gmlreader utils.cpp gmlreader.hpp main.cpp node.cpp graph.cpp arc.cpp)
