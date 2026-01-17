cmake_minimum_required(VERSION 3.1...3.22)
project(poly_starter)

include_directories("/usr/local/include" "/opt/local/include" "./libs")

find_package(CGAL REQUIRED COMPONENTS Core)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_BUILD_TYPE Release)

# create a target per cppfile
set(SRCS
	poly_starter.cpp
	mesh_functions.cpp)

set(HEADERS
	mesh_functions.hpp
	starter_defs.h)

add_executable(poly_starter ${SRCS} ${HEADERS})

target_link_libraries(poly_starter CGAL::CGAL CGAL::CGAL_Core)
