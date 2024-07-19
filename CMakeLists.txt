cmake_minimum_required(VERSION 3.22)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Set the project name and language
project(
  pmarp-is
  VERSION 0.0.1
  DESCRIPTION ""
  LANGUAGES CXX C)

find_package(GUROBI REQUIRED)
find_package(fmt REQUIRED)

# Adding the src, examples
add_subdirectory(src)