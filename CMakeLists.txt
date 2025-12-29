cmake_minimum_required(VERSION 4.0..4.2)
project(advent-of-code-2025
    DESCRIPTION "advent of code 2025"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

include(cmake/UpdateSubmodules.cmake)

include(CTest)

add_subdirectory(external)

add_subdirectory(src)
