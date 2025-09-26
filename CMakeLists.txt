cmake_minimum_required(VERSION 3.10)

project(Path_planning)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)

set(SFML_DIR /usr/lib/x86_64-linux-gnu/cmake/SFML)
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

find_package(OpenCV REQUIRED)
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

add_executable(Astar src/Astar.cpp)
add_executable(RRT src/RRT.cpp)
add_executable(RRTstart src/RRTstart.cc)

target_include_directories(Astar PRIVATE ${OpenCV_INCLUDE_DIRS})
target_include_directories(RRT PRIVATE ${OpenCV_INCLUDE_DIRS})

target_link_libraries(Astar sfml-graphics sfml-window sfml-system ${OpenCV_LIBS})
target_link_libraries(RRT sfml-graphics sfml-window sfml-system ${OpenCV_LIBS})
target_link_libraries(RRTstart sfml-graphics sfml-window sfml-system ${OpenCV_LIBS})