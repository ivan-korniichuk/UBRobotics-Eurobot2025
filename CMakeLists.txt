cmake_minimum_required(VERSION 3.10)
project(Robot_Controls)

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SOURCES "src/*.cpp")
file(GLOB_RECURSE HEADERS "include/*.hpp")

add_executable(robot_controls ${SOURCES})
target_include_directories(robot_controls PUBLIC include)
target_link_libraries(robot_controls ${OpenCV_LIBS})
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
