cmake_minimum_required(VERSION 2.8)
project( VOtest )
add_definitions(-std=c++11)
find_package( OpenCV REQUIRED )
find_package (Eigen3 REQUIRED NO_MODULE)

set(CMAKE_BUILD_TYPE Debug)

file(GLOB SOURCE ./src/*.cpp)
file(GLOB HEADER ./src/*.h)

add_executable( VOtest ${SOURCE} ${HEADER})
target_link_libraries( VOtest ${OpenCV_LIBS})