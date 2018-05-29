cmake_minimum_required(VERSION 2.8)
project( VOtest )
add_definitions(-std=c++14)
find_package( OpenCV REQUIRED )
find_package( Ceres REQUIRED )
find_package(GTest REQUIRED)
find_package (Threads)
find_package (Eigen3 REQUIRED NO_MODULE)

set(CMAKE_BUILD_TYPE Debug)

file(GLOB SOURCE ./src/*.cpp)
file(GLOB HEADER ./src/*.h)

add_executable( VOtest ${SOURCE} ${HEADER})
target_link_libraries( VOtest ${OpenCV_LIBS} ${CERES_LIBRARIES})

enable_testing()
add_test(NAME MyTest COMMAND runUnitTests)

add_executable(runUnitTests ./test/test.cpp ./src/Utils.cpp ./src/Utils.h)
target_link_libraries(runUnitTests gtest gtest_main ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})

