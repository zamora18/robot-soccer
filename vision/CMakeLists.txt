cmake_minimum_required(VERSION 2.8)
project( test )

find_package( OpenCV REQUIRED )
add_executable( test main.cpp visionobject.cpp robot.cpp)
target_link_libraries( test ${OpenCV_LIBS} )
