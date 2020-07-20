cmake_minimum_required(VERSION 2.8)
project(aruco_ros)

set(CMAKE_VERBOSE_MAKEFILE "true")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++11 -march=native -DEIGEN_DONT_PARALLELIZE ${CMAKE_CXX_FLAGS} -g -O0 -Wall")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        sensor_msgs
        tf
#        duckietown_msgs
#        message_generation
        )

find_package(Eigen3 REQUIRED)

#add_message_files(
#        FILES
#        AprilTagExtended.msg
#)

#generate_messages(
#        DEPENDENCIES
#        std_msgs
#        geometry_msgs
#)

catkin_package(
#        INCLUDE_DIRS include
#        CATKIN_DEPENDS message_runtime
)

include_directories(
        SYSTEM
#        include
        ../../devel/include
        lib/aruco-3.1.12/src
#	/opencv2
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})

link_directories(
#        ${PCL_LIBRARY_DIRS}
        ${PROJECT_SOURCE_DIR}/lib/mosek7)

add_subdirectory(lib/aruco-3.1.12)


#add_executable(aruco_dcf2 lib/aruco-3.1.12/utils_dcf/aruco_dcf.cpp)
#target_link_libraries(aruco_dcf2		aruco opencv_calib3d opencv_highgui pthread)


add_executable(aruco_detector
        src/aruco_detector.cpp
#        lib/aruco-3.1.12/utils_dcf/aruco_dcf.cpp
        )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(aruco_detector
        ${catkin_LIBRARIES}
#	opencv2
        aruco
#        opencv_aruco
        opencv_calib3d
        opencv_videoio
        opencv_imgproc
        opencv_highgui
        pthread
        )

#INSTALL(TARGETS aruco_detector RUNTIME DESTINATION bin)
