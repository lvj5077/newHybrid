CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(newHybrid)

# SET(CMAKE_CXX_COMPILER "/usr/bin/c++")
SET( CMAKE_BUILD_TYPE Release )
SET( CMAKE_CXX_FLAGS "-O3 -std=c++11" )

# Eigen
include_directories("/opt/homebrew/Cellar/eigen/3.3.9/include")
set(EIGEN3_INCLUDE_DIR "/opt/homebrew/Cellar/eigen/3.3.9/include/")
include_directories("/opt/homebrew/Cellar/eigen/3.3.9/include/eigen3")

SET(OpenCV_DIR "/Users/jin/Q_Mac/third_party/opencv-3.4.12/build")
find_package( OpenCV 3.4 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_LIB_DIR})
MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( STATUS "OpenCV_LIB_DIR: " ${OpenCV_LIB_DIR} )
MESSAGE( STATUS "OpenCV_LIBS: " ${OpenCV_LIBS} )




SET(CSparse_include_DIR "/Users/jin/Q_Mac/third_party/SuiteSparse/include/")
SET(CSparse_LIB "/Users/jin/Q_Mac/third_party/SuiteSparse/lib/")
include_directories( ${CSparse_include_DIR} )
link_directories( ${CSparse_LIB} )

# SET( Ceres_DIR  "/Users/jin/Q_Mac/third_party/ceres-solver/builds")

# SET(CERES_INCLUDE_DIRS "/Users/jin/Q_Mac/third_party/ceres-solver/include/")
# SET(CERES_LIBRARIES "/Users/jin/Q_Mac/third_party/ceres-solver/builds/lib/")

find_package( Ceres REQUIRED )
MESSAGE( STATUS "CERES_INCLUDE_DIRS: " ${CERES_INCLUDE_DIRS} ) # include eigen3 as well
MESSAGE( STATUS "CERES_LIBRARIES: " ${CERES_LIBRARIES} )
include_directories( ${CERES_INCLUDE_DIRS} )

# /usr/local/Cellar/glog/0.4.0
SET(glog_include_DIR "/opt/homebrew/Cellar/glog/0.4.0/include/")
SET(glog_LIB "/opt/homebrew/Cellar/glog/0.4.0/lib/")
find_package( glog )
include_directories( ${glog_include_DIR})
link_directories( ${glog_LIB} )

# "/usr/local/Cellar/gflags/2.2.2/include"
SET(gflags_include_DIR "/opt/homebrew/Cellar/gflags/2.2.2/include/")
SET(gflags_LIB "/opt/homebrew/Cellar/gflags/2.2.2/lib/")
find_package( gflags )
include_directories( ${gflags_include_DIR})
link_directories( ${gflags_LIB} )




ADD_EXECUTABLE( newHybrid newHybrid.cpp) 
target_link_libraries(newHybrid  ${CERES_LIBRARIES} ) 
