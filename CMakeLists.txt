cmake_minimum_required(VERSION 2.8.3)
project(graph_slam)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules/ )

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg tf)

#########################################################
# OPENCV
#########################################################
find_package( OpenCV REQUIRED )
message("OpenCV include dirs: ${OPENCV_INCLUDE_DIRS}")

find_package(cvsba REQUIRED )

# find_package( Eigen3 REQUIRED ) # for rot matrix to quaternion conversion
# include_directories( ${EIGEN3_INCLUDE_DIR} )

# PCL ########################## PCL (apparently) links the required BOOST libraries and Eigen3 
find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
################################


############ G2O #################
SET( G2O_INCLUDE ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o CACHE PATH "Directory of G2O")
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${G2O_INCLUDE})

# Add g2o lib dir
LINK_DIRECTORIES( ${LINK_DIRECTORIES} "${G2O_INCLUDE}/lib" )

#Generate config.h
configure_file(EXTERNAL/g2o/config.h.in ${PROJECT_BINARY_DIR}/EXTERNAL/g2o/config.h)
include_directories(${PROJECT_BINARY_DIR} ${PROJECT_BINARY_DIR}/EXTERNAL)
INSTALL(FILES ${PROJECT_BINARY_DIR}/EXTERNAL/g2o/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o)

# Include the subdirectories
ADD_SUBDIRECTORY(EXTERNAL/g2o)

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR}) #You can use CSPARSE or CHOLMOD
#INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

set(G2O_DIR ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o)
include_directories(${G2O_DIR})
link_directories(${G2O_DIR}/lib)

include_directories(external/
                    /usr/include/suitesparse/
                    ${CSPARSE_INCLUDE_DIR}
                    ${G2O_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIR})

################################################
# STAM set-up (for Visual Odometry)
################################################

SET(STAM_DIR ${PROJECT_SOURCE_DIR}/EXTERNAL/STAM CACHE PATH "Directory of STAM")
SET(STAM_INC ${STAM_DIR}/include)
find_library(STAM_LIBRARY libvostam.so HINTS ${STAM_DIR}/build/)

########################################################

SET(GSLAM_INC ${PROJECT_SOURCE_DIR}/include)
SET(GSLAM_SRC_DIR ${PROJECT_SOURCE_DIR}/src)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visual_odom
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    ${GSLAM_INC}
    ${STAM_INC}
)

SET(GSLAM_SOURCES ${GSLAM_SRC_DIR}/gslam/ros_visualizer.cpp
                  ${GSLAM_SRC_DIR}/gslam/graphslam.cpp
                  ${GSLAM_SRC_DIR}/gslam/data_spot.cpp
                  ${GSLAM_SRC_DIR}/gslam/data_pool.cpp
                  ${GSLAM_SRC_DIR}/gslam/fabmap.cpp
                  ${GSLAM_SRC_DIR}/gslam/transform_estimator.cpp
                  ${GSLAM_SRC_DIR}/gslam/slam_utils.cpp
                  ${GSLAM_SRC_DIR}/gslam/graph.cpp
                  ${GSLAM_SRC_DIR}/gslam/image_matcher.cpp)



SET(GSLAM_LINK_LIBS ${PCL_LIBRARIES}) 
SET(STAM_LINK_LIBS ${OpenCV_LIBS} ${cvsba_LIBS}) 

add_library(gslam ${GSLAM_SOURCES})

add_executable(vis_odo_node src/vis_odo_node.cpp)
add_executable(test_node src/test.cpp)
add_executable(main_slam_node src/main.cpp)
add_executable(rename_dataset src/rename_image.cpp)

add_executable(points_and_lines src/test_pub.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})

target_link_libraries(vis_odo_node
   ${catkin_LIBRARIES}
    ${STAM_LIBRARY} ${OpenCV_LIBS} ${cvsba_LIBS}
)
target_link_libraries(rename_dataset
   ${catkin_LIBRARIES}
    ${STAM_LIBRARY} ${OpenCV_LIBS} ${cvsba_LIBS}
)
target_link_libraries(test_node
   ${catkin_LIBRARIES}
	${STAM_LIBRARY}	${OpenCV_LIBS} ${cvsba_LIBS}
)
target_link_libraries(gslam ${STAM_LIBRARY} ${STAM_LINK_LIBS} ${GSLAM_LINK_LIBS} ${catkin_LIBRARIES} core types_slam3d
                                   solver_csparse)

target_link_libraries(main_slam_node
   ${catkin_LIBRARIES} gslam
    ${STAM_LIBRARY} ${STAM_LINK_LIBS} ${GSLAM_LINK_LIBS} core types_slam3d
                                   solver_csparse
)
