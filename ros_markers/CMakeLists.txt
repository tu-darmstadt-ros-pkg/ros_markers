cmake_minimum_required(VERSION 2.8.3)
project(ros_markers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf 
        cv_bridge 
        image_transport 
        image_geometry 
        nodelet
        ros_markers_msgs
        #chilitags
)

## System dependencies
find_package(OpenCV REQUIRED)
find_package(chilitags REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ros_markers
  CATKIN_DEPENDS 
    tf
    nodelet
    ros_markers_msgs
  DEPENDS OpenCV
  LIBRARIES 
)

##########################################
##           Options                   ###
##########################################

option (WITH_KNOWLEDGE "Add support for exporting to a KB-API knowledge base" OFF)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${chilitags_INCLUDE_DIR})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${chilitags_LIBRARIES}
    )
    
if (WITH_KNOWLEDGE)

    add_definitions(-DWITH_KNOWLEDGE)
    pkg_check_modules(LIBKB REQUIRED liboro)
    include_directories(${LIBKB_INCLUDE_DIRS})
    set (LIBS ${LIBS} ${LIBKB_LDFLAGS})

endif()

add_library(ros_markers src/chilitagsdetector.cpp src/chilitagsdetector.hpp)
add_dependencies(ros_markers ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_markers ${LIBS})

add_executable(detect src/chilitagsdetector.cpp src/detect.cpp)
target_link_libraries(detect ${LIBS})

add_library(markers_nodelet src/chilitagsdetector.cpp src/nodelet.cpp)
add_dependencies(markers_nodelet ${catkin_EXPORTED_TARGETS})
target_link_libraries(markers_nodelet ${LIBS})


#############
## Install ##
#############

install(TARGETS detect markers_nodelet ros_markers
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    launch/detect.launch
    launch/detect_gscam.launch
    launch/gscam_nodelet_pipeline.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
    config/markers_configuration_sample.yml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
    nodelet_markers.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

