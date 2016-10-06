cmake_minimum_required(VERSION 2.8.3)
project(b_usb_cam)

find_package(catkin REQUIRED COMPONENTS
  image_transport roscpp std_msgs std_srvs sensor_msgs camera_info_manager
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(avcodec libavcodec REQUIRED)
pkg_check_modules(swscale libswscale REQUIRED)

catkin_package()


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${avcodec_INCLUDE_DIRS}
  ${swscale_INCLUDE_DIRS}
)

## Build the USB camera library
add_library(${PROJECT_NAME} src/usb_cam.cpp)
target_link_libraries(${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node nodes/usb_cam_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_library(b_usb_cam_nodelet nodes/usb_cam_nodelet.cpp)
target_link_libraries(b_usb_cam_nodelet
  ${PROJECT_NAME}
  ${avcodec_LIBRARIES}
  ${swscale_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(b_usb_cam_nodelet ${PROJECT_NAME}_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
 install(TARGETS b_usb_cam_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
 
 install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 
 install(FILES
   launch/b_usb_cam.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 )
 