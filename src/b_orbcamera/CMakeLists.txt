cmake_minimum_required(VERSION 2.8.3)
project(b_orbcamera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  image_geometry
  image_transport
  sensor_msgs
  message_generation
  
  pluginlib 
  nodelet
)

#find_package(catkin REQUIRED cmake_modules)
#find_package(Eigen3 REQUIRED) 

add_service_files(
   FILES
   skeltrack.srv
 )
generate_messages(
  DEPENDENCIES
  std_msgs
)



catkin_package(
DEPENDS Eigen
CATKIN_DEPENDS nodelet roscpp std_msgs
)




###########
## Build ##
###########
	     
find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS ../../tools/NiTE-Linux-x64-2.2/Include
	  )	  
find_library(Nite2_LIBRARIES 
	  NAMES NiTE2
	  HINTS ../../tools/NiTE-Linux-x64-2.2/Redist
	  PATH_SUFFIXES lib
	  )	
message(status ${Nite2_INCLUDEDIR})
message(status ${Nite2_LIBRARIES})

find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS ../../tools/OpenNI-Linux-x64-2.2/Include
	  )

#is RedistforNite, not Redist	  
find_library(OpenNI2_LIBRARIES
     NAMES OpenNI2 OniFile ORBBEC PSLink
     HINTS ../../tools/OpenNI-Linux-x64-2.2/RedistforNite ../../tools/OpenNI-Linux-x64-2.2/RedistforNite/OpenNI2/Drivers
     PATH_SUFFIXES lib
     )

message(STATUS ${OpenNI2_LIBRARIES})
	  
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenNI2_INCLUDEDIR}
  ${Nite2_INCLUDEDIR} 
)
 
###########
## Compile ##
###########
 
# add_executable(b_skeleton_node src/main_skeleton.cpp src/CxSkeleton.hpp src/OrbbecCamera.hpp)
# target_link_libraries(b_skeleton_node
#   ${Nite2_LIBRARIES}
#   ${OpenNI2_LIBRARIES}
#   ${catkin_LIBRARIES}     
   #${Eigen_LIBRARIES}  
# )
#add_dependencies(b_skeleton_node ${PROJECT_NAME}_generate_messages_cpp)
 
 add_executable(b_orbcamera_node src/main_orbcamera.cpp)
 target_link_libraries(b_orbcamera_node
   ${Nite2_LIBRARIES}
   ${OpenNI2_LIBRARIES}
   ${catkin_LIBRARIES}      
 )
add_dependencies(b_orbcamera_node ${PROJECT_NAME}_generate_messages_cpp)

 add_executable(b_orbcamera_node_test src/TestOrbbecCameraNode.cpp)
 target_link_libraries(b_orbcamera_node_test
   ${Nite2_LIBRARIES}
   ${OpenNI2_LIBRARIES}
   ${catkin_LIBRARIES}      
 )
#add_dependencies(b_orbcamera_node_test ${PROJECT_NAME}_generate_messages_cpp)

# Nodelet library
 #add_library(orbbec_camera_nodelet src/OrbbecCameraNodelet.hpp)
 #target_link_libraries(orbbec_camera_nodelet
 #  ${Nite2_LIBRARIES}
  # ${OpenNI2_LIBRARIES}
  # ${catkin_LIBRARIES}      
 #)
 #add_dependencies(orbbec_camera_nodelet ${PROJECT_NAME}_generate_messages_cpp)
 
 add_library(b_orbcamera_nodelet src/OrbbecCameraNodelet.cpp)
target_link_libraries(b_orbcamera_nodelet
  ${Nite2_LIBRARIES}
  ${OpenNI2_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(b_orbcamera_nodelet ${PROJECT_NAME}_generate_messages_cpp)

 add_library(b_orbcamera_nodelet_test src/TestOrbbecCameraNodelet.cpp)
target_link_libraries(b_orbcamera_nodelet_test
  ${Nite2_LIBRARIES}
  ${OpenNI2_LIBRARIES}
  ${catkin_LIBRARIES}
)
add_dependencies(b_orbcamera_nodelet_test ${PROJECT_NAME}_generate_messages_cpp)

###########
## Install ##
###########

## Mark executables and/or libraries for installation
 install(TARGETS b_orbcamera_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 install(TARGETS b_orbcamera_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )
 
 install(DIRECTORY
   launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
 
  install(FILES
   ../../tools/NiTE-Linux-x64-2.2/Redist/libNiTE2.so
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )
 
   install(DIRECTORY
   ../../tools/NiTE-Linux-x64-2.2/Redist/NiTE2
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )

  install(FILES
   ../../tools/OpenNI-Linux-x64-2.2/RedistforNite/libOpenNI2.so
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )
 
  install(DIRECTORY
   ../../tools/OpenNI-Linux-x64-2.2/RedistforNite/OpenNI2
   DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 )  
