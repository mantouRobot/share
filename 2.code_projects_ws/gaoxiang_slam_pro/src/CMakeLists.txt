#添加G2O的依赖
#因为G2O不是常用库，要添加它的findg2o.cmake文件
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O)
#CSparse
find_package(CSparse)
include_directories(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

find_package(PCL REQUIRED )
find_package(OpenCV REQUIRED)

#OpenCV的头文件，库文件位置不需要再显式指出，只需要目标链接时包含
#PCL的头文件，库文件位置需要显示指出
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LINK_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_library(slamBase slamBase.cpp)
target_link_libraries(slamBase ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(feature_detect feature_detect.cpp)
target_link_libraries(feature_detect ${OpenCV_LIBS} slamBase)

add_executable(joint_point_cloud joint_point_cloud.cpp)
target_link_libraries(joint_point_cloud ${OpenCV_LIBS} ${PCL_LIBRARIES} slamBase)

add_executable(vo vo.cpp)
target_link_libraries(vo ${OpenCV_LIBS} ${PCL_LIBRARIES} slamBase)

add_executable(slam_end slam_end.cpp)
target_link_libraries(slam_end ${OpenCV_LIBS} ${PCL_LIBRARIES} slamBase
    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})


#add_compile_options(-std=c++11)
## 增加一个可执行的二进制
##ADD_EXECUTABLE( main main.cpp )

## 增加PCL库的依赖
#FIND_PACKAGE( PCL REQUIRED COMPONENTS common io visualization filters )

## 增加opencv的依赖
#FIND_PACKAGE( OpenCV REQUIRED )

## 添加头文件和库文件
#ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
#INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
#LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

## 添加g2o的依赖
## 因为g2o不是常用库，要添加它的findg2o.cmake文件
#LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
#SET( G2O_ROOT /usr/local/include/g2o )
#FIND_PACKAGE( G2O )
## CSparse
#FIND_PACKAGE( CSparse )
#INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

##ADD_EXECUTABLE( generate_pointcloud generatePointCloud.cpp )
##TARGET_LINK_LIBRARIES( generate_pointcloud ${OpenCV_LIBS}
##    ${PCL_LIBRARIES} )

#ADD_LIBRARY( slamBase slamBase.cpp )
#TARGET_LINK_LIBRARIES( slamBase
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES} )

#ADD_EXECUTABLE( feature_detect feature_detect.cpp )
#TARGET_LINK_LIBRARIES( feature_detect
#    slamBase
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES} )

#ADD_EXECUTABLE( joint_point_cloud joint_point_cloud.cpp)
#TARGET_LINK_LIBRARIES( joint_point_cloud
#    slamBase
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES} )

#ADD_EXECUTABLE( vo vo.cpp)
#TARGET_LINK_LIBRARIES( vo
#    slamBase
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES} )

#ADD_EXECUTABLE( slam_end slam_end.cpp )
#TARGET_LINK_LIBRARIES( slam_end

#    slamBase
#    ${OpenCV_LIBS}
#    ${PCL_LIBRARIES}
#    g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension ${CSPARSE_LIBRARY})

#add_executable(main main.cpp)
#target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})
