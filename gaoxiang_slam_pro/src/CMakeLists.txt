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
