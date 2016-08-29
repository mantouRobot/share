find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)

#OpenCV的头文件，库文件位置不需要再显式指出，只需要目标链接时包含
#PCL的头文件，库文件位置需要显示指出
#add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LINK_DIRS})

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})


