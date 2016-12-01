find_package(OpenCV)
add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBRARIES})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
