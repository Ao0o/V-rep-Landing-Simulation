project(SolvePnP)
cmake_minimum_required(VERSION 2.8)

find_package( OpenCV REQUIRED )

#Show Images?
#ADD_DEFINITIONS(-DSHOW_DEBUG_IMAGES)
#

link_libraries (pthread)
link_directories(
/home/ubuntu/lib
/home/noah/lib
)

INCLUDE_DIRECTORIES( #相关头文件的目录
${PROJET_SOURCE_DIR}
/home/ubuntu/include
/home/noah/include
)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST})
target_link_libraries( 
    ${PROJECT_NAME} 
    ${OpenCV_LIBS} 
    #dcam
    )

add_library(camerareader SHARED
CameraCalibration.cpp
CameraCalibration.hpp
CameraReader.cpp
CameraReader.h
GeometryTypes.cpp
GeometryTypes.hpp
Marker.cpp
Marker.hpp
MarkerDetector.cpp
MarkerDetector.hpp
TinyLA.cpp
TinyLA.hpp
)
target_link_libraries(
    camerareader 
    ${OpenCV_LIBS} 
    pthread 
    #dcam
    )
INSTALL(TARGETS camerareader LIBRARY DESTINATION lib)
INSTALL(FILES CameraReader.h Marker.hpp MarkerDetector.hpp CameraCalibration.hpp MarkerDetector.hpp TinyLA.hpp  GeometryTypes.hpp BGRAVideoFrame.h DESTINATION include/camerareader)
set( CMAKE_BUILD_TYPE Debug )


