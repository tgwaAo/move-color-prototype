cmake_minimum_required (VERSION 3.0)
project (Move_Color_Prototype CXX)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB SOURCES
    "Code/*.h"
    "Code/*.cpp"
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)
