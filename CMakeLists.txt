cmake_minimum_required (VERSION 3.0)
project (Move_Color_Prototype CXX)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB SOURCES
    "Code/header/*.h"
    "Code/src/*.cpp"
)

include_directories(
    "${CMAKE_SOURCE_DIR}/Code/include"
    "${CMAKE_SOURCE_DIR}/Code/src"
)

add_executable(${PROJECT_NAME} ${SOURCES}
    Code/main.cpp
    Code/include/CalibrationHandler.h Code/include/CircleHandler.h Code/include/ParticleWeighting.h)

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)
