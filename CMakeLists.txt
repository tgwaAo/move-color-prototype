cmake_minimum_required (VERSION 3.0)
project (move-color-prototype CXX)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB SOURCES
    "include/*.h"
    "src/*.cpp"
)

include_directories(
    "${CMAKE_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/src"
)

add_executable(${PROJECT_NAME} ${SOURCES}
    examples/main.cpp
    include/CalibrationHandler.h include/CircleHandler.h include/ParticleWeighting.h)

target_link_libraries( ${PROJECT_NAME} ${OpenCV_LIBS} )
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)
