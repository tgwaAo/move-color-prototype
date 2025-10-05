cmake_minimum_required (VERSION 3.0)
project(move-color-prototype)
set(EXE_NAME "move_color_prototype")
set(LIB_NAME "move_color_prototype_lib")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(
    "${CMAKE_SOURCE_DIR}/include"
)

add_library(${LIB_NAME} include/ParticleWeighting.h include/CalibrationHandler.h include/CircleHandler.h src/ParticleWeighting.cpp src/CalibrationHandler.cpp src/CircleHandler.cpp)
target_compile_options(${LIB_NAME} PRIVATE -Wall -Wextra)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME} examples/main.cpp)
option(BUILD_EXAMPLES "Build the examples" ON)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME ${EXE_NAME})

if(BUILD_EXAMPLES)
    target_link_libraries(${PROJECT_NAME} PRIVATE  ${LIB_NAME})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE  ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} PRIVATE  Eigen3::Eigen)

