cmake_minimum_required(VERSION 3.10)

# set the project name
project(GTExtractor)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(Eigen3 REQUIRED)
message("EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message("EIGEN3_INCLUDE_DIRS: ${EIGEN3_INCLUDE_DIRS}")
message("EIGEN3_LIBRARIES: ${EIGEN3_LIBRARIES}")

# Include the vendor directory for the extern libraries
include_directories(src ${EIGEN3_INCLUDE_DIR})

# add the executable
set(SRC
    src/main.cpp
    src/nvmreader.cpp
)

add_executable(GTExtractor ${SRC})
#target_link_libraries(GTExtractor)

