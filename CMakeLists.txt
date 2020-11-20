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

find_package(Boost 1.71 COMPONENTS program_options thread system iostreams filesystem REQUIRED)
message("Boost_INCLUDE_DIR: ${Boost_INCLUDE_DIR}")
message("Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message("Boost_LIBRARIES: ${Boost_LIBRARIES}")


# Include the vendor directory for the extern libraries
include_directories(src ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

# add the executable
set(SRC
    src/main.cpp
    src/nvmreader.cpp
    src/detectionsreader.cpp
    src/bboxpointsextractor.cpp
    src/pointcloud.cpp
    )

add_executable(GTExtractor ${SRC})
target_link_libraries(GTExtractor ${Boost_LIBRARIES})

