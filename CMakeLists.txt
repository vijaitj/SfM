cmake_minimum_required(VERSION 2.6)

project(SFM)

#set(CMAKE_C_FLAGS_DEBUG "-g -DDEBUG")
#set(CMAKE_CXX_FLAGS_DEBUG "-g -DDEBUG")

# Find Packages
find_package(OpenCV REQUIRED core imgproc highgui features2d calib3d imgcodecs )
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED system filesystem)
find_package(Ceres REQUIRED)

# Include Directories
set(INCLDS "${OpenCV_INCLUDE_DIRS}")
set(INCLDS "${INCLDS}" "${EIGEN3_INCLUDE_DIRS}")
include_directories("${INCLDS}")
include_directories(include)
include_directories(${CERES_INCLUDE_DIRS})
#message(STATUS "${INCLDS}")

# Linker Libs
set(LIBS "${OpenCV_LIBRARIES}")
set(LIBS "${LIBS}" Eigen3::Eigen)
set(LIBS "${LIBS}" ${Boost_LIBRARIES})
set(LIBS "${LIBS}" ${CERES_LIBRARIES})
# set(LIBS "${LIBS}" gtsam)
#message(STATUS "${LIBS}")

# Source Files to create executable
file(GLOB SOURCES "src/*.cpp")
add_executable(SFM ${SOURCES})
set_property(TARGET SFM PROPERTY CXX_STANDARD 14)

target_link_libraries(SFM "${LIBS}")



