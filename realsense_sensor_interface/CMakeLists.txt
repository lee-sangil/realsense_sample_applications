# project
SET(PROJECT_NAME librealsense)
PROJECT(${PROJECT_NAME} CXX C)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(SENSE_LIB_DIR "/usr/local/lib/" CACHE PATH " ")


# Add plain cmake packages
FIND_PACKAGE(OpenCV REQUIRED)


# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${OpenCV_INCLUDE_DIRS}
)


# Set link libraries
LIST(APPEND LINK_LIBS
  ${OpenCV_LIBS}
  ${SENSE_LIB_DIR}/librealsense.so
)


# Set sourcefiles
LIST(APPEND SOURCEFILES
  src/core/main_script.cpp)


# Set build flags, set ARM_ARCHITECTURE environment variable on Odroid
set(CMAKE_CXX_FLAGS "-msse3")
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m64 -Ofast -flto -march=native")


################################################################################
# TESTS

ADD_EXECUTABLE(librealsense ${SOURCEFILES})
TARGET_LINK_LIBRARIES(librealsense ${LINK_LIBS})
