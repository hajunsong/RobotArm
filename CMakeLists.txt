cmake_minimum_required(VERSION 3.0.0)

project(RobotArm)

set(CMAKE_CXX_STANDARD 11)

IF(CMAKE_BUILD_TYPE MATCHES Debug)
  message("Debug build.")
ELSEIF(CMAKE_BUILD_TYPE MATCHES Release)
  message("Release build.")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")
ELSE()
    message("CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
ENDIF()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(SRCS
    src/numerical.cpp
    src/robotarm.cpp
)

set(LIB_NAME robotarmik)

add_library(
    ${LIB_NAME} SHARED ${SRCS}
)

add_executable(${PROJECT_NAME} main.cpp ${SRCS})
