cmake_minimum_required(VERSION 3.5)
project(vicon_zmq_standalone VERSION 1.0.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -g)
endif()

# Get the project root directory
get_filename_component(PROJECT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" ABSOLUTE)

# Include directories
include_directories(
  ${PROJECT_ROOT}/include
  ${PROJECT_ROOT}/libs
)

# Find Boost
find_package(Boost REQUIRED COMPONENTS thread system chrono timer)

# Find ZMQ using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

# Vicon SDK library path
set(VICON_SDK_DIR ${PROJECT_ROOT}/libs)
set(VICON_SDK_LIB ${VICON_SDK_DIR}/libViconDataStreamSDK_CPP.so)

# Check if Vicon SDK library exists
if(NOT EXISTS ${VICON_SDK_LIB})
    message(WARNING "Vicon SDK library not found at ${VICON_SDK_LIB}")
    message(WARNING "Please copy the Vicon SDK libraries to the libs/ directory")
endif()

# Add executable
add_executable(vicon_zmq_client 
    src/zmq_communicator.cpp 
    src/zmq_publisher.cpp 
    src/main.cpp
)

target_include_directories(vicon_zmq_client PRIVATE 
    ${ZMQ_INCLUDE_DIRS}
    ${PROJECT_ROOT}/include
)

target_link_libraries(vicon_zmq_client 
    ${VICON_SDK_LIB}
    ${Boost_LIBRARIES}
    ${ZMQ_LIBRARIES}
    pthread
)

target_compile_options(vicon_zmq_client PRIVATE ${ZMQ_CFLAGS_OTHER})

# Set RPATH so the executable can find the Vicon SDK libraries at runtime
set_target_properties(vicon_zmq_client PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/vicon_zmq:/usr/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Install target
install(TARGETS vicon_zmq_client DESTINATION bin)

# Install headers
install(DIRECTORY include/vicon_zmq DESTINATION include)

# Install libraries
install(DIRECTORY libs/ DESTINATION lib/vicon_zmq)

# Install Python subscriber script
install(PROGRAMS scripts/vicon_zmq_subscriber.py DESTINATION bin)

