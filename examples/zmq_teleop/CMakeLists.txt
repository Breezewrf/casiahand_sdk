cmake_minimum_required(VERSION 3.16)
project(casia_zmq_teleop LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Reuse the exact vendored SDK sources used by the Python package without
# requiring pybind11 or an installed CASIA library.
set(CASIAHAND_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/../.." casiahand_sdk EXCLUDE_FROM_ALL)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED IMPORTED_TARGET libzmq)

add_executable(casia_zmq_teleop main.cpp)
target_link_libraries(casia_zmq_teleop PRIVATE casia_hand_m_sdk PkgConfig::ZMQ)
