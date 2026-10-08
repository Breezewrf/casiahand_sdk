cmake_minimum_required(VERSION 3.16)
project(casiahand_sdk VERSION 0.1.0 LANGUAGES CXX)

if(NOT UNIX OR APPLE)
  message(FATAL_ERROR "casiahand-sdk currently supports Linux only")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)

option(CASIAHAND_BUILD_PYTHON "Build the Python extension" ON)

set(SERIAL_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/sdk/third_party/serial")
add_library(casia_serial STATIC
  "${SERIAL_ROOT}/src/serial.cc"
  "${SERIAL_ROOT}/src/impl/unix.cc"
  "${SERIAL_ROOT}/src/impl/list_ports/list_ports_linux.cc")
target_include_directories(casia_serial PUBLIC "${SERIAL_ROOT}/include")
target_link_libraries(casia_serial PUBLIC Threads::Threads rt)

add_library(casia_hand_m_sdk STATIC
  sdk/src/casia_grasp_control.cpp
  sdk/src/casia_grasp_probuf.cpp
  sdk/src/casia_hand_m.cpp)
target_include_directories(casia_hand_m_sdk PUBLIC sdk/include)
target_link_libraries(casia_hand_m_sdk PUBLIC casia_serial Threads::Threads)

if(CASIAHAND_BUILD_PYTHON)
  find_package(Python REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(pybind11 CONFIG REQUIRED)
  python_add_library(_native MODULE src/py_binding.cpp WITH_SOABI)
  target_link_libraries(_native PRIVATE pybind11::headers casia_hand_m_sdk)
  install(TARGETS _native DESTINATION casiahand_sdk)
endif()

option(CASIAHAND_BUILD_TESTS "Build hardware-free native transport tests" OFF)
if(CASIAHAND_BUILD_TESTS)
  enable_testing()
  add_executable(casia_startup_deadline_test tests/native_startup_deadline.cpp)
  target_link_libraries(casia_startup_deadline_test PRIVATE casia_hand_m_sdk util)
  add_test(NAME casia_startup_deadline COMMAND casia_startup_deadline_test)
  set_tests_properties(casia_startup_deadline PROPERTIES TIMEOUT 5)
endif()
