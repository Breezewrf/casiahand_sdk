# 设置库的版本（这里假设版本为 1.0.0，你可根据实际情况修改）
set(serial_VERSION_MAJOR 1)
set(serial_VERSION_MINOR 0)
set(serial_VERSION_PATCH 0)
set(serial_VERSION "${serial_VERSION_MAJOR}.${serial_VERSION_MINOR}.${serial_VERSION_PATCH}")

# 查找库文件
find_library(serial_LIBRARY
             NAMES serial
             PATHS /usr/local/lib
             NO_DEFAULT_PATH)

# 查找头文件目录
set(serial_INCLUDE_DIR /usr/local/include/serial)

# 检查是否找到库和头文件
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(serial
                                  REQUIRED_VARS serial_LIBRARY serial_INCLUDE_DIR
                                  VERSION_VAR serial_VERSION)

# 如果找到，设置目标
if(serial_FOUND)
    set(serial_LIBRARIES ${serial_LIBRARY})
    set(serial_INCLUDE_DIRS ${serial_INCLUDE_DIR})
    if(NOT TARGET serial::serial)
        add_library(serial::serial UNKNOWN IMPORTED)
        set_target_properties(serial::serial PROPERTIES
                              IMPORTED_LOCATION "${serial_LIBRARY}"
                              INTERFACE_INCLUDE_DIRECTORIES "${serial_INCLUDE_DIR}")
    endif()
endif()