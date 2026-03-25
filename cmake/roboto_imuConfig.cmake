# RobotoImuConfig.cmake
#
# Config file for RobotoImu
#
# Defines the IMPORTED target: roboto_imu::roboto_imu

# Check if we already have the target
if(TARGET roboto_imu::roboto_imu)
    return()
endif()

# Compute the installation prefix relative to this file
get_filename_component(CURRENT_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
# We install to ${PREFIX}/lib/cmake/RobotoImu/, so we go up 3 levels to get PREFIX
get_filename_component(_INSTALL_PREFIX "${CURRENT_DIR}/../../../" ABSOLUTE)

set(RobotoImu_INCLUDE_DIR "${_INSTALL_PREFIX}/include")
set(RobotoImu_LIB_DIR     "${_INSTALL_PREFIX}/lib")

# Find dependencies
include(CMakeFindDependencyMacro)
find_dependency(fmt)
find_dependency(spdlog)

# Helper function to find and add libraries
function(_add_imported_lib _lib_name)
    find_library(LIB_${_lib_name} 
        NAMES ${_lib_name}
        PATHS ${RobotoImu_LIB_DIR}
        NO_DEFAULT_PATH
    )
    if(LIB_${_lib_name})
        list(APPEND RobotoImu_LIBRARIES ${LIB_${_lib_name}})
        set(RobotoImu_LIBRARIES ${RobotoImu_LIBRARIES} PARENT_SCOPE)
    else()
        message(FATAL_ERROR "Could not find roboto_imu library: ${_lib_name} in ${RobotoImu_LIB_DIR}")
    endif()
endfunction()

# Find all component libraries
set(RobotoImu_LIBRARIES "")
_add_imported_lib(imu)
_add_imported_lib(imu_protocol)
_add_imported_lib(hipnuc_imu)

# Create the INTERFACE target
add_library(roboto_imu::roboto_imu INTERFACE IMPORTED)

target_include_directories(roboto_imu::roboto_imu INTERFACE ${RobotoImu_INCLUDE_DIR})
target_link_libraries(roboto_imu::roboto_imu INTERFACE 
    ${RobotoImu_LIBRARIES}
    fmt::fmt
    spdlog::spdlog
)

message(STATUS "Found RobotoImu: ${_INSTALL_PREFIX}")
