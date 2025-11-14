cmake_minimum_required(VERSION 3.18)
#cmake_policy(SET CMP0091 NEW)
project(FSL VERSION 0.0.1 LANGUAGES CXX)

# =======================================================
# --- Options (User Configurable Modules) ---
# =======================================================
option(BUILD_STATIC_LIBS "Build static libraries for all modules" OFF)
option(CORE_STATIC "Build core library as static library" ${BUILD_STATIC_LIBS})
option(CUDA_STATIC "Build CUDA libraries as static libraries" ${BUILD_STATIC_LIBS})

option(USE_CUDA "Build with CUDA support" ON)
# =======================================================
#--- 3rdparty ---
# =======================================================
option(USE_QT "Enable Qt-based UI or module integration" OFF)
option(USE_OPENCV "Enable OpenCV-based example and functionality" OFF)
option(USE_EXAMPLE "Enable example programs" ON)

# =======================================================
# --- Compiler settings (MUST be after project()) ---
# =======================================================
#set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")  #cuda static test use 
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =======================================================
# --- core library ---
# =======================================================
add_subdirectory(module/core)

# =======================================================
# --- cuda library ---
# =======================================================
if(USE_CUDA)
    find_package(CUDAToolkit QUIET) 
    find_program(CUDA_COMPILER nvcc QUIET) 
    if(CUDAToolkit_FOUND)
        enable_language(CUDA)
        message(STATUS "✅ CUDA found, building CUDA module")
        add_subdirectory(module/cuda)
    else()
        message(WARNING "⚠️ CUDA not found, skipping CUDA module")
    endif()
endif()

# =======================================================
# --- example programs ---
# =======================================================
if(USE_EXAMPLE)
    add_subdirectory(example)
endif()

# =======================================================
# --- Installation public headers ---
# =======================================================
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include)

# =======================================================        
# --- Installation default path ---
# =======================================================
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Default install path" FORCE)
endif()

# =======================================================
# --- cmake package config ---
# =======================================================
include(CMakePackageConfigHelpers)

install(EXPORT FSLTargets
    FILE FSLTargets.cmake
    NAMESPACE FSL::   
    DESTINATION lib/cmake/FSL
)

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/FSLConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/FSLConfig.cmake"
    INSTALL_DESTINATION lib/cmake/FSL
)

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/FSLConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/FSLConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/FSLConfigVersion.cmake"
    DESTINATION lib/cmake/FSL
)
