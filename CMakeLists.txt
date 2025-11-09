cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

# --- Options ---
option(BUILD_STATIC_LIBS "Build static libraries for all modules" OFF)
option(CORE_STATIC "Build core library as static library" ${BUILD_STATIC_LIBS})
option(USE_CUDA "Build with CUDA support" ON)
option(CUDA_STATIC "Build CUDA libraries as static libraries" ${BUILD_STATIC_LIBS})

# --- Project declaration ---
if(USE_CUDA)
    project(FSL VERSION 0.0.1 LANGUAGES CXX CUDA)
else()
    project(FSL VERSION 0.0.1 LANGUAGES CXX)
endif()

# --- Compiler settings (MUST be after project()) ---
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- core library ---
add_subdirectory(core)

# --- cuda library ---
if(USE_CUDA)
    add_subdirectory(cuda)
endif()

# --- example programs ---
find_package(OpenCV QUIET)

if(OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_DIR}")
    add_subdirectory(example)
else()
    message(STATUS "OpenCV not found")
endif()

# --- Installation public headers ---
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
        DESTINATION include)

# --- Installation OpenCV runtime DLL on windows ---
if(WIN32 AND OpenCV_FOUND)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(OPENCV_ARCH "x64")
    else()
        set(OPENCV_ARCH "x86")
    endif()

    if(MSVC_VERSION GREATER_EQUAL 1930)  # VS 2022
        set(OPENCV_VC_DIR "vc16")
    elseif(MSVC_VERSION GREATER_EQUAL 1920)  # VS 2019
        set(OPENCV_VC_DIR "vc16")
    elseif(MSVC_VERSION GREATER_EQUAL 1910)  # VS 2017
        set(OPENCV_VC_DIR "vc15")
    elseif(MSVC_VERSION GREATER_EQUAL 1900)  # VS 2015
        set(OPENCV_VC_DIR "vc14")
    else()
        set(OPENCV_VC_DIR "vc14")
    endif()

    string(REPLACE "." "" OpenCV_VERSION_CLEAN "${OpenCV_VERSION}")
    set(OPENCV_WORLD_DLL "${OpenCV_DIR}/${OPENCV_ARCH}/${OPENCV_VC_DIR}/bin/opencv_world${OpenCV_VERSION_CLEAN}.dll")

    if(EXISTS "${OPENCV_WORLD_DLL}")
        install(FILES "${OPENCV_WORLD_DLL}"
                DESTINATION bin
                COMPONENT Runtime)
    else()
        message(WARNING "OpenCV world DLL not found at: ${OPENCV_WORLD_DLL}")
    endif()
endif()

# --- Installation default path ---
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Default install path" FORCE)
endif()

# --- cmake package config ---
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
