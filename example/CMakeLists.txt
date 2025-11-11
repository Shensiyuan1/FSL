
message(STATUS "🔍 Checking for OpenCV in example module...")
find_package(OpenCV QUIET)

if(NOT OpenCV_FOUND)
    message(WARNING "⚠️ OpenCV not found. Skipping example module build.")
    return()  # ❗ 不构建 example 子模块，直接退出
endif()

message(STATUS "✅ OpenCV found: ${OpenCV_DIR}")
message(STATUS "    Version: ${OpenCV_VERSION}")
message(STATUS "    Include: ${OpenCV_INCLUDE_DIRS}")

add_subdirectory(StandPhaseShift)
add_subdirectory(LutPhaseShift)
add_subdirectory(HierarchicalUnwrap)


install(DIRECTORY calib DESTINATION bin)
install(DIRECTORY img   DESTINATION bin)

# --- Installation OpenCV runtime DLL on windows ---
if(WIN32)
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