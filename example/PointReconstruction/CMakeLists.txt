

add_executable(PointReconstruction WIN32
    PointCloudWidget.h
    PointCloudWidget.cpp
    widget.h
    widget.cpp
    PointReconstruction.cpp
)

set_target_properties(PointReconstruction PROPERTIES AUTOMOC ON)       
set_target_properties(PointReconstruction PROPERTIES AUTOUIC ON)       
set_target_properties(PointReconstruction PROPERTIES AUTORCC ON)        

target_link_libraries(PointReconstruction PRIVATE FSL::CORE)
target_link_libraries(PointReconstruction PRIVATE ${OpenCV_LIBS})
target_link_libraries(PointReconstruction PRIVATE Qt5::Core)
target_link_libraries(PointReconstruction PRIVATE Qt5::Widgets) 
target_link_libraries(PointReconstruction PRIVATE Qt5::Gui) 



install(TARGETS PointReconstruction
        RUNTIME DESTINATION bin)

if(WIN32 AND Qt5_FOUND)
    get_filename_component(QT5_CMAKE_DIR "${Qt5_DIR}" PATH)
    get_filename_component(QT5_LIB_DIR "${QT5_CMAKE_DIR}" PATH)
    get_filename_component(QT5_ROOT_DIR "${QT5_LIB_DIR}" PATH)
    set(QT_PLUGINS_PLATFORM_DIR "${QT5_ROOT_DIR}/plugins/platforms")
    message(STATUS "🔍 Qt platform plugins directory: ${QT_PLUGINS_PLATFORM_DIR}")

    #--- platform dll ---
    set(QT_PLATFORM_DLL "${QT_PLUGINS_PLATFORM_DIR}/qwindows.dll")
    if(EXISTS "${QT_PLATFORM_DLL}")
        install(FILES "${QT_PLATFORM_DLL}"
                DESTINATION bin/platforms)
        message(STATUS "✅ Qt platform plugin qwindows.dll will be installed to bin/platforms")
    else()
        message(WARNING "⚠️ Qt platform plugin not found: ${QT_PLATFORM_DLL}")
    endif()

    #--- Qt5Core.dll ---
    set(QT_CORE_DLL "${QT5_ROOT_DIR}/bin/Qt5Core.dll")
    if(EXISTS "${QT_CORE_DLL}")
        install(FILES "${QT_CORE_DLL}"
                DESTINATION bin)
        message(STATUS "✅ Qt5Core.dll will be installed to bin")
    else()
        message(WARNING "⚠️ Qt5Core.dll not found: ${QT_CORE_DLL}")
    endif()

    #--- Qt5Widgets.dll ---
    set(QT_WIDGETS_DLL "${QT5_ROOT_DIR}/bin/Qt5Widgets.dll")
    if(EXISTS "${QT_WIDGETS_DLL}")
        install(FILES "${QT_WIDGETS_DLL}"
                DESTINATION bin)
        message(STATUS "✅ Qt5Widgets.dll will be installed to bin")
    else()
        message(WARNING "⚠️ Qt5Widgets.dll not found: ${QT_WIDGETS_DLL}")
    endif()

    #--- Qt5Gui.dll ---
    set(QT_GUI_DLL "${QT5_ROOT_DIR}/bin/Qt5Gui.dll")
    if(EXISTS "${QT_GUI_DLL}")
        install(FILES "${QT_GUI_DLL}"
                DESTINATION bin)
        message(STATUS "✅ Qt5Gui.dll will be installed to bin")
    else()
        message(WARNING "⚠️ Qt5Gui.dll not found: ${QT_GUI_DLL}")
    endif()

    #--- Qt5OpenGL.dll ---
    set(QT_OPENGL_DLLS
        libGLESv2.dll
        libEGL.dll
        D3Dcompiler_47.dll
        opengl32sw.dll
    )

    foreach(dll ${QT_OPENGL_DLLS})
        set(DLL_PATH "${QT5_ROOT_DIR}/bin/${dll}")
        if(EXISTS "${DLL_PATH}")
            install(FILES "${DLL_PATH}" DESTINATION bin)
            message(STATUS "✅ ${dll} installed to bin")
        else()
            message(WARNING "⚠️ ${dll} not found: ${DLL_PATH}")
        endif()
    endforeach()

endif()