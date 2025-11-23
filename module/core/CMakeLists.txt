# core/CMakeLists.txt

set(CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/unwrap.cpp
)

if(CORE_STATIC)
    add_library(core STATIC ${CORE_SOURCES})
    target_compile_definitions(core PRIVATE FSL_STATIC)
    set_target_properties(core PROPERTIES OUTPUT_NAME "FSL_CORE")  

    target_include_directories(core
        PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    install(TARGETS core
        EXPORT FSLTargets
        ARCHIVE DESTINATION lib
    )
else()
    add_library(core SHARED ${CORE_SOURCES})
    target_compile_definitions(core PRIVATE FSL_CORE_EXPORTS)
    set_target_properties(core PROPERTIES OUTPUT_NAME "FSL_CORE")

    target_include_directories(core
        PUBLIC
            $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>
    )

    install(TARGETS core
        EXPORT FSLTargets
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
    )
endif()

add_library(FSL::CORE ALIAS core)