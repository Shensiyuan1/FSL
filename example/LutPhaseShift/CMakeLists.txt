
add_executable(LutPhaseShift
    LutPhaseShift.cpp
)

if (CORE_STATIC)
    target_compile_definitions(LutPhaseShift PRIVATE FSL_STATIC)
endif()


target_link_libraries(LutPhaseShift PRIVATE ${OpenCV_LIBS})
target_link_libraries(LutPhaseShift PRIVATE FSL::CORE)

install(TARGETS LutPhaseShift
        RUNTIME DESTINATION bin)
