
add_executable(StandPhaseShift
    StandPhaseShift.cpp
)

if (CORE_STATIC)
    target_compile_definitions(StandPhaseShift PRIVATE FSL_STATIC)
endif()

target_link_libraries(StandPhaseShift PRIVATE ${OpenCV_LIBS})
target_link_libraries(StandPhaseShift PRIVATE FSL::CORE)

install(TARGETS StandPhaseShift
        RUNTIME DESTINATION bin)