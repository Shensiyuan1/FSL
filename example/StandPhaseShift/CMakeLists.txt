
add_executable(StandPhaseShift
    StandPhaseShift.cpp
)

target_link_libraries(StandPhaseShift PRIVATE ${OpenCV_LIBS})
target_link_libraries(StandPhaseShift PRIVATE FSL::CORE)

install(TARGETS StandPhaseShift
        RUNTIME DESTINATION bin)