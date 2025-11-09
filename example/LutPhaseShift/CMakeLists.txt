
add_executable(LutPhaseShift
    LutPhaseShift.cpp
)

target_link_libraries(LutPhaseShift PRIVATE ${OpenCV_LIBS})
target_link_libraries(LutPhaseShift PRIVATE FSL::CORE)

install(TARGETS LutPhaseShift
        RUNTIME DESTINATION bin)
