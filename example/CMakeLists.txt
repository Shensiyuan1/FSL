
add_subdirectory(StandPhaseShift)
add_subdirectory(LutPhaseShift)
add_subdirectory(HierarchicalUnwrap)


install(DIRECTORY calib DESTINATION bin)
install(DIRECTORY img   DESTINATION bin)