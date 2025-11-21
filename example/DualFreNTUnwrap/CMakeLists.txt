
add_executable(DualFreNTUnwrap DualFreNTUnwrap.cpp)


if (CORE_STATIC)
    target_compile_definitions(HierarchicalUnwrap PRIVATE FSL_STATIC)
endif()

target_link_libraries(DualFreNTUnwrap PRIVATE FSL::CORE)
target_link_libraries(DualFreNTUnwrap PRIVATE ${OpenCV_LIBS})

install(TARGETS DualFreNTUnwrap
        RUNTIME DESTINATION bin)

