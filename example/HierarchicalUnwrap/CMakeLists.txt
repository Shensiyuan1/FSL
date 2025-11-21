add_executable(HierarchicalUnwrap
    HierarchicalUnwrap.cpp
)

if (CORE_STATIC)
    target_compile_definitions(HierarchicalUnwrap PRIVATE FSL_STATIC)
endif()

target_link_libraries(HierarchicalUnwrap PRIVATE ${OpenCV_LIBS})
target_link_libraries(HierarchicalUnwrap PRIVATE FSL::CORE)

install(TARGETS HierarchicalUnwrap
        RUNTIME DESTINATION bin)
