add_executable(HierarchicalUnwrap
    HierarchicalUnwrap.cpp
)

target_link_libraries(HierarchicalUnwrap PRIVATE ${OpenCV_LIBS})
target_link_libraries(HierarchicalUnwrap PRIVATE FSL::CORE)

install(TARGETS HierarchicalUnwrap
        RUNTIME DESTINATION bin)
