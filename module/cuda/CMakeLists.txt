set(CMAKE_CUDA_ARCHITECTURES "50;60;61;75;86;89;90")

set(CUDA_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/cuda.cu
)

if(CUDA_STATIC)
    add_library(cuda STATIC ${CUDA_SOURCES})
    target_compile_definitions(cuda PRIVATE FSLCUDA_STATIC)
else()
    add_library(cuda SHARED ${CUDA_SOURCES})
    target_compile_definitions(cuda PRIVATE FSL_STATIC_DEFINE)
endif()

set_target_properties(cuda PROPERTIES
    OUTPUT_NAME "FSL_CUDA"
    CUDA_RUNTIME_LIBRARY Shared
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
)

target_include_directories(cuda
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(cuda PUBLIC CUDA::cudart)

install(TARGETS cuda
    EXPORT FSLTargets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

add_library(FSL::CUDA ALIAS cuda)