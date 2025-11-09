#ifndef CUDA_H
#define CUDA_H

#include "cuda_runtime.h" 
#include "fsl/core.h"
#include <chrono>  

#if defined(FSL_STATIC_DEFINE)
    #define FSL_CUDA_API
#elif defined(_WIN32)
    #if defined(FSL_CUDA_EXPORTS)
        #define FSL_CUDA_API __declspec(dllexport)
    #else
        #define FSL_CUDA_API __declspec(dllimport)
    #endif
#else
    #define FSL_CUDA_API
#endif

#define CUDA_ACC_BLOCK_SIZE 256

namespace Fringe
{
    
   FSL_CUDA_API Phase StandardPhaseShiftGPU(const std::vector<RawImg>& imgs,bool cacuB=false);




}

#endif 