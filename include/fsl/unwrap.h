#ifndef UNWRAP_H
#define UNWRAP_H 

#include <unordered_map>
#include <fsl/core.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <iostream>
#include <queue>


#if defined(FSL_STATIC)
    #define FSL_CORE_API
#elif defined(_WIN32)
    #if defined(FSL_CORE_EXPORTS)
        #define FSL_CORE_API __declspec(dllexport)
    #else
        #define FSL_CORE_API __declspec(dllimport)
    #endif
#else
    #define FSL_CORE_API
#endif



namespace Fringe
{



    // =======================================
    // Space phase unwrap
    // =======================================
    FSL_CORE_API std::pair<Phase,std::unordered_map<int,int>> segphase(const Phase& phase,const std::vector<double>& mask,std::string mode = "mask",int thresh = 3,double seg_thresh = M_PI);

    FSL_CORE_API void  optPhaseBYvalue(Phase &phase,Phase& value,Phase& segment,const std::unordered_map<int,int>& lut);

}


#endif