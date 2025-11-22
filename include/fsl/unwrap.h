#ifndef UNWRAP_H
#define UNWRAP_H 

#include <unordered_map>
#include <fsl/core.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <iostream>


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



    std::pair<Phase,std::unordered_map<int,int>> segphase(const Phase& phase,const std::vector<double>& mask,std::string mode = "mask",int thresh = 3);

}


#endif