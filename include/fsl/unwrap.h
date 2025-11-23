#ifndef UNWRAP_H
#define UNWRAP_H 

#include <unordered_map>
#include "fsl/utils.h"
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
    // Temporal phase unwrap
    // =======================================
    FSL_CORE_API Phase HierarchicalUnwrap(const std::vector<Phase>& wraps,double* ratio,const int ratio_count);

    FSL_CORE_API std::unordered_map<int, int> DualFreNumberTheoreticalLUT(int highFre,int lowFre);

    FSL_CORE_API Phase DualFreNumberTheoreticalUnwrap(const std::vector<Phase>& wraps, const std::unordered_map<int, int>& lut,const int highFre,const int lowFre);

    FSL_CORE_API std::pair<Phase,Phase> DualFreNumberTheoreticalUnwrapAdValue(const std::vector<Phase>& wraps, const std::unordered_map<int, int>& lut,const int highFre,const int lowFre);

    // =======================================
    // Space phase unwrap
    // =======================================
    FSL_CORE_API std::pair<Phase,std::unordered_map<int,int>> segphase(const Phase& phase,const std::vector<double>& mask,std::string mode = "mask",int thresh = 3,double seg_thresh = M_PI);

    FSL_CORE_API void  optPhaseBYvalue(Phase &phase,Phase& value,Phase& segment,const std::unordered_map<int,int>& lut);

}


#endif