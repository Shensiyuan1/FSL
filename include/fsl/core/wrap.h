#ifndef WRAP_H
#define WRAP_H

#include "utils.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <iomanip>



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


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
    
    FSL_CORE_API Phase Standardphaseshift(const std::vector<RawImg>& imgs,bool cacuB=false);

    FSL_CORE_API Phase MD2Phase(const Phase& M,const Phase& D,bool cacuB=false);

    FSL_CORE_API bool ThreeStepLUTGenerate(LUTP& lut);

    FSL_CORE_API bool FourStepLUTGenerate(LUTP& lut);

    FSL_CORE_API Phase LUT3StepPhaseshift(const std::vector<RawImg>& imgs,bool cacuB,const LUTP& lut);

    FSL_CORE_API Phase LUT4StepPhaseshift(const std::vector<RawImg>& imgs,bool cacuB,const LUTP& lut);

    FSL_CORE_API Phase CacuQuality(const Phase& wrap,bool fourOeight=false);

}


#endif