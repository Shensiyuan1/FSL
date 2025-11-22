#ifndef NumberTheoreticalUnwrap_h
#define NumberTheoreticalUnwrap_h

#include <unordered_map>
#include <fsl/core.h>
#include <cmath>
#include <cstdlib>
#include <string>
#include <iostream>

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

    



    FSL_CORE_API std::unordered_map<int, int> DualFreNumberTheoreticalLUT(int highFre,int lowFre);

    FSL_CORE_API Phase DualFreNumberTheoreticalUnwrap(const std::vector<Phase>& wraps, const std::unordered_map<int, int>& lut,const int highFre,const int lowFre);

    FSL_CORE_API std::pair<Phase,Phase> DualFreNumberTheoreticalUnwrapAdValue(const std::vector<Phase>& wraps, const std::unordered_map<int, int>& lut,const int highFre,const int lowFre);

}


#endif

