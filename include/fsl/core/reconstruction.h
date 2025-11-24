#ifndef RECONSTRUCTION_H
#define RECONSTRUCTION_H


#include "fsl/utils.h"

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
    FSL_CORE_API Phase Phase2Projection(const Phase& unwrap,const double level,const int projection_pixel);

    FSL_CORE_API PointCloud UnidirectionReconstruction(const Phase& unwrap,
                                                        const SystemParams& sysparams,
                                                        const std::string mode="all",
                                                        const std::vector<double>& mask={},
                                                        const double thresh=3.0,
                                                        const std::string platform="matlab");

    FSL_CORE_API bool LoadProjectionMParams(const std::string& filename, SystemParams& sysparams);

    FSL_CORE_API bool LoadProjectionParams(const std::string& filename, SystemParams& sysparams);

    FSL_CORE_API bool LoadCameraParams(const std::string& filename, SystemParams& sysparams);

    FSL_CORE_API bool  Cloud2TxT(const PointCloud& cloud,std::string filename);



}

#endif