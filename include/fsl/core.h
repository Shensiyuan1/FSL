#ifndef CORE_H
#define CORE_H

#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <string>
#include <iomanip>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#if defined(FSL_STATIC_DEFINE)
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
    struct RawImg {
        int rows;
        int cols;
        int ch;
        std::vector<uint8_t> data;
    };

    struct Phase {
        int rows;
        int cols;
        std::vector<double> data;
        std::vector<double> B;
    };

    struct Point{
        double x;
        double y;
        double z;
    };

    struct PointCloud{
        std::vector<Point> points;
    };

    struct SystemParams{
        double cameraparam[3][3];
        double projectionparam[3][3];
        double projection_m[7];
    };

    struct LUTP{
        std::vector<double> phase;
        std::vector<double> modulation;
    };


    FSL_CORE_API Phase Standardphaseshift(const std::vector<RawImg>& imgs,bool cacuB=false);

    FSL_CORE_API Phase CacuQuality(const Phase& wrap,bool fourOeight=false);

    FSL_CORE_API Phase MD2Phase(const Phase& M,const Phase& D,bool cacuB=false);

    FSL_CORE_API Phase HierarchicalUnwrap(const std::vector<Phase>& wraps,double* ratio,const int ratio_count);

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

    FSL_CORE_API bool ThreeStepLUTGenerate(LUTP& lut);

    FSL_CORE_API bool FourStepLUTGenerate(LUTP& lut);

    FSL_CORE_API Phase LUT3StepPhaseshift(const std::vector<RawImg>& imgs,bool cacuB,const LUTP& lut);

    FSL_CORE_API Phase LUT4StepPhaseshift(const std::vector<RawImg>& imgs,bool cacuB,const LUTP& lut);

    FSL_CORE_API bool  Cloud2TxT(const PointCloud& cloud,std::string filename);

}


#endif 
