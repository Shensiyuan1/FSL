#ifndef UTILS_H
#define UTILS_H

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


}


#endif 
