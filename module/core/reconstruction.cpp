#include "fsl/reconstruction.h"

namespace Fringe
{
    namespace 
    {
        Phase createEmptyPhase() 
        {
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }

        PointCloud createEmptyPointCloud() 
        {
            PointCloud empty;
            return empty;
        }

    }
    
    Phase Phase2Projection(const Phase& unwrap,const double level,const int projection_pixel)
    {
        if (unwrap.data.empty()) {
            std::cerr << "Warning: empty unwrap.\n";
            return createEmptyPhase();
        }
        if (projection_pixel < 1) {
            std::cerr << "Warning: projection pixel is less than 1.\n";
            return createEmptyPhase();
        }
        Phase projection;
        projection.rows = unwrap.rows;
        projection.cols = unwrap.cols;
        projection.data.resize(projection.rows*projection.cols);

        const double scale = projection_pixel / (2.0 * M_PI * level);

        for (int i = 0; i < projection.rows*projection.cols; ++i) {
            projection.data[i] = unwrap.data[i]*scale;
        }
        if (!unwrap.B.empty()) {
            projection.B = unwrap.B;
        }
        return projection;
    }

    PointCloud UnidirectionReconstruction(const Phase& unwrap,
                                        const SystemParams& sysparams,
                                        const  std::string mode,
                                        const std::vector<double>& mask,
                                        const double thresh,
                                        const std::string platform)
    {
        if (unwrap.data.empty()) {
            std::cerr << "Warning: empty unwrap.\n";
            return createEmptyPointCloud();
        }

        const size_t N = unwrap.data.size();
        std::vector<double> local_mask;

        if (mode == "all") {
            local_mask.assign(N, 1.0);
        }
        else if (mode == "unwrap") {
            local_mask.reserve(N);
            for (size_t i = 0; i < N; ++i) {
                local_mask[i] = (unwrap.data[i] != 0.0) ? 1.0 : 0.0;
            }
        }
        else if (mode == "modulation" || mode == "mask") {
            if (mask.size() != N) {
                std::cerr << "Warning: mask size (" << mask.size()
                        << ") does not match unwrap size (" << N << ").\n";
                return createEmptyPointCloud();
            }

            local_mask.reserve(N);
            if (mode == "modulation") {
                for (size_t i = 0; i < N; ++i) {
                    local_mask[i] = (mask[i] >= thresh) ? 1.0 : 0.0;
                }
            } else { 
                local_mask = mask;
            }
        }
        else {
            std::cerr << "Error: unsupported mode '" << mode << "'. Supported: all, unwrap, modulation, mask.\n";
            return createEmptyPointCloud();
        }

        PointCloud cloud;
        for (int i = 0; i < unwrap.data.size(); ++i) {
            if(local_mask[i] == 0.0) continue;
            Point p;
            
            double bias = (platform == "matlab") ? 1.0 : 0.0;

            double vc = i / unwrap.cols + bias;      //cols 0-608 
            double uc = i % unwrap.cols + bias;      // vc 1-800
            
            //std::cout << "uc: " << uc << " vc: " << vc << "\n";

            double b3 = sysparams.projection_m[3] * unwrap.data[i] - 1;
            double k1 = sysparams.cameraparam[0][0] - sysparams.cameraparam[2][0] * uc;
            double k3 = sysparams.cameraparam[0][2] - sysparams.cameraparam[2][2] * uc;
            double k5 = sysparams.cameraparam[1][1] - sysparams.cameraparam[2][1] * vc;
            double k6 = sysparams.cameraparam[1][2] - sysparams.cameraparam[2][2] * vc;
            double k7 = sysparams.projection_m[4] - sysparams.projection_m[0] * unwrap.data[i];
            double k8 = sysparams.projection_m[5] - sysparams.projection_m[1] * unwrap.data[i];
            double k9 = sysparams.projection_m[6] - sysparams.projection_m[2] * unwrap.data[i];

            double fz = b3;
            double fm = k9-(k3*k7/k1)-(k8*k6/k5);
            p.z = fz/fm;
            p.x = -k3*p.z/k1;
            p.y = -k6*p.z/k5;
            
            cloud.points.push_back(p); 
        }

        return cloud;
    }

    bool LoadProjectionMParams(const std::string& filename, SystemParams& sysparams) 
    {
        std::ifstream profile(filename.c_str());
        if (!profile.is_open()) {
            std::cerr << "Error: failed to open " << filename << std::endl;
            return false;
        }

        // read all params to temp vector
        std::vector<double> all_params;
        double temp;
        while (profile >> temp) {
            all_params.push_back(temp);
        }
        profile.close();
        if (all_params.size() != 7) {
            std::cerr << "Error: wrong number of parameters in " << filename << std::endl;
            std::cerr << "Found " << all_params.size() << " values, expected exactly 7" << std::endl;
            return false;
        }
        for (int i = 0; i < 7; ++i) {
            sysparams.projection_m[i] = all_params[i];
        }
        
        return true;
    }

    bool LoadProjectionParams(const std::string& filename, SystemParams& sysparams) 
    {
        std::ifstream file(filename.c_str());
        if (!file.is_open()) {
            std::cerr << "Error: failed to open " << filename << std::endl;
            return false;
        }

        std::vector<double> all_params;
        double temp;
        while (file >> temp) {
            all_params.push_back(temp);
        }
        file.close();
        
        if (all_params.size() != 9) {
            std::cerr << "Error: wrong number of parameters in " << filename << std::endl;
            std::cerr << "Found " << all_params.size() << " values, expected exactly 9" << std::endl;
            return false;
        }
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                sysparams.projectionparam[i][j] = all_params[i * 3 + j];
            }
        }
        return true;
    }

    bool LoadCameraParams(const std::string& filename, SystemParams& sysparams) 
    {
        std::ifstream file(filename.c_str());
        if (!file.is_open()) {
            std::cerr << "Error: failed to open " << filename << std::endl;
            return false;
        }

        std::vector<double> all_params;
        double temp;
        while (file >> temp) {
            all_params.push_back(temp);
        }
        file.close();
        
        if (all_params.size() != 9) {
            std::cerr << "Error: wrong number of parameters in " << filename << std::endl;
            std::cerr << "Found " << all_params.size() << " values, expected exactly 9" << std::endl;
            return false;
        }
        
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                sysparams.cameraparam[i][j] = all_params[i * 3 + j];
            }
        }
        return true;
    }

    bool Cloud2TxT(const PointCloud& cloud,const std::string filename)
    {
        
        if (cloud.points.empty()) {
            std::clog << "Warning: empty cloud.\n";
            return false;
        }

        std::ofstream file(filename);
        file << std::scientific << std::setprecision(std::numeric_limits<double>::max_digits10);
        for (const auto& point : cloud.points) {
                file << point.x << " " << point.y << " " << point.z << "\n";
            }
            
        file.close();

        return true;
    }

    
}