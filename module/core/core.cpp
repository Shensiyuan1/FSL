#include "fsl/core.h"

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
    
    Phase Standardphaseshift(const std::vector<RawImg>& imgs,bool cacuB)
    {
        if (imgs.empty()) {
        std::clog << "Warning: empty images.\n";
        return createEmptyPhase();
        }

        const int baseWidth = imgs[0].rows;
        const int baseHeight = imgs[0].cols;

        for (int i = 0; i < imgs.size(); ++i)
        {
            if (imgs[i].ch != 1) 
            {
                std::cerr << "Warning: image " << i << " is not grayscale.\n";
                return createEmptyPhase();
            }
            if (imgs[i].rows != baseWidth || imgs[i].cols != baseHeight || imgs[i].data.size() != baseWidth*baseHeight)
            {
                std::cerr << "Warning: image " << i << " has different size.\n";
                return createEmptyPhase();
            }
        }
        
        Phase wrap;
        wrap.rows = baseWidth;
        wrap.cols = baseHeight;
        wrap.data.resize(baseWidth*baseHeight);

        if (cacuB) {
            wrap.B.resize(baseWidth * baseHeight);
        }

        int step = imgs.size();
        if (step < 3)
        {
            std::cerr << "Warning: too few images.\n";
            return createEmptyPhase();
        }

        
        std::cout<<"start standard "<< step <<"-step phaseshift!"<<std::endl;

        for (int y=0; y<baseHeight*baseWidth; ++y)
        {
            double M,D;
            M = 0.0;
            D = 0.0;

            for (int n=0; n<step; ++n)
            {
                M += (double)imgs[n].data[y]*sin(2.0*M_PI*n/step);
                D += (double)imgs[n].data[y]*cos(2.0*M_PI*n/step);
            }
            
            wrap.data[y] = atan2(-M,D);
            if (wrap.data[y] < 0.0) wrap.data[y] += 2.0*M_PI;
           
            
            if (cacuB) {
                wrap.B[y] = hypot(-M,D)*2.0/step;
            }
        }

        return wrap;
    }

    Phase CacuQuality(const Phase& wrap,bool fourOeight)
    {
        if (wrap.data.empty()) {
            std::cerr << "Warning: empty wrap.\n";
            return createEmptyPhase();
        }

        if (wrap.data.size() != wrap.rows * wrap.cols) {
            std::cerr << "Warning: wrap data size is not equal to rows * cols.\n";
            return createEmptyPhase();
        }
 
        Phase quality;
        quality.rows = wrap.rows;
        quality.cols = wrap.cols;
        quality.data.resize(wrap.data.size());

        const int XX_4[4] = {-1, 0, 1, 0};  
        const int YY_4[4] = {0, 1, 0, -1};
        const int XX_8[8] = {-1, 0, 1, 0, -1, 1, 1, -1};  
        const int YY_8[8] = {0, 1, 0, -1, 1, 1, -1, -1};
        
  
        const int* XX = fourOeight ? XX_8 : XX_4;
        const int* YY = fourOeight ? YY_8 : YY_4;
        const int numDirections = fourOeight ? 8 : 4;

        for (int i = 0; i < quality.rows * quality.cols; i++) {
            int y = i / quality.cols;     
            int x = i % quality.cols;    
                
            if (wrap.data[i] != 0) {
                float max_diff = 0.0f;
                    
                for (int cc = 0; cc < numDirections; cc++) {
                    int nx = x + XX[cc];
                    int ny = y + YY[cc];
                        

                    if (nx >= 0 && nx < wrap.cols && ny >= 0 && ny < wrap.rows) {
                        int newindex = ny * wrap.cols + nx;

                        float diff = std::abs(wrap.data[newindex] - wrap.data[i]) / (2 * M_PI);
                        if (diff > max_diff) {
                            max_diff = diff;
                        }
                    }
                }
                    
                quality.data[i] = 1.0f - max_diff;
            } else {

                quality.data[i] = 1.0f;
            }
        }
        return quality;
    }
    
    Phase MD2Phase(const Phase& M,const Phase& D,bool cacuB)
    {
        const std::size_t N = M.data.size();
        if (N == 0 || N != D.data.size() ||
        M.rows != D.rows || M.cols != D.cols) {
        std::cerr << "Warning: invalid M/D.\n";
        return createEmptyPhase();
        }

        Phase wrap;
        wrap.rows = M.rows;
        wrap.cols = M.cols;
        wrap.data.resize(N);
        if (cacuB) wrap.B.resize(N);

        for (std::size_t i = 0; i < N; ++i) {
            const double yy = -M.data[i];
            const double xx =  D.data[i];
            wrap.data[i] = atan2(yy, xx);
            if (wrap.data[i] < 0.0) wrap.data[i] += 2.0*M_PI;
            if (cacuB) {
                wrap.B[i] =  2.0 * hypot(yy, xx);
            }
        }

        return wrap;

    }

    Phase HierarchicalUnwrap(const std::vector<Phase>& wraps,double* ratio,const int ratio_count)
    {
        if (wraps.empty() || !ratio || ratio_count < 1 ||
        ratio_count + 1 != static_cast<int>(wraps.size()))
        {
            std::cerr << "Warning: invalid args.\n";
            return createEmptyPhase();
        }

        Phase unwrap;
        unwrap.rows = wraps[0].rows;
        unwrap.cols = wraps[0].cols;
        unwrap.data = wraps[0].data;
        
        const double scale = 1.0/(2.0*M_PI);

        for (int i = 1; i < wraps.size(); ++i) {
            for (int n = 0; n < unwrap.data.size(); ++n) {
                unwrap.data[n] = wraps[i].data[n] + 2.0*M_PI*round((ratio[i-1]*unwrap.data[n]-wraps[i].data[n])*scale);
            } 
        }

        if (!wraps[wraps.size()-1].B.empty()) {
            unwrap.B = wraps[wraps.size()-1].B;
        }

        return unwrap;
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

        std::vector<double> local_mask = mask;

        //caculate all points
        if (mode =="all") {
            local_mask.clear();
            local_mask.resize(unwrap.data.size(),1.0);
        }

        // mask == modulation
        if (mode == "modulation" && mask.size() != unwrap.data.size()){
            std::cerr << "Warning: mask size is not equal to unwrap size.\n";
            return createEmptyPointCloud();
        }

        if(mode == "modulation" && mask.size() == unwrap.data.size())
        {
            for (int i = 0; i < unwrap.data.size(); ++i) {
                if (mask[i] < thresh) {
                    local_mask[i] = 0.0;
                }
            }
        }

        if (mode == "unwrap") {
            local_mask.clear();
            local_mask.resize(unwrap.data.size(),1.0);
            for (int i = 0; i < unwrap.data.size(); ++i) {
                if (unwrap.data[i] == 0.0) {
                    local_mask[i] = 0.0;
                }
            }
        }

        if (mode == "mask" && mask.size() != unwrap.data.size()){
            std::cerr << "Warning: mask size is not equal to unwrap size.\n";
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

    bool ThreeStepLUTGenerate(LUTP& lut)
    {
        // I1 - I2   -255 ~ 255  2I0 - I1 - I2  -510 ~ 510
        lut.phase.clear();
        lut.modulation.clear();

        lut.phase.resize(511*1021,0.0);
        lut.modulation.resize(511*1021,0.0);

        for (int i0 = 0; i0 < 256; ++i0) {
            for (int i1 = 0; i1 < 256; ++i1) {
                for (int i2 = 0; i2 < 256; ++i2) {
                    int idx = (i1-i2+255)*1021 + (2*i0-i1-i2+510);

                    double fz = sqrt(3.0) * (i1 - i2);
                    double fm = 2*i0-i1-i2;
                    double p = atan2(-fz,fm);
                    if (p < 0.0) p += 2.0*M_PI;
                    lut.phase[idx] = p;
                    lut.modulation[idx] = 2.0 * hypot(fz, fm)/3;
                }
            }
            
        }
        return true;
    }
    
    bool FourStepLUTGenerate(LUTP& lut)
    {
        //I1 -I3  -255 ~ 255    I0 - I2  -255 ~ 255
        lut.phase.clear();
        lut.modulation.clear(); 

        lut.phase.resize(511*511,0.0);
        lut.modulation.resize(511*511,0.0);

        for (int i0 = 0; i0 < 256; ++i0) {
            for (int i1 = 0; i1 < 256; ++i1) {
                for (int i2 = 0; i2 < 256; ++i2) {
                    for (int i3 = 0; i3 < 256; ++i3) {
                        int idx = (i1 - i3 + 255) * 511 + (i0 - i2 + 255);

                        double fz = i1 - i3;
                        double fm = i0 - i2;
                        double p = atan2(-fz,fm);
                        if (p < 0.0) p += 2.0*M_PI;
                        lut.phase[idx] = p;
                        lut.modulation[idx] = 2.0 * hypot(fz, fm)/4;
                    }
                }
            }
            
        }
        return true;
    }
    
    Phase LUT3StepPhaseshift(const std::vector<RawImg>& imgs,bool cacuB,const LUTP& lut)
    {
        if (imgs.empty()) {
            std::clog << "Warning: empty images.\n";
            return createEmptyPhase();
        }

        if (imgs.size() != 3)
        {
            std::cerr << "Warning: LUT3StepPhaseshift requires 3 images.\n";
            return createEmptyPhase();
        }   

        const int baseWidth = imgs[0].rows;
        const int baseHeight = imgs[0].cols;

        for (int i = 0; i < imgs.size(); ++i)
        {
            if (imgs[i].ch != 1) 
            {
                std::cerr << "Warning: image " << i << " is not grayscale.\n";
                return createEmptyPhase();
            }
            if (imgs[i].rows != baseWidth || imgs[i].cols != baseHeight || imgs[i].data.size() != baseWidth*baseHeight)
            {
                std::cerr << "Warning: image " << i << " has different size.\n";
                return createEmptyPhase();
            }
        }

        Phase wrap;
        wrap.rows = baseWidth;
        wrap.cols = baseHeight;
        wrap.data.resize(baseWidth*baseHeight,0.0);
        if (cacuB) wrap.B.resize(baseWidth*baseHeight,0.0);

        for (int i = 0; i < baseHeight*baseWidth; ++i) {
            int idx = (imgs[1].data[i]-imgs[2].data[i]+255)*1021 + (2*imgs[0].data[i]-imgs[1].data[i]-imgs[2].data[i]+510);
            wrap.data[i] = lut.phase[idx];
            if (cacuB) wrap.B[i] = lut.modulation[idx];
        }

        return wrap;

    }

    Phase LUT4StepPhaseshift(const std::vector<RawImg>& imgs,bool cacuB,const LUTP& lut)
    {
        if (imgs.empty()) {
            std::clog << "Warning: empty images.\n";
            return createEmptyPhase();
        }

        if (imgs.size() != 4)
        {
            std::cerr << "Warning: LUT3StepPhaseshift requires 4 images.\n";
            return createEmptyPhase();
        }   

        const int baseWidth = imgs[0].rows;
        const int baseHeight = imgs[0].cols;

        for (int i = 0; i < imgs.size(); ++i)
        {
            if (imgs[i].ch != 1) 
            {
                std::cerr << "Warning: image " << i << " is not grayscale.\n";
                return createEmptyPhase();
            }
            if (imgs[i].rows != baseWidth || imgs[i].cols != baseHeight || imgs[i].data.size() != baseWidth*baseHeight)
            {
                std::cerr << "Warning: image " << i << " has different size.\n";
                return createEmptyPhase();
            }
        }

        Phase wrap;
        wrap.rows = baseWidth;
        wrap.cols = baseHeight;
        wrap.data.resize(baseWidth*baseHeight,0.0);
        if (cacuB) wrap.B.resize(baseWidth*baseHeight,0.0);


        for (int i = 0; i < baseHeight*baseWidth; ++i) {
            int idx = (imgs[1].data[i]-imgs[3].data[i]+255)*511 + (imgs[0].data[i]-imgs[2].data[i]+255);
            wrap.data[i] = lut.phase[idx];
            if (cacuB) wrap.B[i] = lut.modulation[idx];
        }
        return wrap;    
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


