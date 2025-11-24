#include "fsl/wrap.h"


namespace Fringe
{
    namespace{
        Phase createEmptyPhase() 
        {
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
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

}