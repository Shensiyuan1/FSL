#include "fsl/cuda.cuh"

__global__ void PhaseShiftCuda(const uint8_t* dev_img, const int step, 
                                const int width, const int height, 
                                double* dev_wrap, double* dev_wrapB) {

        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        if (x < width*height)
        {
            double M = 0;
            double D = 0;
            for (int i = 0; i < step; ++i)
            {
                M += (double)dev_img[x+height*width*i]*sin(2.0*M_PI*i/step);
                D += (double)dev_img[x+height*width*i]*cos(2.0*M_PI*i/step);
            }

            dev_wrap[x] = atan2(-M,D);
            if (dev_wrap[x] < 0.0) dev_wrap[x] += 2.0*M_PI;

            dev_wrapB[x] = hypot(-M,D)*2.0/step;
        }
}

__global__ void PhaseShiftCudaPreCompute(const uint8_t* dev_img, const int step, 
                                const int width, const int height, 
                                double* dev_wrap, double* dev_wrapB,
                                double* sin_table, double* cos_table) {

        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        if (x < width*height)
        {
            double M = 0;
            double D = 0;
            for (int i = 0; i < step; ++i)
            {
                double v = __ldg(&dev_img[x + i * height*width]); 
                M += (double)dev_img[x+height*width*i]*sin_table[i];
                D += (double)dev_img[x+height*width*i]*cos_table[i];
            }

            dev_wrap[x] = atan2(-M,D);
            if (dev_wrap[x] < 0.0) dev_wrap[x] += 2.0*M_PI;

            dev_wrapB[x] = hypot(-M,D)*2.0/step;
        }
}

__global__ void PhaseShiftCudaContinue(const uint8_t* dev_img, const int step, 
                                const int width, const int height, 
                                double* dev_wrap, double* dev_wrapB,
                                double* sin_table, double* cos_table) {

        const int x = blockIdx.x * blockDim.x + threadIdx.x;
        if (x < width*height)
        {
            double M = 0;
            double D = 0;
            for (int i = 0; i < step; ++i)
            {
                M += (double)dev_img[x*step+i]*sin_table[i];
                D += (double)dev_img[x*step+i]*cos_table[i];
            }
            dev_wrap[x] = atan2(-M,D);
            if (dev_wrap[x] < 0.0) dev_wrap[x] += 2.0*M_PI;
            dev_wrapB[x] = hypot(-M,D)*2.0/step;

        }
    }


namespace Fringe
{

    void precomputeTrigTables(double* sin_table, double* cos_table, int step) {
        for (int i = 0; i < step; i++) {
            double angle = 2.0 * M_PI * i / step;
            sin_table[i] = sin(angle);
            cos_table[i] = cos(angle);
        }
    }

    void reorganizeImageData(const std::vector<RawImg>& imgs, 
                            uint8_t* reorganizedData,
                            int baseWidth, int baseHeight, int step) {
        
        for (int y = 0; y < baseHeight; y++) {
            for (int x = 0; x < baseWidth; x++) {
                for (int i = 0; i < step; i++) {
                    int originalIndex = y * baseWidth + x;
                    int newIndex = (x + y * baseWidth) * step + i; // 0 1 2 3 ->
                    reorganizedData[newIndex] = imgs[i].data[originalIndex];
                }
            }
        }
    }

    void resavetocpu(const std::vector<RawImg>& imgs, uint8_t* reorganizedData,
                            int baseWidth, int baseHeight, int step) {
        
        for (int i = 0; i < step; i++)
        {
            memcpy(reorganizedData + i * baseWidth * baseHeight, imgs[i].data.data(), baseWidth * baseHeight);
        }
 
        }


    Phase StandardPhaseShiftGPU(const std::vector<RawImg>& imgs,bool cacuB)
    {

        if (imgs.empty()) {
            std::clog << "Warning: empty images.\n";
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;

            }

        const int baseWidth = imgs[0].rows;
        const int baseHeight = imgs[0].cols;

        for (int i = 0; i < imgs.size(); ++i)
        {
            if (imgs[i].ch != 1) 
            {
                std::cerr << "Warning: image " << i << " is not grayscale.\n";
                Phase empty;
                empty.rows = 0;
                empty.cols = 0;
                return empty;
            }
            if (imgs[i].rows != baseWidth || imgs[i].cols != baseHeight || imgs[i].data.size() != baseWidth*baseHeight)
            {
                std::cerr << "Warning: image " << i << " has different size.\n";
                Phase empty;
                empty.rows = 0;
                empty.cols = 0;
                return empty;
            }
        }

        
        int step = imgs.size();

        // auto lut_start = std::chrono::high_resolution_clock::now();
        uint8_t* rData = new uint8_t[step*baseWidth * baseHeight];
        resavetocpu(imgs, rData, baseWidth, baseHeight, step);
        //reorganizeImageData(imgs, rData, baseWidth, baseHeight, step);
       
        
        Phase wrap;
        wrap.rows = baseWidth;
        wrap.cols = baseHeight;
        wrap.data.resize(baseWidth*baseHeight);

        uint8_t* dev_img;
        double* dev_wrap;
        double* dev_wrapB;

        if (cacuB) {
            wrap.B.resize(baseWidth * baseHeight);
        }

        cudaError_t err = cudaMalloc((void**)&dev_img, step*baseWidth * baseHeight * sizeof(uint8_t));
        
        if (err != cudaSuccess) {
            std::cerr << "Error: cudaMalloc failed: " << cudaGetErrorString(err) << std::endl;
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }

        err = cudaMalloc((void**)&dev_wrap, baseWidth * baseHeight * sizeof(double));
        if (err != cudaSuccess) {
            std::cerr << "Error: cudaMalloc failed: " << cudaGetErrorString(err) << std::endl;
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }
        
        err = cudaMalloc((void**)&dev_wrapB, baseWidth * baseHeight * sizeof(double));
        if (err != cudaSuccess) {
            std::cerr << "Error: cudaMalloc failed: " << cudaGetErrorString(err) << std::endl;
            Phase empty;
            empty.rows = 0;
            empty.cols = 0;
            return empty;
        }


        // for (int i = 0; i < step; ++i)
        // {
        //     cudaMemcpy(dev_img + i*baseWidth * baseHeight, imgs[i].data.data(), baseWidth * baseHeight * sizeof(uint8_t), cudaMemcpyHostToDevice);
        // }
        
        cudaMemcpy(dev_img, rData, step*baseWidth * baseHeight * sizeof(uint8_t), cudaMemcpyHostToDevice);

        double* sin_table = new double[step];
        double* cos_table = new double[step];
        precomputeTrigTables(sin_table, cos_table, step);
        double* dev_sin_table;
        double* dev_cos_table;

        cudaMalloc((void**)&dev_sin_table, step * sizeof(double));
        cudaMemcpy(dev_sin_table, sin_table, step * sizeof(double), cudaMemcpyHostToDevice);

        cudaMalloc((void**)&dev_cos_table, step * sizeof(double));
        cudaMemcpy(dev_cos_table, cos_table, step * sizeof(double), cudaMemcpyHostToDevice);

        constexpr int threadNum = CUDA_ACC_BLOCK_SIZE;  // 256
        int blockNum = (baseHeight*baseWidth*step + threadNum - 1) / threadNum;

        PhaseShiftCuda<<<blockNum, threadNum>>>(dev_img, step, baseWidth, baseHeight, dev_wrap, dev_wrapB);

        //PhaseShiftCudaPreCompute<<<blockNum, threadNum>>>(dev_img, step, baseWidth, baseHeight, 
                                     //dev_wrap, dev_wrapB, dev_sin_table, dev_cos_table);

        
        //PhaseShiftCudaContinue<<<blockNum, threadNum>>>(dev_img, step, baseWidth, baseHeight, 
                                        //dev_wrap, dev_wrapB, dev_sin_table, dev_cos_table);
        
        cudaMemcpy(wrap.data.data(), dev_wrap, baseWidth * baseHeight * sizeof(double), cudaMemcpyDeviceToHost);
        if (cacuB) {
            cudaMemcpy(wrap.B.data(), dev_wrapB, baseWidth * baseHeight * sizeof(double), cudaMemcpyDeviceToHost);
        }

        cudaFree(dev_img);
        cudaFree(dev_wrap);
        cudaFree(dev_wrapB);
        cudaFree(dev_sin_table);
        cudaFree(dev_cos_table);
        delete[] sin_table;
        delete[] cos_table;
        delete[] rData;
        // auto lut_end = std::chrono::high_resolution_clock::now();
        // auto lut_duration = std::chrono::duration_cast<std::chrono::milliseconds>(lut_end - lut_start);
        // std::cout << "reorgnize time in " << lut_duration.count() << " ms" << std::endl;

        return wrap;


    }


}