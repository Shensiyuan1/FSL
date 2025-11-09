#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>  
#include <iomanip>
#include "fsl/core.h"


int main()
{
    std::vector<Fringe::RawImg> imgs;
    const std::string folder = "./img/";

    for (int i = 0; i < 3; ++i) 
    {
        std::string file = folder + std::to_string(i) + ".bmp";
        cv::Mat m = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (m.empty()) {
            std::cerr << "Warning: skip " << file << "\n";
            continue;
        }
        if (m.empty()) continue;
        imgs.push_back({m.rows, m.cols, m.channels(),
                        std::vector<uint8_t>(m.data, m.data + m.total() * m.channels())});
    }

    std::cout << "Loaded " << imgs.size() << " images.\n";
    if (imgs.empty()) {
        std::cerr << "Error: no valid images loaded.\n";
        return 1;
    }

    Fringe::LUTP lut;
    Fringe::ThreeStepLUTGenerate(lut);

    Fringe::Phase w1 = Fringe::LUT3StepPhaseshift(imgs,true,lut);

    cv::Mat w1Img(w1.rows, w1.cols, CV_64F, w1.data.data());
    w1Img.convertTo(w1Img, CV_8U, 255.0/2.0/M_PI);
    cv::imshow("w1", w1Img);
    cv::waitKey(0);

    return 0;
}