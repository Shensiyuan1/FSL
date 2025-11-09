#include <iostream>
#include <fsl/core.h>
#include <opencv2/opencv.hpp>



int main()
{
    std::cout<<"Hierarchical Unwrap Test"<<std::endl;

    std::vector<Fringe::RawImg> imgs;
    const std::string folder = "./img/";
    // --- first wrap caculation ---
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
    Fringe::Phase w1 = Fringe::Standardphaseshift(imgs,true);

    // --- second wrap caculation ---
    imgs.clear();
    for (int i = 0; i < 3; ++i) 
    {
        std::string file = folder + std::to_string(i+3) + ".bmp";
        cv::Mat m = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (m.empty()) {
            std::cerr << "Warning: skip " << file << "\n";
            continue;
        }
        if (m.empty()) continue;
        imgs.push_back({m.rows, m.cols, m.channels(),
                        std::vector<uint8_t>(m.data, m.data + m.total() * m.channels())});
    }
    Fringe::Phase w2 = Fringe::Standardphaseshift(imgs,true);

    // --- third wrap caculation ---
    imgs.clear();
    for (int i = 0; i < 3; ++i) 
    {
        std::string file = folder + std::to_string(i+6) + ".bmp";
        cv::Mat m = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (m.empty()) {
            std::cerr << "Warning: skip " << file << "\n";
            continue;
        }
        if (m.empty()) continue;
        imgs.push_back({m.rows, m.cols, m.channels(),
                        std::vector<uint8_t>(m.data, m.data + m.total() * m.channels())});
    }
    Fringe::Phase w3 = Fringe::Standardphaseshift(imgs,true);

    // --- fourth wrap caculation ---
    imgs.clear();
    for (int i = 0; i < 20; ++i) 
    {
        std::string file = folder + std::to_string(i+9) + ".bmp";
        cv::Mat m = cv::imread(file, cv::IMREAD_GRAYSCALE);
        if (m.empty()) {
            std::cerr << "Warning: skip " << file << "\n";
            continue;
        }
        if (m.empty()) continue;
        imgs.push_back({m.rows, m.cols, m.channels(),
                        std::vector<uint8_t>(m.data, m.data + m.total() * m.channels())});
    }
    Fringe::Phase w4 = Fringe::Standardphaseshift(imgs,true);

    // --- all wrap ---
    double ratio[3] = {8,2,57.0/14.0}; 
    std::vector<Fringe::Phase> wraps;
    wraps.push_back(w1);
    wraps.push_back(w2);
    wraps.push_back(w3);
    wraps.push_back(w4);

    // --- unwrap phase calculation ---
    Fringe::Phase unwrap = Fringe::HierarchicalUnwrap(wraps,ratio,3);

    // --- show all phase img ---
    cv::Mat w1Img(w1.rows, w1.cols, CV_64F, w1.data.data());
    w1Img.convertTo(w1Img, CV_8U, 255.0/2.0/M_PI);
    cv::imshow("w1", w1Img);
    cv::waitKey(0);

    cv::Mat w2Img(w2.rows, w2.cols, CV_64F, w2.data.data());
    w2Img.convertTo(w2Img, CV_8U, 255.0/2.0/M_PI);
    cv::imshow("w2", w2Img);
    cv::waitKey(0);

    cv::Mat w3Img(w3.rows, w3.cols, CV_64F, w3.data.data());
    w3Img.convertTo(w3Img, CV_8U, 255.0/2.0/M_PI);
    cv::imshow("w3", w3Img);
    cv::waitKey(0);

    cv::Mat w4Img(w4.rows, w4.cols, CV_64F, w4.data.data());
    w4Img.convertTo(w4Img, CV_8U, 255.0/2.0/M_PI);
    cv::imshow("w4", w4Img);
    cv::waitKey(0);

    cv::Mat unwrapImg(unwrap.rows, unwrap.cols, CV_64F, unwrap.data.data());
    double minVal, maxVal;
    cv::minMaxLoc(unwrapImg, &minVal, &maxVal);
    cv::normalize(unwrapImg, unwrapImg, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("unwrap", unwrapImg);
    cv::waitKey(0);


    return 0;
}