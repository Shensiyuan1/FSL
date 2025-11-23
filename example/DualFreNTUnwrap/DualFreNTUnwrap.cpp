#include <fsl/core.h>
#include <opencv2/opencv.hpp>



int main()
{

    std::cout<<"Hierarchical Unwrap Test"<<std::endl;

    std::vector<Fringe::RawImg> imgs;
    const std::string folder = "./img/";

    // --- third wrap caculation ---
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
    std::vector<Fringe::Phase> wraps;
    wraps.push_back(w3);
    wraps.push_back(w4);

    std::unordered_map<int,int> lut = Fringe::DualFreNumberTheoreticalLUT(14,57);

    // --- unwrap phase calculation ---
    //Fringe::Phase unwrap = Fringe::DualFreNumberTheoreticalUnwrap(wraps,lut,14,57);

    // cpp 17
    //auto [unwrap, Value] = Fringe::DualFreNumberTheoreticalUnwrapAdValue(wraps,lut,14,57);

    //
    auto result = Fringe::DualFreNumberTheoreticalUnwrapAdValue(wraps,lut,14,57);
    Fringe::Phase unwrap = result.first;
    Fringe::Phase Value = result.second;

    cv::Mat unwrapImg1(unwrap.rows, unwrap.cols, CV_64F, unwrap.data.data());
    double minVal, maxVal;
    cv::minMaxLoc(unwrapImg1, &minVal, &maxVal);
    cv::normalize(unwrapImg1, unwrapImg1, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("unwrap", unwrapImg1);
    cv::waitKey(0);


    auto sr = Fringe::segphase(unwrap,unwrap.B,"modulation",3);
    Fringe::Phase segment = sr.first;
    auto segNumber = sr.second;

    Fringe::optPhaseBYvalue(unwrap,Value,segment,segNumber);


    cv::Mat w3Img(w3.rows, w3.cols, CV_64F, w3.data.data());
    w3Img.convertTo(w3Img, CV_8U, 255.0/2.0/M_PI);
    cv::imshow("w3", w3Img);
    cv::waitKey(0);

    cv::Mat w4Img(w4.rows, w4.cols, CV_64F, w4.data.data());
    w4Img.convertTo(w4Img, CV_8U, 255.0/2.0/M_PI);
    cv::imshow("w4", w4Img);
    cv::waitKey(0);

    cv::Mat unwrapImg(unwrap.rows, unwrap.cols, CV_64F, unwrap.data.data());
    cv::minMaxLoc(unwrapImg, &minVal, &maxVal);
    cv::normalize(unwrapImg, unwrapImg, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("unwrap", unwrapImg);
    cv::waitKey(0);

    cv::Mat valueimg(Value.rows, Value.cols, CV_64F, Value.data.data());
    cv::minMaxLoc(valueimg, &minVal, &maxVal);
    cv::normalize(valueimg, valueimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("Value", valueimg);
    cv::waitKey(0);

    cv::Mat semgnetimg(Value.rows, Value.cols, CV_64F, segment.data.data());
    cv::minMaxLoc(semgnetimg, &minVal, &maxVal);
    cv::normalize(semgnetimg, semgnetimg, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imshow("Value", semgnetimg);
    cv::waitKey(0);


    return 0;
}
