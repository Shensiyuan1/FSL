#include "widget.h"

PointWindow::PointWindow(QWidget *parent)
    : QWidget(parent)
{

    setWindowTitle("Point Window");
    setMinimumSize(800, 600);

    cameraIntrinsicLabel = new QLabel(this);
    cameraIntrinsicLabel->setFont(QFont("Courier New", 10));
    cameraIntrinsicLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    cameraIntrinsicLabel->setAlignment(Qt::AlignCenter);
    //cameraIntrinsicLabel->setFixedHeight(50);


    projectionMLabel = new QLabel(this);
    projectionMLabel->setFont(QFont("Courier New", 10));
    projectionMLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    projectionMLabel->setAlignment(Qt::AlignCenter);
    //projectionMLabel->setFixedHeight(50);

    QHBoxLayout* paramlayout = new QHBoxLayout();
    paramlayout->addStretch();
    paramlayout->addWidget(cameraIntrinsicLabel);
    paramlayout->addWidget(projectionMLabel);
    paramlayout->addStretch();

    pointCloudWidget = new PointCloudWidget(this);

    QVBoxLayout* mainlayout = new QVBoxLayout(this);
    mainlayout->addLayout(paramlayout,1);
    mainlayout->addWidget(pointCloudWidget,9);
    //mainlayout->addStretch();

    Fringe::SystemParams sysparams;
    SetSystemParams(sysparams);
    Fringe::Phase unwrap = UnwrapPhase();
    Fringe::Phase pixel = Fringe::Phase2Projection(unwrap,57,912);
    Fringe::PointCloud cloud = Fringe::UnidirectionReconstruction(pixel,sysparams,"modulation",pixel.B,3.0,"matlab");
    //bool succ = Fringe::Cloud2TxT(cloud,"./config/pointcloud.txt");

    std::cout << "Point count:" << cloud.points.size() << std::endl;

    pointCloudWidget->setCameraIntrinsics(sysparams, pixel.cols, pixel.rows);
    pointCloudWidget->setPointCloud(cloud);
}

PointWindow::~PointWindow()
{


}


void PointWindow::SetSystemParams(Fringe::SystemParams &params)
{
    bool error = Fringe::LoadCameraParams("./config/cameraIntrinsic.txt", params);
    if(!error){
        QMessageBox::critical(
            nullptr,  // parent widget
            "Camera Parameter Error",
            "Failed to load camera parameters!\n"
            "Please check if './config/cameraIntrinsic.txt' exists\n"
            "and contains exactly 9 numeric values."
        );

        return;
    }
    else
    {
        QStringList rows;
        for (int i = 0; i < 3; ++i) {
            QString row = "[";
            for (int j = 0; j < 3; ++j) {
                row += QString("%1").arg(params.cameraparam[i][j], 8, 'g', 5);
            }
            row += " ]";
            rows << row;
        }

        QString label = "Camera Intrinsic:";
        
        QString result;
        result += QString(" ").repeated(label.size()) + " " + rows[0] + "\n";
        result += label + " " + rows[1] + "\n";   
        result += QString(" ").repeated(label.size()) + " " + rows[2];
        cameraIntrinsicLabel->setText(result);
    }
        
    error = Fringe::LoadProjectionMParams("./config/projection_m.txt", params);
    if(!error){
        QMessageBox::critical(
            nullptr,  // parent widget
            "Projection Parameter Error",
            "Failed to load projection parameters M!\n"
            "Please check if './config/projection_m.txt' exists\n"
            "and contains exactly 7 numeric values."
        );
        return;
    }
    else
    {
        QString title = "Projection Parameter M:";
        QString line2; 
        QString line3; 

        for (int i = 0; i < 7; ++i) {
            QString numStr = QString("%1").arg(params.projection_m[i], 8, 'e', 2);
            if (i < 3) {
                line2 += numStr;
            } else {
                line3 += numStr;
            }
        }

        projectionMLabel->setText(title + "\n" + line2 + "\n" + line3);
    }
}   

Fringe::Phase PointWindow::UnwrapPhase()
{
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
    return unwrap;

}
