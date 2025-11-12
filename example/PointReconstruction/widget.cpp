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
    cameraIntrinsicLabel->setFixedHeight(50);


    projectionMLabel = new QLabel(this);
    projectionMLabel->setFont(QFont("Courier New", 10));
    projectionMLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    projectionMLabel->setAlignment(Qt::AlignCenter);
    projectionMLabel->setFixedHeight(50);

    QHBoxLayout* paramlayout = new QHBoxLayout();
    paramlayout->addStretch();
    paramlayout->addWidget(cameraIntrinsicLabel);
    paramlayout->addWidget(projectionMLabel);
    paramlayout->addStretch();

    QVBoxLayout* mainlayout = new QVBoxLayout(this);
    mainlayout->addLayout(paramlayout);
    mainlayout->addStretch();


    PointCloudCaculate();


}

PointWindow::~PointWindow()
{


}

void PointWindow::PointCloudCaculate()
{
    Fringe::SystemParams sysparams;
    bool error = Fringe::LoadCameraParams("./config/cameraIntrinsic.txt", sysparams);
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
                row += QString("%1").arg(sysparams.cameraparam[i][j], 8, 'g', 5);
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
        
    error = Fringe::LoadProjectionMParams("./config/projection_m.txt", sysparams);
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
            QString numStr = QString("%1").arg(sysparams.projection_m[i], 8, 'e', 2);
            if (i < 3) {
                line2 += numStr;
            } else {
                line3 += numStr;
            }
        }

        projectionMLabel->setText(title + "\n" + line2 + "\n" + line3);

    }


    

    return ;


}
