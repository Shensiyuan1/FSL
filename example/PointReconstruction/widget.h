#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QMessageBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <fsl/core.h>
#include <opencv2/opencv.hpp>
#include "PointCloudWidget.h"

class PointWindow : public QWidget
{
    Q_OBJECT

public:
    PointWindow(QWidget *parent = nullptr);
    ~PointWindow();

    void SetSystemParams(Fringe::SystemParams &params);
    Fringe::Phase UnwrapPhase();

private:
    QLabel *cameraIntrinsicLabel;
    QLabel *projectionMLabel;
    PointCloudWidget *pointCloudWidget;


};







#endif // 