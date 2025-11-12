#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QMessageBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <fsl/core.h>

class PointWindow : public QWidget
{
    Q_OBJECT

public:
    PointWindow(QWidget *parent = nullptr);
    ~PointWindow();

    void PointCloudCaculate();

private:
    QLabel *cameraIntrinsicLabel;
    QLabel *projectionMLabel;

};







#endif // 