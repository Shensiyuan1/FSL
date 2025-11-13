#ifndef POINTCLOUDWIDGET_H
#define POINTCLOUDWIDGET_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QMatrix4x4>
#include <QPoint>
#include <QVector3D>
#include <QMouseEvent>
#include <QWheelEvent>

#include <fsl/core.h>


class PointCloudWidget : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT

public:
    explicit PointCloudWidget(QWidget *parent = nullptr);

    void setPointCloud(const Fringe::PointCloud& cloud);

    void setCameraIntrinsics(const Fringe::SystemParams& params, int imgWidth, int imgHeight);


protected:
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;

    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void setupShaders();
    void updateProjection();
    void uploadBuffer();

    QVector3D computePointCloudCenterFromBuffer();
    void computePointCloudZRangeFromBuffer(float &zMin, float &zMax);

     // GL objects
    QOpenGLShaderProgram *program_{nullptr};
    QOpenGLBuffer vbo_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vao_;
    int pointCount_{0};

    std::vector<float> cpuBuffer_;

    // transforms / camera
    QMatrix4x4 projection_;
    QMatrix4x4 view_;
    QMatrix4x4 model_;
    // draw settings
    float pointSize_{2.0f};   //point size 
    bool bufferDirty_{false}; // point need update or not

    float fx_{0}, fy_{0}, cx_{0}, cy_{0};
    int imgWidth_{0}, imgHeight_{0};

    QVector3D center_ = QVector3D(0,0,0);   
    float distance_ = 1.0f;                
    float pnear = 0.1f, pfar = 1000.0f;
    float zMin_, zMax_;

    float yaw_ = 0.0f;      
    float pitch_ = 0.0f;     
    float panX_ = 0.0f;      
    float panY_ = 0.0f;
    QPoint lastMousePos_;
};


#endif // POINTCLOUDWIDGET_H
